//
// Copyright (c) 2008-2014 the Urho3D project.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//

#include "AnimatedModel.h"
#include "AnimationController.h"
#include "Camera.h"
#include "Character.h"
#include "CollisionShape.h"
#include "Controls.h"
#include "CoreEvents.h"
#include "Engine.h"
#include "FileSystem.h"
#include "Font.h"
#include "Input.h"
#include "Light.h"
#include "Material.h"
#include "Model.h"
#include "Octree.h"
#include "PhysicsWorld.h"
#include "ProcessUtils.h"
#include "Renderer.h"
#include "RigidBody.h"
#include "ResourceCache.h"
#include "Scene.h"
#include "StaticModel.h"
#include "Text.h"
#include "Touch.h"
#include "UI.h"
#include "Zone.h"

#include "AutoRunner.h"

#include "XMLFile.h"
#include "PhysicsEvents.h"
#include "SmoothedTransform.h"
#include "DebugNew.h"
#include "Log.h"

// Expands to this example's entry-point
DEFINE_APPLICATION_MAIN(AutoRunner)

AutoRunner::AutoRunner(Context* context) :
	Sample(context),
	touch_(new Touch(context)),
	yaw_(0.0f),
	pitch_(0.0f),
	drawDebug_(false),
	scoreText_(0)
{
	Character::RegisterObject(context);
}

void AutoRunner::Start()
{
	// Execute base class startup
	Sample::Start();
	
	// Init scene content
	InitScene();

	// Create Camera
	CreateCamera();

	// Create the controllable character
	CreateCharacter();

	// Create overlays
	CreateOverlays();

	// Subscribe to necessary events
	SubscribeToEvents();

	// Initialize touch input on Android & iOS
	if (GetPlatform() == "Android" || GetPlatform() == "iOS")
	{
		SetLogoVisible(false);
		touch_->InitTouchInput();
	}
}

void AutoRunner::InitScene()
{
	ResourceCache* cache = GetSubsystem<ResourceCache>();
	FileSystem* fs = GetSubsystem<FileSystem>();

	scene_ = new Scene(context_);
	File loadFile(context_, fs->GetProgramDir() + "Data/Scenes/AutoRunner.xml", FILE_READ);
	scene_->LoadXML(loadFile);
	CreateInitialLevel();

	// Pass knowledge of the scene & camera node to the Touch helper object.
	touch_->scene_ = scene_;
	touch_->cameraNode_ = cameraNode_;
}

void AutoRunner::CreateCharacter()
{
	ResourceCache* cache = GetSubsystem<ResourceCache>();

	Node* objectNode = scene_->CreateChild("Player");
	objectNode->SetPosition(Vector3(0.0f, 3.0f, -9.0f));
	Node* modelNode = objectNode->CreateChild("PlayerModel");
	modelNode->SetScale(Vector3(.4f, .4f, .4f));
	modelNode->SetRotation(Quaternion(180, Vector3::UP));

	// Create the rendering component + animation controller
	AnimatedModel* object = modelNode->CreateComponent<AnimatedModel>();
	object->SetModel(cache->GetResource<Model>("Models/vempire.mdl"));
	object->SetMaterial(cache->GetResource<Material>("Materials/Vempire.xml"));
	object->SetCastShadows(true);
	modelNode->CreateComponent<AnimationController>();

	String headName = "Bip001 Head";
	// Set the head bone for manual control
	object->GetSkeleton().GetBone(headName)->animated_ = false;

	// Create rigidbody, and set non-zero mass so that the body becomes dynamic
	RigidBody* body = objectNode->CreateComponent<RigidBody>();
	body->SetCollisionLayer(FLOOR_COLLISION_MASK|COIN_COLLISION_MASK);
	body->SetMass(1.0f);

	// Set zero angular factor so that physics doesn't turn the character on its own.
	// Instead we will control the character yaw manually
	body->SetAngularFactor(Vector3::ZERO);

	// Set the rigidbody to signal collision also when in rest, so that we get ground collisions properly
	body->SetCollisionEventMode(COLLISION_ALWAYS);

	// Set a capsule shape for collision
	CollisionShape* shape = objectNode->CreateComponent<CollisionShape>();
	shape->SetCapsule(0.9f, 1.5f, Vector3(0.0f, 0.9f, 0.0f));

	// Create the character logic component, which takes care of steering the rigidbody
	// Remember it so that we can set the controls. Use a WeakPtr because the scene hierarchy already owns it
	// and keeps it alive as long as it's not removed from the hierarchy
	character_ = objectNode->CreateComponent<Character>();
	// Set the head of this character body.
	characterHead_ = modelNode->GetChild(headName, true);

	// Add smoothed transform component.
	objectNode->CreateComponent<SmoothedTransform>();

	// Update first path.
	UpdatePath();
}

void AutoRunner::CreateCamera()
{
	cameraNode_ = new Node(context_);
	cameraNode_->SetPosition(Vector3(0.0f, 3.0f, -5.0f));
	Camera* camera = cameraNode_->CreateComponent<Camera>();
	camera->SetFarClip(300.0f);

	GetSubsystem<Renderer>()->SetViewport(0, new Viewport(context_, scene_, camera));
}

void AutoRunner::CreateOverlays()
{
	ResourceCache* cache = GetSubsystem<ResourceCache>();
	UI* ui = GetSubsystem<UI>();

	// Construct new Text object, set string to display and font to use
	scoreText_ = ui->GetRoot()->CreateChild<Text>();
	scoreText_->SetText("Score 0");
	scoreText_->SetFont(cache->GetResource<Font>("Fonts/BlueHighway.ttf"), 17);
	scoreText_->SetPosition(5, 5);
	scoreText_->SetAlignment(HA_LEFT, VA_TOP);
	scoreText_->SetColor(C_BOTTOMLEFT, Color(1, 1, 0.25));
	scoreText_->SetColor(C_BOTTOMRIGHT, Color(1, 1, 0.25));
}

void AutoRunner::SubscribeToEvents()
{
	// Subscribe to FixedUpdate event for updating the character path
	SubscribeToEvent(E_PHYSICSPRESTEP, HANDLER(AutoRunner, HandleFixedUpdate));

	// Subscribe HandleUpdate() function for processing update events
	SubscribeToEvent(E_UPDATE, HANDLER(AutoRunner, HandleUpdate));

	// Subscribe to PostUpdate event for updating the camera position after physics simulation
	SubscribeToEvent(E_POSTUPDATE, HANDLER(AutoRunner, HandlePostUpdate));

	// Subscribe HandlePostRenderUpdate() function for processing the post-render update event, during which we request debug geometry
	SubscribeToEvent(E_POSTRENDERUPDATE, HANDLER(AutoRunner, HandlePostRenderUpdate));
}

void AutoRunner::HandleFixedUpdate(StringHash eventType, VariantMap& eventData)
{
	using namespace PhysicsPreStep;

	if (character_) {
		Vector3 currentPoint;
		if (character_->GetCurrentPoint(currentPoint)) {
			Vector3 worldPos = character_->GetNode()->GetWorldPosition();
			currentPoint.y_ = worldPos.y_;
			float length = (worldPos - currentPoint).Length();
			if (length <= 1.0f)
				character_->RemoveFirstPoint();
		}
	}
}

void AutoRunner::HandleUpdate(StringHash eventType, VariantMap& eventData)
{
	using namespace Update;

	float timeStep = eventData[P_TIMESTEP].GetFloat();
	Input* input = GetSubsystem<Input>();
	ResourceCache* cache = GetSubsystem<ResourceCache>();

	if (character_)
	{
		// Update path.
		Vector3 point;
		if (character_->GetNumPoints() <= 1 && character_->HasTurnRequest()) {
			auto it = blocks_.Front();
			int outs = it->GetVar("Out").GetInt();
			if (outs == 0)
				blocks_.PopFront();

			UpdatePath(false);
		}

		character_->FollowPath(timeStep);

		// Create Incremental Blocks.
		/*Vector3 worldPos = character_->GetNode()->GetWorldPosition();
		Node* lastRoadBlock = blocks_.Back();
		Vector3 tailPos = lastRoadBlock->GetWorldPosition();
		Vector3 target = blockPos - worldPos;
		if (target.Length() < 3) {
			int randNumber = Rand();
			String fileName = String::EMPTY;

			if (1/ *randNumber % 2 == 0* /) { // straight road block
				fileName = "Objects/Block1.xml";
				blockPos += Vector3(0, 0, 20);
			} else if (randNumber % 3 == 0) {// cornered road block
				fileName = "Objects/RoadBlock1.xml";
				blockPos += Vector3(0, 0, 90);
			} else { // sloped road block
				fileName = "Objects/RoadBlock3.xml";
				blockPos += Vector3(0, 0, 89);
			}

			SharedPtr<File> fBlock1 = cache->GetFile(fileName);
			Node* blockNode = scene_->InstantiateXML(*fBlock1, blockPos, Quaternion::IDENTITY);
			if (blockNode)
				blocks_.Push(blockNode);

			// Create coins
			PODVector<Node*> allBlocks;
			blockNode->GetChildren(allBlocks, true);
			for (auto it = allBlocks.Begin(); it != allBlocks.End(); ++it) {
				Node* floorNode = *it;
				if (floorNode->GetName() == "Floor") {
					Vector3 pos = floorNode->GetWorldPosition();
					XMLFile* cbXML = cache->GetResource<XMLFile>("Objects/CoinBlue.xml");
					XMLFile* crXML = cache->GetResource<XMLFile>("Objects/CoinRed.xml");
					XMLFile* cgXML = cache->GetResource<XMLFile>("Objects/CoinGold.xml");
					if (cbXML) {
						scene_->InstantiateXML(cbXML->GetRoot(), pos + Vector3(-1, 1, -1), Quaternion::IDENTITY);
						scene_->InstantiateXML(cbXML->GetRoot(), pos + Vector3(-1, 1, -3), Quaternion::IDENTITY);
						scene_->InstantiateXML(cbXML->GetRoot(), pos + Vector3(-1, 1, -5), Quaternion::IDENTITY);
					}

					if (crXML) {
						scene_->InstantiateXML(crXML->GetRoot(), pos + Vector3(1, 1, 7), Quaternion::IDENTITY);
						scene_->InstantiateXML(cgXML->GetRoot(), pos + Vector3(1, 1, 10), Quaternion::IDENTITY);
						scene_->InstantiateXML(crXML->GetRoot(), pos + Vector3(1, 1, 13), Quaternion::IDENTITY);
					}
				}
			}

		}

		// Remove unused blocks from block list.
		if (blocks_.Size() > 2) {
			Node* block = blocks_.Front();
			block->Remove();
			blocks_.PopFront();
		}*/

		// Clear previous controls
		character_->controls_.Set(CTRL_FORWARD | CTRL_LEFT | CTRL_RIGHT /*| CTRL_BACK */| CTRL_JUMP, false);

		if (touch_->touchEnabled_)
		{
			// Update controls using touch (mobile)
			touch_->UpdateTouches(character_->controls_);
		}
		else
		{
			// Update controls using keys (desktop)
			UI* ui = GetSubsystem<UI>();

			if (!ui->GetFocusElement())
			{
				static bool starting = false;
				if (input->GetKeyDown('W') || input->GetKeyDown(KEY_UP))
					starting = true;

				character_->controls_.Set(CTRL_FORWARD, starting);
				character_->controls_.Set(CTRL_LEFT, input->GetKeyDown('A'));
				character_->controls_.Set(CTRL_RIGHT, input->GetKeyDown('D'));
				//character_->controls_.Set(CTRL_BACK, input->GetKeyDown('S'));
				character_->controls_.Set(CTRL_JUMP, input->GetKeyDown(KEY_SPACE));

				// Add character yaw & pitch from the mouse motion
				yaw_ += (float)input->GetMouseMoveX() * YAW_SENSITIVITY;
				pitch_ += (float)input->GetMouseMoveY() * YAW_SENSITIVITY;
				// Limit pitch
				pitch_ = Clamp(pitch_, -80.0f, 80.0f);

				// Switch between 1st and 3rd person
				if (input->GetKeyPress('F'))
					touch_->firstPerson_ = touch_->newFirstPerson_ = !touch_->firstPerson_;
			}
		}
	}

	// Toggle debug geometry with space
	if (input->GetKeyPress(KEY_F3))
		drawDebug_ = !drawDebug_;
}

void AutoRunner::HandlePostUpdate(StringHash eventType, VariantMap& eventData)
{
	if (!character_)
		return;

	// Update score
	if (scoreText_)
		scoreText_->SetText("Score " + (String)character_->GetScore());

	Node* characterNode = character_->GetNode();
	// Get camera lookat dir from character yaw + pitch
	Quaternion rot = Quaternion(yaw_, Vector3::UP);
	Quaternion dir = rot * Quaternion(pitch_, Vector3::RIGHT);

	// Turn head to camera pitch, but limit to avoid unnatural animation.
	/*Vector3 headWorldTarget; 
	if (character_->GetCurrentPoint(headWorldTarget)) {
		headWorldTarget.y_ = characterHead_->GetWorldPosition().y_;
		characterHead_->LookAt(headWorldTarget, Vector3(0.0f, 1.0f, 0.0f));
		// Correct head orientation because LookAt assumes Z = forward, but the bone has been authored differently (Y = forward)
		characterHead_->Rotate(Quaternion(0.0f, 90.0f, 90.0f));
	}*/

	if (touch_->firstPerson_)
	{
		cameraNode_->SetPosition(characterHead_->GetWorldPosition() + rot * Vector3(0.0f, 0.15f, 0.2f));
		cameraNode_->SetRotation(dir);
	}
	else
	{
		// Third person camera: position behind the character
		Vector3 aimPoint = characterNode->GetPosition() + rot * Vector3(0.0f, 1.7f, 0.0f);

		// Collide camera ray with static physics objects (layer bitmask 2) to ensure we see the character properly
		Vector3 rayDir = dir * Vector3::BACK;
		float rayDistance = touch_->cameraDistance_;
		PhysicsRaycastResult result;
		scene_->GetComponent<PhysicsWorld>()->RaycastSingle(result, Ray(aimPoint, rayDir), rayDistance, 2);
		if (result.body_)
			rayDistance = Min(rayDistance, result.distance_);
		rayDistance = Clamp(rayDistance, CAMERA_MIN_DIST, CAMERA_MAX_DIST);

		cameraNode_->SetPosition(aimPoint + rayDir * rayDistance);
		cameraNode_->SetRotation(dir);
	}
}

void AutoRunner::HandlePostRenderUpdate(StringHash eventType, VariantMap& eventData)
{
	// If draw debug mode is enabled, draw viewport debug geometry. Disable depth test so that we can see the effect of occlusion
	if (drawDebug_)
	{
		GetSubsystem<Renderer>()->DrawDebugGeometry(false);
		scene_->GetComponent<PhysicsWorld>()->DrawDebugGeometry(true);
	}
}

void AutoRunner::CreateInitialLevel()
{
	int cnt = 15;
	int maxBlockNumber = 4;
	ResourceCache* cache = GetSubsystem<ResourceCache>();
	SetRandomSeed(443);

	while (cnt > 0) {
		bool attached = false;
		Vector3 blockPos = Vector3::ZERO;
		Quaternion blockRot = Quaternion(90, Vector3(0, 1, 0));

		if (blocks_.Size() > 0) {
			// Get last block from array.
			Node* lastBlock = blocks_.Back();
			// Get Out variables.
			int outs = lastBlock->GetVar("Out").GetInt();
			// Get Out's node.
			Node* outNode = lastBlock->GetChild("Out");
			blockPos = outNode->GetWorldPosition();
			blockRot = outNode->GetWorldRotation();
			attached = true;
		}

		// Initial transform has been given from out node.
		int rnd = (Rand() % maxBlockNumber) + 1;
		String prefabName = "Objects/Block" + (String)rnd + ".xml";
		SharedPtr<File> fBlock1 = cache->GetFile(prefabName);
		Node* blockNode = scene_->InstantiateXML(*fBlock1, blockPos, blockRot);
		if (blockNode)
			blocks_.Push(blockNode);

		// And, then set actual transform of this block to get offset In node.
		if (attached) {
			Node* inNode = blockNode->GetChild("In");
			Vector3 offset = inNode->GetVar("Offset").GetVector3();
			Vector3 trans = inNode->GetWorldRotation() * offset;
			blockNode->Translate(trans);
		}

		// Create coins.
		Node* floorNode = blockNode->GetChild("Floor");
		Vector3 floorDirection = floorNode->GetWorldDirection();
		Vector3 floorRight = floorNode->GetWorldRight();
		Vector3 pos = floorNode->GetWorldPosition();
		pos.y_ += 1.0f;

		// Check obstacles before creating coins to prevent cycling path.
		/*Node* outNode = blockNode->GetChild("Out");
		Vector3 outDir = outNode->GetWorldRotation() * Vector3::LEFT;
		Vector3 origin = outNode->GetWorldPosition();
		Ray ray = Ray(origin, outDir);
		PhysicsWorld* world = scene_->GetComponent<PhysicsWorld>();
		PhysicsRaycastResult result;
		world->SphereCast(result, ray, 10, 20);

		if (result.body_) {
			PODVector<Node*> floors;
			blockNode->GetChildrenWithComponent<StaticModel>(floors, true);
			for (auto it = floors.Begin(); it != floors.End(); ++it) {
				Node* node = *it;
				String name = node->GetName();
			}
		}

		Node* sphere = scene_->CreateChild("SphereCast");
		StaticModel* mdl = sphere->CreateComponent<StaticModel>();
		mdl->SetModel(cache->GetResource<Model>("Models/Sphere.mdl"));
		mdl->SetMaterial(cache->GetResource<Material>("Materials/StoneTiled.xml"));
		sphere->SetScale(Vector3(10, 10, 10));
		sphere->SetPosition(origin);*/

		Node* coin = 0;
		XMLFile* cbXML = cache->GetResource<XMLFile>("Objects/CoinBlue.xml");
		XMLFile* crXML = cache->GetResource<XMLFile>("Objects/CoinRed.xml");
		XMLFile* cgXML = cache->GetResource<XMLFile>("Objects/CoinGold.xml");

		if (cbXML) {
			coin = scene_->InstantiateXML(cbXML->GetRoot(), pos, Quaternion::IDENTITY);
			coin->Translate(floorRight * 3.0f + floorDirection);
			coin = scene_->InstantiateXML(cbXML->GetRoot(), pos, Quaternion::IDENTITY);
			coin->Translate(floorRight * 5.0f + floorDirection);
			coin = scene_->InstantiateXML(cbXML->GetRoot(), pos, Quaternion::IDENTITY);
			coin->Translate(floorRight * 7.0f + floorDirection);
		}

		if (crXML && cgXML) {
			coin = scene_->InstantiateXML(crXML->GetRoot(), pos, Quaternion::IDENTITY);
			coin->Translate(floorRight * -3.0f + -floorDirection);
			coin = scene_->InstantiateXML(cgXML->GetRoot(), pos, Quaternion::IDENTITY);
			coin->Translate(floorRight * -5.0f + -floorDirection);
			coin = scene_->InstantiateXML(crXML->GetRoot(), pos, Quaternion::IDENTITY);
			coin->Translate(floorRight * -7.0f + -floorDirection);
		}

		cnt--;
	}
}

void AutoRunner::UpdatePath(bool startIn)
{
	List<Vector3> leftPoints;
	List<Vector3> rightPoints;
	List<Vector3> centerPoints;
	bool goon = true;

	while (goon)
	{
		Node* block = blocks_.Front();
		int outs = block->GetVar("Out").GetInt();
		Node* paths = block->GetChild("Paths");
		String posix = (startIn || outs == 0) ? "In" : "Out";

		Node* inNode = paths->GetChild("Center" + posix);
		unsigned int numChildren = inNode->GetNumChildren();
		for (unsigned int i = 0; i < numChildren; i++) {
			Node* pointNode = inNode->GetChild(i);
			centerPoints.Push(pointNode->GetWorldPosition());
		}

		inNode = paths->GetChild("Left" + posix);
		numChildren = inNode->GetNumChildren();
		for (unsigned int i = 0; i < numChildren; i++) {
			Node* pointNode = inNode->GetChild(i);
			leftPoints.Push(pointNode->GetWorldPosition());
		}

		inNode = paths->GetChild("Right" + posix);
		numChildren = inNode->GetNumChildren();
		for (unsigned int i = 0; i < numChildren; i++) {
			Node* pointNode = inNode->GetChild(i);
			rightPoints.Push(pointNode->GetWorldPosition());
		}

		switch (outs)
		{
		case 0:
			{
				blocks_.PopFront();
				goon = blocks_.Size() > 0;
			}
			break;
		case 1:
		case 2:
		default:
			{
				if (!startIn)
				{
					blocks_.PopFront();
					startIn = true;
					goon = blocks_.Size() > 0;
				}
				else
				{
					goon = false;
				}
			}
			break;
		}
	}

	character_->AddToPath(CharacterSide::LEFT_SIDE, leftPoints);
	character_->AddToPath(CharacterSide::RIGHT_SIDE, rightPoints);
	character_->AddToPath(CharacterSide::CENTER_SIDE, centerPoints);
}
