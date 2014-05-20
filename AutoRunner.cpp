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
#include "Log.h"
#include "DebugNew.h"

// Expands to this example's entry-point
DEFINE_APPLICATION_MAIN(AutoRunner)

AutoRunner::AutoRunner(Context* context) :
	Sample(context),
	touch_(new Touch(context)),
	yaw_(0.0f),
	pitch_(0.0f),
	drawDebug_(false),
	scoreText_(0),
	seed_(483)
{
	Character::RegisterObject(context);
	lastOutWorldTransform_ = Matrix3x4(Vector3::ZERO, Quaternion(90, Vector3(0, 1, 0)), 1);
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

	// Pass knowledge of the scene & camera node to the Touch helper object.
	touch_->scene_ = scene_;
	touch_->cameraNode_ = cameraNode_;
}

void AutoRunner::CreateCharacter()
{
	ResourceCache* cache = GetSubsystem<ResourceCache>();

	Node* objectNode = scene_->CreateChild("Player");
	objectNode->SetPosition(Vector3(0.0f, 3.0f, 0.0f));
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
	shape->SetCapsule(0.7f, 1.5f, Vector3(0.0f, 0.8f, 0.0f));

	// Create the character logic component, which takes care of steering the rigidbody
	// Remember it so that we can set the controls. Use a WeakPtr because the scene hierarchy already owns it
	// and keeps it alive as long as it's not removed from the hierarchy
	character_ = objectNode->CreateComponent<Character>();
	// Set the head of this character body.
	characterHead_ = modelNode->GetChild(headName, true);

	// Add smoothed transform component.
	objectNode->CreateComponent<SmoothedTransform>();

	// Create initial level and update the follow-path.
	CreateLevel();
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
			UpdatePath(false);
			character_->RemovePassedBlocks();

			if (blocks_.Size() <= 0)
				CreateLevel();
		}

		character_->FollowPath(timeStep);
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

	// Toggle fill mode on main camera
	if (input->GetKeyPress(KEY_F4))
	{
		Camera* cam = cameraNode_->GetComponent<Camera>();
		FillMode mode = cam->GetFillMode();
		if (mode == FILL_WIREFRAME)
			cam->SetFillMode(FILL_SOLID);
		else
			cam->SetFillMode(FILL_WIREFRAME);
	}
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

	DebugRenderer* debug = scene_->GetComponent<DebugRenderer>();
	for (unsigned int i = 0; i < lines_.Size(); i++)
	{
		DebugLine line = lines_[i];
		debug->AddLine(line.start_, line.end_, line.color_);
	}

	for (unsigned int i = 0; i < spheres_.Size(); i++)
	{
		debug->AddSphere(spheres_[i], Color::RED);
	}
}

void AutoRunner::CreateLevel()
{
	int cnt = 3;
	int maxRecursive = 30;
	int maxBlockNumber = 3;
	ResourceCache* cache = GetSubsystem<ResourceCache>();

	ChangeSeed();
	while (cnt > 0)
	{
		Vector3 blockPos = lastOutWorldTransform_.Translation();
		Quaternion blockRot = lastOutWorldTransform_.Rotation();

		// Initial transform has been given from out node.
		int rnd = Random(maxBlockNumber) + 1;
		String prefabName = "Objects/Block" + String(rnd) + ".xml";
		SharedPtr<File> fBlock1 = cache->GetFile(prefabName);
		Node* blockNode = scene_->InstantiateXML(*fBlock1, blockPos, blockRot);

		// And, then set actual transform of this block to get offset In node.
		Node* inNode = blockNode->GetChild("In");
		Vector3 offset = inNode->GetVar("Offset").GetVector3();
		Vector3 trans = inNode->GetWorldRotation() * offset;
		blockNode->Translate(trans);

		// Create coins.
		Node* floorNode = blockNode->GetChild("Floor");
		Vector3 floorDirection = floorNode->GetWorldDirection();
		Vector3 floorRight = floorNode->GetWorldRight();
		Vector3 pos = floorNode->GetWorldPosition();
		pos.y_ += 1.0f;

		// Check obstacles before creating coins to prevent cycling path.
		Node* outNode = blockNode->GetChild("Out");
		Vector3 outDir = outNode->GetWorldRotation() * Vector3::LEFT;
		Vector3 origin = outNode->GetWorldPosition();
		Ray ray(origin, outDir);
		PhysicsRaycastResult result;

		PhysicsWorld* world = scene_->GetComponent<PhysicsWorld>();
		world->RaycastSingle(result, ray, 20.0f, FLOOR_COLLISION_MASK);
		//lines_.Push(DebugLine(origin, origin + outDir * 20.0f, Color::RED.ToUInt()));

		if (result.body_)
		{
			if (maxRecursive == 0)
				assert(false);

			ChangeSeed();
			blockNode->Remove();
			maxRecursive--;
			continue;
		}
		else
		{
			maxRecursive = 30;
		}

		/*Node* coin = 0;
		XMLFile* cbXML = cache->GetResource<XMLFile>("Objects/CoinBlue.xml");
		XMLFile* crXML = cache->GetResource<XMLFile>("Objects/CoinRed.xml");
		XMLFile* cgXML = cache->GetResource<XMLFile>("Objects/CoinGold.xml");

		if (cbXML) {
			coin = scene_->InstantiateXML(cbXML->GetRoot(), pos, Quaternion::IDENTITY);
			coin->Translate(floorRight * 3.0f + floorDirection);
			coin->SetParent(blockNode);
			coin = scene_->InstantiateXML(cbXML->GetRoot(), pos, Quaternion::IDENTITY);
			coin->Translate(floorRight * 5.0f + floorDirection);
			coin->SetParent(blockNode);
			coin = scene_->InstantiateXML(cbXML->GetRoot(), pos, Quaternion::IDENTITY);
			coin->Translate(floorRight * 7.0f + floorDirection);
			coin->SetParent(blockNode);
		}

		if (crXML && cgXML) {
			coin = scene_->InstantiateXML(crXML->GetRoot(), pos, Quaternion::IDENTITY);
			coin->Translate(floorRight * -3.0f + -floorDirection);
			coin->SetParent(blockNode);
			coin = scene_->InstantiateXML(cgXML->GetRoot(), pos, Quaternion::IDENTITY);
			coin->Translate(floorRight * -5.0f + -floorDirection);
			coin->SetParent(blockNode);
			coin = scene_->InstantiateXML(crXML->GetRoot(), pos, Quaternion::IDENTITY);
			coin->Translate(floorRight * -7.0f + -floorDirection);
			coin->SetParent(blockNode);
		}*/

		cnt--;
		lastOutWorldTransform_ = Matrix3x4(outNode->GetWorldPosition(), outNode->GetWorldRotation(), 1);
		blocks_.Push(blockNode);

		// If the last block is the straight then,
		// Go ahead creating the block until the last block is turned one.
		if (cnt == 0 && rnd == 1)
		{
			ChangeSeed();
			cnt++;
		}
	}

	UpdatePath();
}

void AutoRunner::UpdatePath(bool startIn)
{
	List<Vector3> leftPoints;
	List<Vector3> rightPoints;
	List<Vector3> centerPoints;

	while (blocks_.Size() > 0)
	{
		Node* block = blocks_.Front();
		int outs = block->GetVar("Out").GetInt();

		Node* paths = block->GetChild("Paths");
		String posix = (startIn || outs == 0) ? "In" : "Out";
		Node* inNode = paths->GetChild("Center" + posix);
		unsigned int numChildren = inNode->GetNumChildren();

		for (unsigned int i = 0; i < numChildren; i++)
		{
			Node* pointNode = inNode->GetChild(i);
			centerPoints.Push(pointNode->GetWorldPosition());
			// Create a box-model component to see each path point.
			/*StaticModel* boxObject = pointNode->CreateComponent<StaticModel>();
			boxObject->SetModel(GetSubsystem<ResourceCache>()->GetResource<Model>("Models/Box.mdl"));
			boxObject->SetMaterial(GetSubsystem<ResourceCache>()->GetResource<Material>("Materials/Stone.xml"));*/
		}

		inNode = paths->GetChild("Left" + posix);
		numChildren = inNode->GetNumChildren();

		for (unsigned int i = 0; i < numChildren; i++)
		{
			Node* pointNode = inNode->GetChild(i);
			leftPoints.Push(pointNode->GetWorldPosition());
			// Create a box-model component to see each path point.
			/*StaticModel* boxObject = pointNode->CreateComponent<StaticModel>();
			boxObject->SetModel(GetSubsystem<ResourceCache>()->GetResource<Model>("Models/Box.mdl"));
			boxObject->SetMaterial(GetSubsystem<ResourceCache>()->GetResource<Material>("Materials/Stone.xml"));*/
		}

		inNode = paths->GetChild("Right" + posix);
		numChildren = inNode->GetNumChildren();

		for (unsigned int i = 0; i < numChildren; i++)
		{
			Node* pointNode = inNode->GetChild(i);
			rightPoints.Push(pointNode->GetWorldPosition());
			// Create a box-model component to see each path point.
			/*StaticModel* boxObject = pointNode->CreateComponent<StaticModel>();
			boxObject->SetModel(GetSubsystem<ResourceCache>()->GetResource<Model>("Models/Box.mdl"));
			boxObject->SetMaterial(GetSubsystem<ResourceCache>()->GetResource<Material>("Materials/Stone.xml"));*/
		}

		if (outs > 0)
		{
			if (!startIn)
				startIn = true;
			else
				break;
		}

		blocks_.PopFront();
	}

	character_->AddToPath(CharacterSide::LEFT_SIDE, leftPoints);
	character_->AddToPath(CharacterSide::RIGHT_SIDE, rightPoints);
	character_->AddToPath(CharacterSide::CENTER_SIDE, centerPoints);
}

void AutoRunner::ChangeSeed()
{
	int seedFactor = Random(1, 10);
	if (seedFactor % 2 == 0)
		seed_ ++;
	else
	{
		if (seed_ > 0)
			seed_--;
	}

	SetRandomSeed(seed_);
}