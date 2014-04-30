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
#include "DebugNew.h"

// Expands to this example's entry-point
DEFINE_APPLICATION_MAIN(AutoRunner)

AutoRunner::AutoRunner(Context* context) :
	Sample(context),
	touch_(new Touch(context)),
	yaw_(0.0f),
	pitch_(0.0f),
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

	// Pass knowledge of the scene & camera node to the Touch helper object.
	touch_->scene_ = scene_;
	touch_->cameraNode_ = cameraNode_;
}

void AutoRunner::CreateCharacter()
{
	ResourceCache* cache = GetSubsystem<ResourceCache>();

	Node* objectNode = scene_->CreateChild("Jack");
	objectNode->SetPosition(Vector3(0.0f, 3.0f, 0.0f));

	// Create the rendering component + animation controller
	AnimatedModel* object = objectNode->CreateComponent<AnimatedModel>();
	object->SetModel(cache->GetResource<Model>("Models/Jack.mdl"));
	object->SetMaterial(cache->GetResource<Material>("Materials/Jack.xml"));
	object->SetCastShadows(true);
	objectNode->CreateComponent<AnimationController>();

	// Set the head bone for manual control
	object->GetSkeleton().GetBone("Bip01_Head")->animated_ = false;

	// Create rigidbody, and set non-zero mass so that the body becomes dynamic
	RigidBody* body = objectNode->CreateComponent<RigidBody>();
	body->SetCollisionLayer(1);
	body->SetMass(1.0f);

	// Set zero angular factor so that physics doesn't turn the character on its own.
	// Instead we will control the character yaw manually
	body->SetAngularFactor(Vector3::ZERO);

	// Set the rigidbody to signal collision also when in rest, so that we get ground collisions properly
	body->SetCollisionEventMode(COLLISION_ALWAYS);

	// Set a capsule shape for collision
	CollisionShape* shape = objectNode->CreateComponent<CollisionShape>();
	shape->SetCapsule(0.7f, 1.8f, Vector3(0.0f, 0.9f, 0.0f));

	// Create the character logic component, which takes care of steering the rigidbody
	// Remember it so that we can set the controls. Use a WeakPtr because the scene hierarchy already owns it
	// and keeps it alive as long as it's not removed from the hierarchy
	character_ = objectNode->CreateComponent<Character>();

	// Create first block.
	Node* blockNode = scene_->CreateChild("RoadBlock");
	blockNode->SetScale(Vector3(6.0f, 1.0f, 2.0f));
	StaticModel* planeObject = blockNode->CreateComponent<StaticModel>();
	planeObject->SetModel(cache->GetResource<Model>("Models/Box.mdl"));
	planeObject->SetMaterial(cache->GetResource<Material>("Materials/StoneTiled.xml"));
	blockNode->CreateComponent<RigidBody>();
	CollisionShape* blockShape = blockNode->CreateComponent<CollisionShape>();
	// Set a box shape of size 1 x 1 x 1 for collision. The shape will be scaled with the scene node scale, so the
	// rendering and physics representation sizes should match (the box model is also 1 x 1 x 1.)
	blockShape->SetBox(Vector3::ONE);
	blocks_.Push(blockNode);
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
    // Subscribe HandleUpdate() function for processing update events
	SubscribeToEvent(E_UPDATE, HANDLER(AutoRunner, HandleUpdate));

	// Subscribe to PostUpdate event for updating the camera position after physics simulation
	SubscribeToEvent(E_POSTUPDATE, HANDLER(AutoRunner, HandlePostUpdate));
}

void AutoRunner::HandleUpdate(StringHash eventType, VariantMap& eventData)
{
	using namespace Update;

	float timeStep = eventData[P_TIMESTEP].GetFloat();
	Input* input = GetSubsystem<Input>();
	ResourceCache* cache = GetSubsystem<ResourceCache>();

	if (character_)
	{
		// Create Incremental Blocks.
		Vector3 worldPos = character_->GetNode()->GetWorldPosition();
		Node* lastRoadBlock = blocks_.Back();
		Vector3 blockPos = lastRoadBlock->GetWorldPosition();
		Vector3 target = blockPos - worldPos;
		if (target.Length() < 2) {
			int randNumber = Rand();
			String fileName = String::EMPTY;

			if (1/*randNumber % 2 == 0*/) { // straight road block
				fileName = "Objects/RoadBlock1.xml";
				blockPos += Vector3(0, 0, 100);
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
		}

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

	if (touch_->firstPerson_)
	{
		Node* headNode = characterNode->GetChild("Bip01_Head", true);
		cameraNode_->SetPosition(headNode->GetWorldPosition() + rot * Vector3(0.0f, 0.15f, 0.2f));
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
