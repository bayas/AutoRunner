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
	blockPos_(Vector3::ZERO)
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
	XMLFile* xml = cache->GetResource<XMLFile>("Objects/RoadBlock.xml");
	Node* blockNode = scene_->InstantiateXML(xml->GetRoot(), Vector3::ZERO, Quaternion::IDENTITY);
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

	if (character_)
	{
		// Create Incremental Blocks.
		Vector3 worldPos = character_->GetNode()->GetWorldPosition();
		if (blockPos_.z_ < worldPos.z_) {
			blockPos_ += Vector3(0, 0, 2);

			XMLFile* xml = GetSubsystem<ResourceCache>()->GetResource<XMLFile>("Objects/RoadBlock.xml");
			Node* blockNode = scene_->InstantiateXML(xml->GetRoot(), blockPos_, Quaternion::IDENTITY);
			blocks_.Push(blockNode);
		}

		// Remove unused blocks from block list.
		if (blocks_.Size() > 3) {
			Node* block = blocks_.Front();
			block->Remove();
			blocks_.PopFront();
		}

		// Clear previous controls
		character_->controls_.Set(CTRL_FORWARD /*| CTRL_BACK | CTRL_LEFT | CTRL_RIGHT */| CTRL_JUMP, false);

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
				character_->controls_.Set(CTRL_FORWARD, input->GetKeyDown('W') || input->GetKeyDown(KEY_UP));
				//character_->controls_.Set(CTRL_BACK, input->GetKeyDown('S'));
				//character_->controls_.Set(CTRL_LEFT, input->GetKeyDown('A'));
				//character_->controls_.Set(CTRL_RIGHT, input->GetKeyDown('D'));
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
