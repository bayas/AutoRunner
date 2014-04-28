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

#include "CoreEvents.h"
#include "Engine.h"
#include "Font.h"
#include "Input.h"
#include "ProcessUtils.h"
#include "Text.h"
#include "UI.h"
#include "ResourceCache.h"
#include "Scene.h"
#include "StaticModel.h"

#include "AutoRunner.h"

#include "DebugNew.h"
#include "Touch.h"
#include "Node.h"
#include "Camera.h"
#include "XMLFile.h"

// Expands to this example's entry-point
DEFINE_APPLICATION_MAIN(AutoRunner)

AutoRunner::AutoRunner(Context* context) :
    Sample(context),
    yaw_(0.0f),
    pitch_(0.0f)
{
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

	XMLFile* xml = cache->GetResource<XMLFile>("Objects/RoadBlock.xml");
	Node* blockNode = scene_->InstantiateXML(xml->GetRoot(), Vector3(-1, 2, 0), Quaternion::IDENTITY);

	if (GetPlatform() == "Android" || GetPlatform() == "iOS")
	{
		// Pass knowledge of the scene & camera node to the Touch helper object.
		touch_->scene_ = scene_;
		touch_->cameraNode_ = cameraNode_;
	}
}

void AutoRunner::CreateCharacter()
{
	// TODO: implement character.
}

void AutoRunner::CreateCamera()
{
	cameraNode_ = new Node(context_);
	cameraNode_->SetPosition(Vector3(0.0f, 5.0f, 0.0f));
	Camera* camera = cameraNode_->CreateComponent<Camera>();
	camera->SetFarClip(300.0f);

	GetSubsystem<Renderer>()->SetViewport(0, new Viewport(context_, scene_, camera));
}

void AutoRunner::SubscribeToEvents()
{
    // Subscribe HandleUpdate() function for processing update events
    SubscribeToEvent(E_UPDATE, HANDLER(AutoRunner, HandleUpdate));
}

void AutoRunner::MoveCamera(float timeStep)
{
	// Do not move if the UI has a focused element (the console)
	if (GetSubsystem<UI>()->GetFocusElement())
		return;

	Input* input = GetSubsystem<Input>();

	// Movement speed as world units per second
	const float MOVE_SPEED = 20.0f;
	// Mouse sensitivity as degrees per pixel
	const float MOUSE_SENSITIVITY = 0.1f;

	// Use this frame's mouse motion to adjust camera node yaw and pitch. Clamp the pitch between -90 and 90 degrees
	IntVector2 mouseMove = input->GetMouseMove();
	yaw_ += MOUSE_SENSITIVITY * mouseMove.x_;
	pitch_ += MOUSE_SENSITIVITY * mouseMove.y_;
	pitch_ = Clamp(pitch_, -90.0f, 90.0f);

	// Construct new orientation for the camera scene node from yaw and pitch. Roll is fixed to zero
	cameraNode_->SetRotation(Quaternion(pitch_, yaw_, 0.0f));

	// Read WASD keys and move the camera scene node to the corresponding direction if they are pressed
	// Use the TranslateRelative() function to move relative to the node's orientation. Alternatively we could
	// multiply the desired direction with the node's orientation quaternion, and use just Translate()
	if (input->GetKeyDown('W'))
		cameraNode_->TranslateRelative(Vector3::FORWARD * MOVE_SPEED * timeStep);
	if (input->GetKeyDown('S'))
		cameraNode_->TranslateRelative(Vector3::BACK * MOVE_SPEED * timeStep);
	if (input->GetKeyDown('A'))
		cameraNode_->TranslateRelative(Vector3::LEFT * MOVE_SPEED * timeStep);
	if (input->GetKeyDown('D'))
		cameraNode_->TranslateRelative(Vector3::RIGHT * MOVE_SPEED * timeStep);
}

void AutoRunner::HandleUpdate(StringHash eventType, VariantMap& eventData)
{
	using namespace Update;

	// Take the frame time step, which is stored as a float
	float timeStep = eventData[P_TIMESTEP].GetFloat();

	// Move the camera, scale movement with time step
	MoveCamera(timeStep);
}


void AutoRunner::HandlePostUpdate(StringHash eventType, VariantMap& eventData)
{

}
