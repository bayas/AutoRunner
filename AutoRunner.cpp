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

#ifdef ENABLE_ANGELSCRIPT
#include "ScriptFile.h"
#include "Script.h"
#include "ScriptInstance.h"
#endif

#ifdef ENABLE_LUA
#include "LuaScript.h"
#include "LuaScriptInstance.h"
#endif

#include "XMLFile.h"
#include "PhysicsEvents.h"
#include "SmoothedTransform.h"
#include "Log.h"
#include "Param.h"
#include "Menu.h"
#include "UIEvents.h"
#include "Console.h"
#include "Profiler.h"
#include "DebugNew.h"

// Expands to this example's entry-point
DEFINE_APPLICATION_MAIN(AutoRunner)

AutoRunner::AutoRunner(Context* context) :
	Sample(context),
	touch_(new Touch(context)),
	drawDebug_(false),
	isPlaying_(false),
	useMouseMove_(false),
	numBlocks_(0)
{
	Character::RegisterObject(context);
}

void AutoRunner::Setup()
{
	Sample::Setup();
	/*// On Android and iOS, read command line from a file as parameters can not otherwise be easily given
#if defined(ANDROID) || defined(IOS)
	engineParameters_["FullScreen"]  = true;
	engineParameters_["Headless"]    = false;
	// Reparse engine startup parameters now
	engineParameters_ = Engine::ParseParameters(GetArguments());
#else
	Sample::Setup();
#endif*/
}

void AutoRunner::Start()
{
#ifdef ENABLE_ANGELSCRIPT
	// Instantiate and register the AngelScript subsystem
	context_->RegisterSubsystem(new Script(context_));
#endif

#ifdef ENABLE_LUA
	// Instantiate and register the Lua script subsystem
	LuaScript* luaScript = new LuaScript(context_);
	context_->RegisterSubsystem(luaScript);
#endif

	// Execute base class startup
	Sample::Start();

	// Init scene content
	InitScene();

	// Create Camera
	CreateCamera();

	// Create overlays
	CreateOverlays();

	// Initialize touch input on Android & iOS
	if (GetPlatform() == "Android" || GetPlatform() == "iOS")
	{
		touch_->InitTouchInput();
		// Pass knowledge of the scene & camera node to the Touch helper object.
		touch_->scene_ = scene_;
		touch_->cameraNode_ = cameraNode_;
		useMouseMove_ = false;
	}

	// Subscribe to necessary events
	SubscribeToEvents();

	GetSubsystem<Console>()->Toggle();
	GetSubsystem<Console>()->SetFocusOnShow(false);
	GetSubsystem<Graphics>()->SetWindowTitle("AutoRunner Kit Game");
	SetLogoVisible(false);
	CreateUI();
}

void AutoRunner::Stop()
{
	ResetGame();
}

void AutoRunner::InitScene()
{
	ResourceCache* cache = GetSubsystem<ResourceCache>();
	FileSystem* fs = GetSubsystem<FileSystem>();

	scene_ = new Scene(context_);
	File loadFile(context_, fs->GetProgramDir() + "Data/Scenes/AutoRunner.xml", FILE_READ);
	scene_->LoadXML(loadFile);

	String platform = GetPlatform();
	if (platform == "Android" || platform == "iOS" || platform == "Raspberry Pi")
	{
		Renderer* renderer = GetSubsystem<Renderer>(); 
		renderer->SetReuseShadowMaps(false);
		renderer->SetShadowQuality(SHADOWQUALITY_LOW_16BIT);
		// Adjust the directional light shadow range slightly further, as only the first
		// cascade is supported
		Node* sunNode = scene_->GetChild("Sun1");
		Light* sun1 = sunNode->GetComponent<Light>();
		sun1->SetShadowCascade(CascadeParameters(15.0f, 0.0f, 0.0f, 0.0f, 0.9f));
		sun1->SetShadowIntensity(0.333f);
		sunNode = scene_->GetChild("Sun2");
		Light* sun2 = sunNode->GetComponent<Light>();
		sun2->SetShadowCascade(CascadeParameters(15.0f, 0.0f, 0.0f, 0.0f, 0.9f));
		sun2->SetShadowIntensity(0.333f);
	}
}

void AutoRunner::CreateCharacter()
{
	ResourceCache* cache = GetSubsystem<ResourceCache>();
	// Create root node of player
	Node* objectNode = scene_->CreateChild("Player");
	objectNode->SetPosition(Vector3(0.0f, 3.0f, 0.0f));
	// Create model node
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
	body->SetCollisionLayer(FLOOR_COLLISION_MASK|COIN_COLLISION_MASK|OBSTACLE_COLLISION_MASK);
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
}

void AutoRunner::CreateCamera()
{
	cameraNode_ = new Node(context_);
	cameraNode_->SetPosition(Vector3(0.0f, 3.0f, -5.0f));
	Camera* camera = cameraNode_->CreateComponent<Camera>();
	camera->SetFarClip(300.0f);
	// Create zone in the camera node
	Node* zoneNode = scene_->CreateChild("Zone");
	zoneNode->SetParent(cameraNode_);
	Zone* zone = zoneNode->CreateComponent<Zone>();
	zone->SetBoundingBox(BoundingBox(-10, 10));
	zone->SetFogEnd(30);
	zone->SetFogStart(10);
	zone->SetFogColor(Color(0.1f, 0.2f, 0.3f));
	zone->SetAmbientColor(Color(0.05f, 0.1f, 0.15f));

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

	if (touch_->touchEnabled_)
		touch_->SubscribeToTouchEvents();
}

void AutoRunner::HandleFixedUpdate(StringHash eventType, VariantMap& eventData)
{
	using namespace PhysicsPreStep;

}

void AutoRunner::HandleUpdate(StringHash eventType, VariantMap& eventData)
{
	using namespace Update;

	UI* ui = GetSubsystem<UI>();
	Input* input = GetSubsystem<Input>();
	ResourceCache* cache = GetSubsystem<ResourceCache>();
	float timeStep = eventData[P_TIMESTEP].GetFloat();

	if (useMouseMove_)
		ui->GetCursor()->SetVisible(!input->GetMouseButtonDown(MOUSEB_RIGHT));

	if (character_ && !character_->IsDead())
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
		character_->controls_.Set(CTRL_FORWARD | CTRL_LEFT | CTRL_RIGHT | CTRL_BACK | CTRL_JUMP, false);

		if (!isPlaying_)
			isPlaying_ = character_->OnGround();

		if (touch_->touchEnabled_)
		{
			// Update controls using touch (mobile)
			touch_->UpdateTouches(character_->controls_);
		}
		else
		{
			// Update controls using keys (desktop)
			//if (!ui->GetFocusElement())
			{
				character_->controls_.Set(CTRL_FORWARD, isPlaying_);
				character_->controls_.Set(CTRL_LEFT, input->GetKeyDown('A'));
				character_->controls_.Set(CTRL_RIGHT, input->GetKeyDown('D'));
				character_->controls_.Set(CTRL_BACK, input->GetKeyDown('S'));
				character_->controls_.Set(CTRL_JUMP, input->GetKeyDown('W'));

				if (useMouseMove_)
				{
					// Add character yaw & pitch from the mouse motion
					yaw_ += (float)input->GetMouseMoveX() * YAW_SENSITIVITY;
					pitch_ += (float)input->GetMouseMoveY() * YAW_SENSITIVITY;
					// Limit pitch
					pitch_ = Clamp(pitch_, -80.0f, 80.0f);
				}

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

	// Toggle using mouse pitch_ and yaw_ for camera.
	if (input->GetKeyPress(KEY_C))
		useMouseMove_ = !useMouseMove_;

	// Toggle zone mode.
	if (input->GetKeyPress(KEY_Z))
	{
		Node* zoneNode = cameraNode_->GetChild("Zone");
		zoneNode->SetEnabled(!zoneNode->IsEnabled());
	}
}

void AutoRunner::HandlePostUpdate(StringHash eventType, VariantMap& eventData)
{
	if (!character_)
		return;

	if (character_->IsDead())
	{
		if (!isPlaying_)
			return;

		static int highScore = 0;

		if (!touch_->touchEnabled_)
			GetSubsystem<UI>()->GetCursor()->SetVisible(true);

		String elementName = "InfoText";
		Text* infoText = static_cast<Text*>(gameMenu_->GetChild(elementName));
		infoText->SetText("You Dead!, Restart or Exit..");
		infoText->SetPosition(60, infoText->GetPosition().y_);
		elementName = "LastScoreText";
		Text* lastScoreText = static_cast<Text*>(gameMenu_->GetChild(elementName));
		lastScoreText->SetVisible(true);
		lastScoreText->SetText("Score: " + String(character_->GetScore()));

		elementName = "HighScoreText";
		Text* highScoreText = static_cast<Text*>(gameMenu_->GetChild(elementName));
		if (highScore < character_->GetScore())
		{
			highScore = character_->GetScore();
			highScoreText->GetChild(0)->SetVisible(true);
		}
		else
		{
			highScoreText->GetChild(0)->SetVisible(false);
		}

		highScoreText->SetVisible(true);
		highScoreText->SetText("High Score: " + String(highScore));

		elementName = "PlayBtn";
		Button* playBtn = static_cast<Button*>(gameMenu_->GetChild(elementName));
		Text* playText = static_cast<Text*>(playBtn->GetChild(0));
		playText->SetText("RESTART!");

		gameMenu_->SetEnabled(true);
		gameMenu_->SetVisible(true);
		gameMenu_->SetFocus(true);
		isPlaying_ = false;
		numBlocks_ = 0;

		return;
	}

	// Update score
	scoreText_->SetText("Score " + String(character_->GetScore()));

	Node* characterNode = character_->GetNode();
	// Get camera lookat dir from character yaw + pitch
	float yawAngle = 0;
	float pitchAngle = 0;
	if (useMouseMove_)
	{
		yawAngle = yaw_;
		pitchAngle = pitch_;
	}
	else
	{
		yawAngle = character_->controls_.yaw_;
		pitchAngle = character_->controls_.pitch_;
	}

	Quaternion rot = Quaternion(yawAngle, Vector3::UP);
	Quaternion dir = rot * Quaternion(pitchAngle, Vector3::RIGHT);

	if (touch_->firstPerson_)
	{
		cameraNode_->SetPosition(characterHead_->GetWorldPosition() + rot * Vector3(0.0f, 0.15f, 0.2f));
		cameraNode_->SetRotation(dir);
	}
	else
	{
		// Third person camera: position behind the character
		Vector3 aimPoint = characterNode->GetPosition() + rot * Vector3(0.0f, 2.3f, -1.5f);

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

void AutoRunner::HandleControlClicked(StringHash eventType, VariantMap& eventData)
{
	// Get control that was clicked
	UIElement* clicked = static_cast<UIElement*>(eventData[UIMouseClick::P_ELEMENT].GetPtr());

	if (clicked)
	{
		String name = clicked->GetName();
		if (name == "PlayBtn")
		{
			if (character_ && character_->IsDead())
				ResetGame();

			InitGame();
			gameMenu_->SetVisible(false);
			gameMenu_->SetEnabled(false);
			gameMenu_->SetFocus(false);
			if (!touch_->touchEnabled_)
				GetSubsystem<UI>()->GetCursor()->SetVisible(false);
		}
		else if (name == "ExitBtn")
		{
			engine_->Exit();
		}
	}
}

void AutoRunner::CreateLevel()
{
	int cnt = 3;
	int maxRecursive = 30;
	int maxBlockNumber = 7;
	ResourceCache* cache = GetSubsystem<ResourceCache>();

	while (cnt > 0)
	{
		Vector3 blockPos = lastOutWorldTransform_.Translation();
		Quaternion blockRot = lastOutWorldTransform_.Rotation();

		// Initial transform has been given from out node.
		int rnd = Random(maxBlockNumber) + 1;
		// Set the starting platform.
		if (numBlocks_ == 0)
			rnd = 1;
		else if (rnd == 1 || rnd == 7 || rnd == 6)
			rnd = 5;

		String prefabName = "Objects/Block" + String(rnd) + ".xml";
		SharedPtr<File> fBlock1 = cache->GetFile(prefabName);
		Node* blockNode = scene_->InstantiateXML(*fBlock1, blockPos, blockRot);
		int outs = blockNode->GetVar(GameVarirants::P_OUT).GetInt();

		// And, then set actual transform of this block to get offset In node.
		Node* inNode = blockNode->GetChild("In");
		Vector3 offset = inNode->GetVar(GameVarirants::P_OFFSET).GetVector3();
		Vector3 trans = inNode->GetWorldRotation() * offset;
		blockNode->Translate(trans);

		// Check obstacles before creating coins to prevent cycling path.
		String posix = String::EMPTY;
		int twoWay = 1;
		// If the path is two way turned.
		if (outs >= 2)
		{
			posix = "R";
			twoWay++;
		}

		bool accepted = true;
		Node* outNode = blockNode->GetChild("Out" + posix);

		while (twoWay > 0)
		{
			Vector3 outDir = outNode->GetWorldRotation() * Vector3::LEFT;
			Vector3 origin = outNode->GetWorldPosition();
			Ray ray(origin, outDir);
			PhysicsRaycastResult result;

			PhysicsWorld* world = scene_->GetComponent<PhysicsWorld>();
			world->RaycastSingle(result, ray, 20.0f, FLOOR_COLLISION_MASK);

			if (result.body_)
			{
				if (maxRecursive == 0)
					assert(false);

				blockNode->Remove();
				maxRecursive--;
				accepted = false;
				break;
			}
			else
			{
				maxRecursive = 30;
			}

			// We tried first way as an Right posix, then will be trying other way as a Left posix.
			if (outs >= 2)
				posix = "L";

			twoWay--;
		}

		// If this created is not accepted then continue.
		if (!accepted)
			continue;

		// Create coins in appropriate slots.
		Node* slots = blockNode->GetChild("Slots");
		PODVector<Node*> coinSlots;
		PODVector<Node*> obstacleSlots;
		for (unsigned int slotIndex = 0; slotIndex < slots->GetNumChildren(); slotIndex++)
		{
			Node* slot = slots->GetChild(slotIndex);
			if (!slot->GetVar(GameVarirants::P_FITTOCOIN).IsEmpty())
				coinSlots.Push(slot);
			if (!slot->GetVar(GameVarirants::P_FITTOOBSTACLE).IsEmpty())
				obstacleSlots.Push(slot);
		}

		if (numBlocks_ > 0 && !coinSlots.Empty())
		{
			int slotSize = coinSlots.Size() - 1;
			int slotIndex = Random(slotSize);
			Node* firstFit = coinSlots[slotIndex];
			XMLFile* coinObj = cache->GetResource<XMLFile>(GetRandomCoinObjectName());

			if (coinObj)
			{
				Node* coinNode = scene_->InstantiateXML(coinObj->GetRoot(), firstFit->GetWorldPosition(), firstFit->GetWorldRotation());
				coinNode->SetParent(blockNode);
			}
		}

		// Create obstacles in appropriate slots.
		if (numBlocks_ > 0 && !obstacleSlots.Empty())
		{
			//int slotSize = obstacleSlots.Size();
			//int slotIndex = Urho3D::Clamp(Random(slotSize), 0, slotSize - 1);
			for (unsigned int slotIndex = 0; slotIndex < obstacleSlots.Size(); slotIndex++)
			{
				Node* firstFit = obstacleSlots[slotIndex];
				XMLFile* obstacleObj = cache->GetResource<XMLFile>("Objects/Obstacle1.xml");

				if (obstacleObj)
				{
					Vector3 pos = firstFit->GetWorldPosition();
					if (firstFit->GetVars().Size() > 1)
						pos.y_ += 1.0f;

					Node* obstacleNode = scene_->InstantiateXML(obstacleObj->GetRoot(), pos, firstFit->GetWorldRotation());
					obstacleNode->SetParent(blockNode);
				}
			}
		}

		cnt--;
		numBlocks_++;
		blocks_.Push(blockNode);

		// If the last block is the straight then,
		// Go ahead creating the block until the last block is turned one.
		if (cnt == 0 && (rnd == 1 || rnd == 5 || rnd == 6))
		{
			// TODO: You should check the length of straight path.
			cnt++;
		}

		// If the block is the last one that has two way turned, then set the cnt is zero.
		if (rnd == 4)
			cnt = 0;

		lastOutWorldTransform_ = Matrix3x4(outNode->GetWorldPosition(), outNode->GetWorldRotation(), 1);
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
		int outs = block->GetVar(GameVarirants::P_OUT).GetInt();

		Node* paths = block->GetChild("Paths");
		String posix = (startIn || outs == 0) ? "In" : "Out";
		// Check the block whether it has two way outs or not, 
		// then add the "L" or "R" to the posix.
		if (outs >= 2 && !startIn)
		{
			TurnState lastState = character_->GetTurnState();
			if (lastState == NO_SUCCEEDED)
				return;

			if (lastState == LEFT_SUCCEEDED)
			{
				posix += "L";
			}
			else
			{
				posix += "R";
			}

			// Set the last out world transform.
			Node* outNode = block->GetChild(posix);
			lastOutWorldTransform_ = Matrix3x4(outNode->GetWorldPosition(), outNode->GetWorldRotation(), 1);
		}

		Node* path = paths->GetChild("Center" + posix);
		unsigned int numChildren = path->GetNumChildren();

		for (unsigned int i = 0; i < numChildren; i++)
		{
			Node* pointNode = path->GetChild(i);
			centerPoints.Push(pointNode->GetWorldPosition());
			// Create a box-model component to see each path point.
			/*StaticModel* boxObject = pointNode->CreateComponent<StaticModel>();
			boxObject->SetModel(GetSubsystem<ResourceCache>()->GetResource<Model>("Models/Box.mdl"));
			boxObject->SetMaterial(GetSubsystem<ResourceCache>()->GetResource<Material>("Materials/Stone.xml"));*/
		}

		path = paths->GetChild("Left" + posix);
		numChildren = path->GetNumChildren();

		for (unsigned int i = 0; i < numChildren; i++)
		{
			Node* pointNode = path->GetChild(i);
			leftPoints.Push(pointNode->GetWorldPosition());
			// Create a box-model component to see each path point.
			/*StaticModel* boxObject = pointNode->CreateComponent<StaticModel>();
			boxObject->SetModel(GetSubsystem<ResourceCache>()->GetResource<Model>("Models/Box.mdl"));
			boxObject->SetMaterial(GetSubsystem<ResourceCache>()->GetResource<Material>("Materials/Stone.xml"));*/
		}

		path = paths->GetChild("Right" + posix);
		numChildren = path->GetNumChildren();

		for (unsigned int i = 0; i < numChildren; i++)
		{
			Node* pointNode = path->GetChild(i);
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

	character_->AddToPath(LEFT_SIDE, leftPoints);
	character_->AddToPath(RIGHT_SIDE, rightPoints);
	character_->AddToPath(CENTER_SIDE, centerPoints);
}

String AutoRunner::GetRandomCoinObjectName()
{
	String coinObjName = String::EMPTY;
	int point = Random(1, 10);

	if (point == 1)
		coinObjName = "Objects/CoinRed.xml";
	else if (point == 5)
		coinObjName = "Objects/CoinGold.xml";
	else
		coinObjName = "Objects/CoinBlue.xml";

	return coinObjName;
}

void AutoRunner::CreateUI()
{
	ResourceCache* cache = GetSubsystem<ResourceCache>();
	UI* ui = GetSubsystem<UI>();
	UIElement* rootElement = ui->GetRoot();

	// Create a Cursor UI element because we want to be able to hide and show it at will. When hidden, the mouse cursor will
	// control the camera, and when visible, it will point the raycast target
	XMLFile* style = cache->GetResource<XMLFile>("UI/DefaultStyle.xml");

	if (!touch_->touchEnabled_)
	{
		SharedPtr<Cursor> cursor(new Cursor(context_));
		cursor->SetStyleAuto(style);
		ui->SetCursor(cursor);
		// Set starting position of the cursor at the rendering window center
		Graphics* graphics = GetSubsystem<Graphics>();
		cursor->SetPosition(graphics->GetWidth() / 2, graphics->GetHeight() / 2);
	}

	XMLFile* gameMenu = cache->GetResource<XMLFile>("UI/AutoRunnerGameMenu.xml");
	if (gameMenu)
	{
		gameMenu_ = rootElement->CreateChild<Menu>();
		if (gameMenu_->LoadXML(gameMenu->GetRoot(), style))
		{
			Graphics* graphics = GetSubsystem<Graphics>();
			IntVector2 menuSize = gameMenu_->GetSize();
			int height = (int)(graphics->GetHeight() - menuSize.y_) / 2;
			int width = (int)(graphics->GetWidth() - menuSize.x_) / 2;
			gameMenu_->SetPosition(width, height);
			gameMenu_->SetFocus(true);
		}

		String elementName = "LastScoreText";
		Text* lastScoreText = static_cast<Text*>(gameMenu_->GetChild(elementName));
		lastScoreText->SetVisible(false);
		elementName = "HighScoreText";
		Text* highScoreText = static_cast<Text*>(gameMenu_->GetChild(elementName));
		highScoreText->SetVisible(false);
		highScoreText->GetChild(0)->SetVisible(false);
	}

	// Subscribe also to all UI mouse clicks just to see where we have clicked
	SubscribeToEvent(E_UIMOUSECLICK, HANDLER(AutoRunner, HandleControlClicked));
}

void AutoRunner::InitGame()
{
	// Create the controllable character
	CreateCharacter();

	// Set initial parameters
	lastOutWorldTransform_ = Matrix3x4(Vector3(0.0f, 0.0f, -2.0f), Quaternion(90, Vector3(0, 1, 0)), 1);
	yaw_ = pitch_ = 0.0f;
	scoreText_->SetText("Score 0");

	// Set random seed according to the system time
	SetRandomSeed(Time::GetSystemTime());

	// Create level
	CreateLevel();
}

void AutoRunner::ResetGame()
{
	if (!character_)
		return;

	// Remove character.
	Node* characterNode = character_->GetNode();
	characterNode->RemoveComponent(character_);
	characterNode->Remove();
	// Check the last time if we have any block in current scene.
	PODVector<Node*> allChildren;
	scene_->GetChildren(allChildren);
	for (PODVector<Node*>::Iterator it = allChildren.Begin(); it != allChildren.End(); ++it)
	{
		Node* child = *it;
		if (child->GetName().Contains("Block"))
			child->Remove();
	}
	// Remove all blocks.
	blocks_.Clear();
}
