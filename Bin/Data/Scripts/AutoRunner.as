// AutoRunner Game!

#include "Scripts/Utilities/Sample.as"
#include "Scripts/Utilities/Touch.as"

const int CTRL_FORWARD = 1;
const int CTRL_BACK = 2;
const int CTRL_LEFT = 4;
const int CTRL_RIGHT = 8;
const int CTRL_JUMP = 16;

const float MOVE_FORCE = 0.8f;
const float INAIR_MOVE_FORCE = 0.02f;
const float BRAKE_FORCE = 0.2f;
const float JUMP_FORCE = 7.0f;
const float YAW_SENSITIVITY = 0.1f;
const float INAIR_THRESHOLD_TIME = 0.1f;

Scene@ gameScene;
Node@ gameCameraNode;
Node@ characterNode;

void Start()
{
    // Execute the common startup for samples
    SampleStart();

    // Init scene content
    InitScene();

    // Create the controllable character
    CreateCharacter();

    // Activate mobile stuff when appropriate
    if (GetPlatform() == "Android" || GetPlatform() == "iOS")
    {
        SetLogoVisible(false);
        InitTouchInput();
    }

    // Subscribe to necessary events
    SubscribeToEvents();
}

void InitScene()
{
	gameScene = Scene("AutoRunner");
    gameScene.LoadXML(cache.GetFile("Scenes/AutoRunner.xml"));
}

void CreateCamera()
{
    // Note: the camera is not in the scene
    gameCameraNode = Node();
    gameCameraNode.position = Vector3(0, 2, -10);

    gameCamera = gameCameraNode.CreateComponent("Camera");
    gameCamera.nearClip = 0.5;
    gameCamera.farClip = 160;

    if (!engine.headless)
    {
        renderer.viewports[0] = Viewport(gameScene, gameCamera);
    }
}

void CreateCharacter()
{
    characterNode = gameScene.CreateChild("Jack");
    characterNode.position = Vector3(0.0f, 1.0f, 0.0f);

    // Create the rendering component + animation controller
    AnimatedModel@ object = characterNode.CreateComponent("AnimatedModel");
    object.model = cache.GetResource("Model", "Models/Jack.mdl");
    object.material = cache.GetResource("Material", "Materials/Jack.xml");
    object.castShadows = true;
    characterNode.CreateComponent("AnimationController");

    // Set the head bone for manual control
    object.skeleton.GetBone("Bip01_Head").animated = false;

    // Create rigidbody, and set non-zero mass so that the body becomes dynamic
    RigidBody@ body = characterNode.CreateComponent("RigidBody");
    body.collisionLayer = 1;
    body.mass = 1.0f;

    // Set zero angular factor so that physics doesn't turn the character on its own.
    // Instead we will control the character yaw manually
    body.angularFactor = Vector3(0.0f, 0.0f, 0.0f);

    // Set the rigidbody to signal collision also when in rest, so that we get ground collisions properly
    body.collisionEventMode = COLLISION_ALWAYS;

    // Set a capsule shape for collision
    CollisionShape@ shape = characterNode.CreateComponent("CollisionShape");
    shape.SetCapsule(0.7f, 1.8f, Vector3(0.0f, 0.9f, 0.0f));

    // Create the character logic object, which takes care of steering the rigidbody
    characterNode.CreateScriptObject(scriptFile, "Character");
}

void SubscribeToEvents()
{
    // Subscribe to Update event for setting the character controls before physics simulation
    SubscribeToEvent("Update", "HandleUpdate");

    // Subscribe to PostUpdate event for updating the camera position after physics simulation
    SubscribeToEvent("PostUpdate", "HandlePostUpdate");
}


void HandleUpdate(StringHash eventType, VariantMap& eventData)
{
    if (characterNode is null)
        return;

    Character@ character = cast<Character>(characterNode.scriptObject);
    if (character is null)
        return;

    // Clear previous controls
    character.controls.Set(CTRL_FORWARD | CTRL_BACK | CTRL_LEFT | CTRL_RIGHT | CTRL_JUMP, false);
	
	// Update controls using keys (desktop)
	if (ui.focusElement is null)
	{
		character.controls.Set(CTRL_FORWARD, input.keyDown['W']);
		character.controls.Set(CTRL_BACK, input.keyDown['S']);
		character.controls.Set(CTRL_LEFT, input.keyDown['A']);
		character.controls.Set(CTRL_RIGHT, input.keyDown['D']);
		character.controls.Set(CTRL_JUMP, input.keyDown[KEY_SPACE]);

		// Add character yaw & pitch from the mouse motion
		character.controls.yaw += input.mouseMoveX * YAW_SENSITIVITY;
		character.controls.pitch += input.mouseMoveY * YAW_SENSITIVITY;
		// Limit pitch
		character.controls.pitch = Clamp(character.controls.pitch, -80.0f, 80.0f);

		// Switch between 1st and 3rd person
		if (input.keyPress['F'])
			firstPerson = newFirstPerson = !firstPerson;

		// Check for loading / saving the scene
		if (input.keyPress[KEY_F5])
		{
			File saveFile(fileSystem.programDir + "Data/Scenes/CharacterDemo.xml", FILE_WRITE);
			scene_.SaveXML(saveFile);
		}
		if (input.keyPress[KEY_F7])
		{
			File loadFile(fileSystem.programDir + "Data/Scenes/CharacterDemo.xml", FILE_READ);
			scene_.LoadXML(loadFile);
			// After loading we have to reacquire the character scene node, as it has been recreated
			// Simply find by name as there's only one of them
			characterNode = scene_.GetChild("Jack", true);
			if (characterNode is null)
				return;
		}
	}
}

void HandlePostUpdate(StringHash eventType, VariantMap& eventData)
{
    if (characterNode is null)
        return;

    Character@ character = cast<Character>(characterNode.scriptObject);
    if (character is null)
        return;

    // Get camera lookat dir from character yaw + pitch
    Quaternion rot = characterNode.rotation;
    Quaternion dir = rot * Quaternion(character.controls.pitch, Vector3(1.0f, 0.0f, 0.0f));

    // Turn head to camera pitch, but limit to avoid unnatural animation
    Node@ headNode = characterNode.GetChild("Bip01_Head", true);
    float limitPitch = Clamp(character.controls.pitch, -45.0f, 45.0f);
    Quaternion headDir = rot * Quaternion(limitPitch, Vector3(1.0f, 0.0f, 0.0f));
    // This could be expanded to look at an arbitrary target, now just look at a point in front
    Vector3 headWorldTarget = headNode.worldPosition + headDir * Vector3(0.0f, 0.0f, 1.0f);
    headNode.LookAt(headWorldTarget, Vector3(0.0f, 1.0f, 0.0f));
    // Correct head orientation because LookAt assumes Z = forward, but the bone has been authored differently (Y = forward)
    headNode.Rotate(Quaternion(0.0f, 90.0f, 90.0f));

    if (firstPerson)
    {
        // First person camera: position to the head bone + offset slightly forward & up
        cameraNode.position = headNode.worldPosition + rot * Vector3(0.0f, 0.15f, 0.2f);
        cameraNode.rotation = dir;
    }
    else
    {
        // Third person camera: position behind the character
        Vector3 aimPoint = characterNode.position + rot * Vector3(0.0f, 1.7f, 0.0f); // You can modify x Vector3 value to translate the fixed character position (indicative range[-2;2])

        // Collide camera ray with static physics objects (layer bitmask 2) to ensure we see the character properly
        Vector3 rayDir = dir * Vector3(0.0f, 0.0f, -1.0f); // For indoor scenes you can use dir * Vector3(0.0, 0.0, -0.5) to prevent camera from crossing the walls
        float rayDistance = cameraDistance;
        PhysicsRaycastResult result = scene_.physicsWorld.RaycastSingle(Ray(aimPoint, rayDir), rayDistance, 2);
        if (result.body !is null)
            rayDistance = Min(rayDistance, result.distance);
        rayDistance = Clamp(rayDistance, CAMERA_MIN_DIST, cameraDistance);

        cameraNode.position = aimPoint + rayDir * rayDistance;
        cameraNode.rotation = dir;
    }
}
