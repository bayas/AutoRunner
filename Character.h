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

#pragma once

#include "Controls.h"
#include "LogicComponent.h"
#include "List.h"

#define BIT(x) (1<<(x))

namespace Urho3D
{
	class AnimationController;
}

using namespace Urho3D;

const int CTRL_FORWARD = BIT(0);
const int CTRL_BACK = BIT(1);
const int CTRL_LEFT = BIT(2);
const int CTRL_RIGHT = BIT(3);
const int CTRL_JUMP = BIT(4);

const float MOVE_FORCE = 1.3f;
const float MOVE_SIDE_FORCE = 13.0f;
const float MOVE_SIDE_AIR_FORCE = 120.0f;
const float INAIR_MOVE_FORCE = 0.02f;
const float BRAKE_FORCE = 0.2f;
const float JUMP_FORCE = 5.0f;
const float YAW_SENSITIVITY = 0.1f;
const float INAIR_THRESHOLD_TIME = 0.1f;

const unsigned int FLOOR_COLLISION_MASK = BIT(1);
const unsigned int COIN_COLLISION_MASK = BIT(2);
const unsigned int OBSTACLE_COLLISION_MASK = BIT(3);

const String ANIM_RUN = "Models/vempire_run.ani";
const String ANIM_ROLL = "Models/vempire_roll.ani";
const String ANIM_DEATH = "Models/vempire_death.ani";
const String ANIM_JUMP_END = "Models/vempire_jmpEnd.ani";
const String ANIM_JUMP_LEFT = "Models/vempire_jmpLeft.ani";
const String ANIM_JUMP_LOOP = "Models/vempire_jmpLoop.ani";
const String ANIM_JUMP_START = "Models/vempire_jmpStart.ani";
const String ANIM_JUMP_RIGHT = "Models/vempire_jmpRight.ani";

enum CharacterSide
{
	LEFT_SIDE = 0,
	RIGHT_SIDE,
	CENTER_SIDE
};

enum JumpState
{
	START_JUMPING = 0,
	LOOP_JUMPING,
	STOP_JUMPING
};

enum TurnState
{
	NO_SUCCEEDED = 0,
	LEFT_SUCCEEDED,
	RIGHT_SUCCEEDED,
	SIDE_LEFT_SUCCEEDED,
	SIDE_RIGHT_SUCCEEDED
};

typedef HashMap<unsigned, List<Vector3>> RunPath;

/// Character component, responsible for physical movement according to controls, as well as animation.
class Character : public LogicComponent
{
    OBJECT(Character)

public:
    /// Construct.
    Character(Context* context);

    /// Register object factory and attributes.
    static void RegisterObject(Context* context);
    
    /// Handle startup. Called by LogicComponent base class. Creates model and loads animations.
	virtual void Start();
    /// Handle physics world update. Called by LogicComponent base class.
	virtual void FixedUpdate(float timeStep);
	/// Handle scene post-update, Called by LogicCOmponent base class.
	virtual void PostUpdate(float timeStep);
	/// Visualize the component as debug geometry.
	virtual void DrawDebugGeometry(DebugRenderer* debug, bool depthTest);

    /// Movement controls. Assigned by the main program each frame.
    Controls controls_;

	void Reset();
	void FollowPath(float timeStep);
	bool HasTurnRequest();
	bool GetCurrentPoint(Vector3& point);
	void RemoveFirstPoint();
	void RemovePassedBlocks();
	void AddToPath(CharacterSide side, const List<Vector3>& points);

	int GetScore() { return score_; }
	CharacterSide GetSide() { return currentSide_; }
	unsigned int GetNumPoints() { return runPath_[currentSide_].Size(); }
	TurnState GetTurnState() { return turnState_; }
	bool IsDead() { return isDead_; }
	bool OnGround() { return onGround_; }
	void SetCurrentPlatform(Node* platform) { currentPlatform_ = platform; }

private:
    /// Handle physics collision events.
	void HandleNodeCollision(StringHash eventType, VariantMap& eventData);
	void HandleNodeCollisionStart(StringHash eventType, VariantMap& eventData);
	void HandleNodeCollisionEnd(StringHash eventType, VariantMap& eventData);

    /// Grounded flag for movement.
    bool onGround_;
    /// Jump flag.
    bool okToJump_;
    /// In air timer. Due to possible physics inaccuracy, character can be off ground for max. 1/10 second and still be allowed to move.
    float inAirTimer_;

	/// Game mechanics.
	bool CheckSide(int control);
	bool IsPlayedAnim(const String& name) const;

	int score_;
	bool turnRequest_;
	bool inTrigger_;
	bool onJumpGround_;
	bool rolling_;
	bool isDead_;

	AnimationController* animCtrl_;
	CharacterSide currentSide_;
	JumpState jumpState_;
	TurnState turnState_;

	RunPath runPath_;
	WeakPtr<Node> currentPlatform_;
	PODVector<Node*> passedBlocks_;

};
