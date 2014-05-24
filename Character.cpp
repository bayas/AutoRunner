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

#include "AnimationController.h"
#include "Character.h"
#include "Context.h"
#include "MemoryBuffer.h"
#include "PhysicsEvents.h"
#include "PhysicsWorld.h"
#include "RigidBody.h"
#include "Scene.h"
#include "SceneEvents.h"
#include "Log.h"
#include "SmoothedTransform.h"
#include "AnimationState.h"
#include "Ray.h"
#include "DebugRenderer.h"
#include "Param.h"
#include "CollisionShape.h"

namespace Urho3D
{
	extern const char* SCENE_CATEGORY;
}

Character::Character(Context* context) :
    LogicComponent(context),
    onGround_(false),
    okToJump_(true),
    inAirTimer_(0.0f),
	score_(0),
	turnRequest_(false),
	inTrigger_(false),
	onJumpGround_(false),
	rolling_(false),
	isDead_(false),
	currentPlatform_(0),
	currentSide_(CharacterSide::CENTER_SIDE),
	jumpState_(JumpState::STOP_JUMPING),
	turnState_(TurnState::NO_SUCCEEDED)
{
	// Only the physics update event is needed: unsubscribe from the rest for optimization
	SetUpdateEventMask(USE_FIXEDUPDATE|USE_POSTUPDATE);
}

void Character::RegisterObject(Context* context)
{
    context->RegisterFactory<Character>(SCENE_CATEGORY);
    
    // These macros register the class attributes to the Context for automatic load / save handling.
    // We specify the Default attribute mode which means it will be used both for saving into file, and network replication
    ATTRIBUTE(Character, VAR_FLOAT, "Controls Yaw", controls_.yaw_, 0.0f, AM_DEFAULT);
    ATTRIBUTE(Character, VAR_FLOAT, "Controls Pitch", controls_.pitch_, 0.0f, AM_DEFAULT);
    ATTRIBUTE(Character, VAR_BOOL, "On Ground", onGround_, false, AM_DEFAULT);
    ATTRIBUTE(Character, VAR_BOOL, "OK To Jump", okToJump_, true, AM_DEFAULT);
    ATTRIBUTE(Character, VAR_FLOAT, "In Air Timer", inAirTimer_, 0.0f, AM_DEFAULT);
}

void Character::Start()
{
	// Component has been inserted into its scene node. Subscribe to events now
	SubscribeToEvent(GetNode(), E_NODECOLLISION, HANDLER(Character, HandleNodeCollision));
	SubscribeToEvent(GetNode(), E_NODECOLLISIONSTART, HANDLER(Character, HandleNodeCollisionStart));
	SubscribeToEvent(GetNode(), E_NODECOLLISIONEND, HANDLER(Character, HandleNodeCollisionEnd));

	animCtrl_ = GetNode()->GetChild("PlayerModel")->GetComponent<AnimationController>();
}

void Character::FixedUpdate(float timeStep)
{
	static float coolDown = 0.0f;
	static int cntLeft = 0;
	static int cntRight = 0;
	static int cntLimit = 20;

	if (coolDown > 0)
		coolDown -= timeStep;

    /// \todo Could cache the components for faster access instead of finding them each frame
    RigidBody* body = GetComponent<RigidBody>();
    
    // Update the in air timer. Reset if grounded
    if (!onGround_)
        inAirTimer_ += timeStep;
    else
        inAirTimer_ = 0.0f;
    // When character has been in air less than 1/10 second, it's still interpreted as being on ground
    bool softGrounded = inAirTimer_ < INAIR_THRESHOLD_TIME;
    
    // Update movement & animation
    const Quaternion& rot = GetNode()->GetRotation();
    Vector3 moveDir = Vector3::ZERO;
    const Vector3& velocity = body->GetLinearVelocity();
    // Velocity on the XZ plane
    Vector3 planeVelocity(velocity.x_, 0.0f, velocity.z_);

	if (inAirTimer_ > 2.0f)
		isDead_ = true;

	if (isDead_)
	{
		if (!animCtrl_->IsPlaying(ANIM_DEATH))
		{
			moveDir = Vector3::BACK;
			body->SetLinearVelocity(Vector3::ZERO);
			body->ApplyImpulse(rot * moveDir * 5.5f);
			//LOGDEBUG("Stopping all animations.");
			animCtrl_->StopAll();
		}

		animCtrl_->Play(ANIM_DEATH, 0, false, 0.2f);
		//LOGDEBUG("Playing death.");
		animCtrl_->SetSpeed(ANIM_DEATH, 0.3f);

		return;
	}
	else
	{
		// Remove closed points by character.
		auto it = runPath_.Find(CharacterSide::CENTER_SIDE);
		for (auto it = runPath_.Begin(); it != runPath_.End(); ++it)
		{
			if (it->second_.Size() > 0)
			{
				Vector3 currentPoint = it->second_.Front();
				Vector3 worldPos = GetNode()->GetWorldPosition();
				currentPoint.y_ = worldPos.y_;
				float length = (worldPos - currentPoint).Length();
				if (length <= 1.0f)
					RemoveFirstPoint();
			}
		}
	}

	if (controls_.IsDown(CTRL_FORWARD))
	{
		moveDir = Vector3::FORWARD;
		body->ApplyImpulse(rot * moveDir * (softGrounded ? MOVE_FORCE : INAIR_MOVE_FORCE));
	}

	if (controls_.IsDown(CTRL_LEFT))
	{
		cntLeft++;
		if (cntLeft > cntLimit)
			turnState_ = TurnState::LEFT_SUCCEEDED;

		if (coolDown <= 0)
			coolDown = .3f;
	}

	if (controls_.IsDown(CTRL_RIGHT))
	{
		cntRight++;
		if (cntRight > cntLimit)
			turnState_ = TurnState::RIGHT_SUCCEEDED;

		if (coolDown <= 0)
			coolDown = .3f;
	}

	if (controls_.IsDown(CTRL_BACK))
		rolling_ = true;

	// Adjusting character collision shape's size and position each movement state.
	if (rolling_ || jumpState_ == LOOP_JUMPING)
	{
		CollisionShape* shape = GetNode()->GetComponent<CollisionShape>();
		shape->SetSize(Vector3(0.7f, 0.6f, 0.7f));
		shape->SetPosition(Vector3(0.0f, 0.5f, 0.0f));
	}
	else
	{
		CollisionShape* shape = GetNode()->GetComponent<CollisionShape>();
		shape->SetSize(Vector3(0.7f, 1.5f, 0.7f));
		shape->SetPosition(Vector3(0.0f, 0.8f, 0.0f));
	}

	if (coolDown <= 0 && !controls_.IsDown(CTRL_LEFT))
	{
		cntLeft = 0;
		turnState_ = TurnState::NO_SUCCEEDED;
	}

	if (coolDown <= 0 && !controls_.IsDown(CTRL_RIGHT))
	{
		cntRight = 0;
		turnState_ = TurnState::NO_SUCCEEDED;
	}

	if (coolDown == .3f && !inTrigger_)
	{
		if (controls_.IsDown(CTRL_LEFT) && CheckSide(CTRL_LEFT))
		{
			moveDir = Vector3::LEFT;
			turnState_ = TurnState::SIDE_LEFT_SUCCEEDED;

			if (jumpState_ == JumpState::LOOP_JUMPING)
				body->ApplyForce(rot * moveDir * MOVE_SIDE_AIR_FORCE);
			else
				body->ApplyImpulse(rot * moveDir * MOVE_SIDE_FORCE);
		}

		if (controls_.IsDown(CTRL_RIGHT) && CheckSide(CTRL_RIGHT))
		{
			moveDir = Vector3::RIGHT;
			turnState_ = TurnState::SIDE_RIGHT_SUCCEEDED;

			if (jumpState_ == JumpState::LOOP_JUMPING)
				body->ApplyForce(rot * moveDir * MOVE_SIDE_AIR_FORCE);
			else
				body->ApplyImpulse(rot * moveDir * MOVE_SIDE_FORCE);
		}
	}

	if (softGrounded)
	{
		// When on ground, apply a braking force to limit maximum ground velocity
		Vector3 brakeForce = -planeVelocity * BRAKE_FORCE;
		body->ApplyImpulse(brakeForce);

		// Jump. Must release jump control inbetween jumps
		if (controls_.IsDown(CTRL_JUMP))
		{
			if (okToJump_)
			{
				body->ApplyImpulse(Vector3::UP * JUMP_FORCE);
				okToJump_ = false;
				//LOGDEBUG("Stopping run.");
				animCtrl_->Stop(ANIM_RUN, 0.2f);
				animCtrl_->Play(ANIM_JUMP_START, 0, false, 0.2f);
				//LOGDEBUG("Playing jump start.");
				jumpState_ = START_JUMPING;
			}
		}
		else
		{
			okToJump_ = true;
		}
	}

	if (jumpState_ == START_JUMPING || jumpState_ == LOOP_JUMPING)
	{
		float minJmpLoop = 0.4f;
		float minDistance = 0.1f;
		PhysicsRaycastResult result;
		Ray ray(GetNode()->GetWorldPosition(), Vector3::DOWN);
		GetNode()->GetScene()->GetComponent<PhysicsWorld>()->RaycastSingle(result, ray, 100.0f, FLOOR_COLLISION_MASK);
		if (result.body_)
		{
			LOGDEBUG("Jump distance from floor: " + String(result.distance_));
			if (result.distance_ < minJmpLoop)
			{
				if (jumpState_ == START_JUMPING)
				{
					animCtrl_->Play(ANIM_JUMP_START, 0, false, 0.2f);
					//LOGDEBUG("Playing jump start.");
				}
				else if (jumpState_ == LOOP_JUMPING)
				{
					if (animCtrl_->IsPlaying(ANIM_JUMP_LOOP))
					{
						//LOGDEBUG("Stopping jump loop.");
						animCtrl_->Stop(ANIM_JUMP_LOOP, 0.2f);
					}

					animCtrl_->Play(ANIM_JUMP_END, 0, false, 0.2f);
					//LOGDEBUG("Playing jump end.");

					if (result.distance_ < minDistance)
					{
						//LOGDEBUG("Stopping jump loop.");
						animCtrl_->Stop(ANIM_JUMP_LOOP, 0.2f);
						jumpState_ = STOP_JUMPING;
					}
				}
			}
			else if (jumpState_ == START_JUMPING)
			{
				//LOGDEBUG("Stopping jump start.");
				animCtrl_->Stop(ANIM_JUMP_START, 0.2f);
				jumpState_ = LOOP_JUMPING;
			}
			else if (jumpState_ == LOOP_JUMPING)
			{
				if (turnState_ == SIDE_LEFT_SUCCEEDED)
				{
					//LOGDEBUG("Stopping jump loop.");
					animCtrl_->Stop(ANIM_JUMP_LOOP, 0.2f);
					//LOGDEBUG("Playing jump left.");
					animCtrl_->Play(ANIM_JUMP_LEFT, 0, false, 0.2f);
				}
				else if (turnState_ == SIDE_RIGHT_SUCCEEDED)
				{
					//LOGDEBUG("Stopping jump loop.");
					animCtrl_->Stop(ANIM_JUMP_LOOP, 0.2f);
					//LOGDEBUG("Playing jump right.");
					animCtrl_->Play(ANIM_JUMP_RIGHT, 0, false, 0.2f);
				}
				else
				{
					if (IsPlayedAnim(ANIM_JUMP_LEFT))
					{
						//LOGDEBUG("Stopping jump left.");
						animCtrl_->Stop(ANIM_JUMP_LEFT, 0.1f);
					}

					if (IsPlayedAnim(ANIM_JUMP_RIGHT))
					{
						//LOGDEBUG("Stopping jump right.");
						animCtrl_->Stop(ANIM_JUMP_RIGHT, 0.1f);
					}

					animCtrl_->Play(ANIM_JUMP_LOOP, 0, true, 0.2f);
					animCtrl_->SetSpeed(ANIM_JUMP_LOOP, 0.4f);
					//LOGDEBUG("Playing jump loop.");
				}
			}
		}
	}

	// Play walk animation if moving on ground, otherwise fade it out
	if (jumpState_ == STOP_JUMPING && softGrounded && !moveDir.Equals(Vector3::ZERO))
	{
		if (rolling_)
		{
			//LOGDEBUG("Stopping run.");
			animCtrl_->Stop(ANIM_RUN, 0.2f);
			//LOGDEBUG("Playing roll.");
			animCtrl_->Play(ANIM_ROLL, 0, false, 0.1f);
			animCtrl_->SetSpeed(ANIM_ROLL, 0.6f);

			if (IsPlayedAnim(ANIM_ROLL))
			{
				//LOGDEBUG("Stopping roll.");
				animCtrl_->Stop(ANIM_ROLL, 0.1f);
				rolling_ = false;
			}
		}
		else
		{
			//LOGDEBUG("Stopping roll.");
			animCtrl_->Stop(ANIM_ROLL, 0.1f);
			//LOGDEBUG("Playing run.");
			animCtrl_->Play(ANIM_RUN, 0, true, 0.2f);
			// Set walk animation speed proportional to velocity
			animCtrl_->SetSpeed(ANIM_RUN, planeVelocity.Length() * 0.3f);
		}
	}
	else
	{
		animCtrl_->Stop(ANIM_RUN, 0.2f);
	}

	// Reset grounded flag for next frame
	onGround_ = false;
}

void Character::PostUpdate(float timeStep)
{
	//LOGDEBUG("Character Node Scale: " + GetNode()->GetWorldScale().ToString());
	return;
}

void Character::DrawDebugGeometry(DebugRenderer* debug, bool depthTest)
{
	debug->AddSphere(Sphere(Vector3::ONE, 10.0f), Color(1, 1, 1));
}

void Character::HandleNodeCollision(StringHash eventType, VariantMap& eventData)
{
	// Check collision contacts and see if character is standing on ground (look for a contact that has near vertical normal)
	using namespace NodeCollision;
	
	MemoryBuffer contacts(eventData[P_CONTACTS].GetBuffer());
	Node* otherNode = reinterpret_cast<Node*>(eventData[P_OTHERNODE].GetPtr());

	// Check turn point
	Variant var = otherNode->GetVar(GameVarirants::P_TURNPOINT);
	if (!var.IsEmpty())
	{
		turnRequest_ = var.GetBool() && (turnState_ != TurnState::NO_SUCCEEDED);
	}

	// Get coin points
	var = otherNode->GetVar(GameVarirants::P_POINT);
	if (!var.IsEmpty())
	{
		score_ += var.GetInt();
		otherNode->Remove();
	}

    while (!contacts.IsEof())
    {
        Vector3 contactPosition = contacts.ReadVector3();
        Vector3 contactNormal = contacts.ReadVector3();
        float contactDistance = contacts.ReadFloat();
        float contactImpulse = contacts.ReadFloat();

        // If contact is below node center and mostly vertical, assume it's a ground contact
        if (contactPosition.y_ < (GetNode()->GetPosition().y_ + 1.0f))
        {
			float level = Abs(contactNormal.y_);
            if (level > 0.75)
                onGround_ = true;
        }
    }
}

void Character::HandleNodeCollisionStart(StringHash eventType, VariantMap& eventData)
{
	using namespace NodeCollisionStart;

	Node* otherNode = reinterpret_cast<Node*>(eventData[P_OTHERNODE].GetPtr());

	// Check turn point
	Variant var = otherNode->GetVar(GameVarirants::P_TURNPOINT);
	if (!var.IsEmpty())
	{
		turnRequest_ = var.GetBool() && (turnState_ != TurnState::NO_SUCCEEDED);
		inTrigger_ = true;
	}

	// Check current platform.
	var = otherNode->GetVar(GameVarirants::P_ISINPLATFORM);
	if (!var.IsEmpty())
	{
		Node* enteringPlatform = otherNode->GetParent();

		if (currentPlatform_)
		{
			if (currentPlatform_->GetID() != enteringPlatform->GetID())
				passedBlocks_.Push(currentPlatform_);
		}

		currentPlatform_ = enteringPlatform;
	}

	// Check obstacles.
	var = otherNode->GetVar(GameVarirants::P_ISOBSTACLE);
	if (!var.IsEmpty())
	{
		isDead_ = true;
	}
}

void Character::HandleNodeCollisionEnd(StringHash eventType, VariantMap& eventData)
{
	using namespace NodeCollisionEnd;

	Node* otherNode = reinterpret_cast<Node*>(eventData[P_OTHERNODE].GetPtr());
	// Check turn point
	Variant var = otherNode->GetVar(GameVarirants::P_TURNPOINT);
	if (!var.IsEmpty())
	{
		turnRequest_ = false;
		inTrigger_ = false;
	}
}

bool Character::CheckSide(int control)
{
	switch (control)
	{
	case CTRL_LEFT:
		{
			switch (currentSide_)
			{
			case LEFT_SIDE:
				return false;
			case RIGHT_SIDE:
				currentSide_ = CENTER_SIDE;
				break;
			case CENTER_SIDE:
				currentSide_ = LEFT_SIDE;
				break;
			}
		}
		break;
	case CTRL_RIGHT:
		{
			switch (currentSide_)
			{
			case LEFT_SIDE:
				currentSide_ = CENTER_SIDE;
				break;
			case RIGHT_SIDE:
				return false;
			case CENTER_SIDE:
				currentSide_ = RIGHT_SIDE;
				break;
			}
		}
		break;
	default:
		return false;
	}

	return true;
}

void Character::AddToPath(CharacterSide side, const List<Vector3>& points)
{
	if (points.Empty())
		return;

	auto it = runPath_.Find(side);
	if (it == runPath_.End())
		runPath_[side] = points;
	else
		it->second_.Insert(it->second_.End(), points);
}

bool Character::GetCurrentPoint(Vector3& point)
{
	auto it = runPath_.Find(currentSide_);
	if (it->second_.Size() == 0)
		return false;

	point = it->second_.Front();
	return true;
}

void Character::RemoveFirstPoint()
{
	runPath_[LEFT_SIDE].PopFront();
	runPath_[RIGHT_SIDE].PopFront();
	runPath_[CENTER_SIDE].PopFront();
}

void Character::Reset()
{
	// Remove all way-points and passed blocks.
	runPath_.Clear();
	RemovePassedBlocks();
	if (!currentPlatform_.Expired())
		currentPlatform_->Remove();

	isDead_ = false;
	rolling_ = false;
	turnRequest_ = false;
	inTrigger_ = false;
	score_ = 0;
	inAirTimer_ = 0.0f;
	currentPlatform_ = 0;
	currentSide_ = CharacterSide::CENTER_SIDE;
	jumpState_ = JumpState::STOP_JUMPING;
	turnState_ = TurnState::NO_SUCCEEDED;

	RigidBody* body = GetComponent<RigidBody>();

	//LOGDEBUG("Stopping all.");
	animCtrl_->StopAll();
	body->SetLinearVelocity(Vector3::ZERO);
}

void Character::FollowPath(float timeStep)
{
	Vector3 nextWaypoint = Vector3::ZERO; 
	if (GetCurrentPoint(nextWaypoint)) {
		nextWaypoint.y_ = GetNode()->GetWorldPosition().y_;
		Vector3 targetPos = nextWaypoint - GetNode()->GetWorldPosition();
		Quaternion targetRotation;
		targetRotation.FromLookRotation(targetPos);
		SmoothedTransform* smooth = GetNode()->GetComponent<SmoothedTransform>();
		float yaw = targetRotation.YawAngle();
		float minYaw = -178.0f;
		float maxYaw = 178.0f;
		//LOGDEBUG("Target Yaw Angle: " + String(yaw));
		yaw = Urho3D::Clamp(yaw, minYaw, maxYaw);
		float absYaw = Urho3D::Abs(yaw);
		if ((absYaw > 0 && absYaw < 1) || yaw == minYaw || yaw == maxYaw)
			yaw = 0;

		if (!targetRotation.IsNaN() && yaw)
		{
			//targetRotation = GetNode()->GetParent()->GetWorldRotation().Inverse() * targetRotation;
			//GetNode()->SetRotation(targetRotation);
			smooth->SetTargetWorldRotation(targetRotation);
			//GetNode()->LookAt(nextWaypoint);
		}
	}
}

bool Character::HasTurnRequest()
{
	if (!turnRequest_)
		return false;

	bool success = false;
	int outs = currentPlatform_->GetVar(GameVarirants::P_OUT).GetInt();
	if (outs > 0) {
		bool leftOut = currentPlatform_->GetVar(GameVarirants::P_LEFTOUT).GetBool();
		bool rightOut = currentPlatform_->GetVar(GameVarirants::P_RIGHTOUT).GetBool();

		success = (leftOut && turnState_ == TurnState::LEFT_SUCCEEDED) || (rightOut && turnState_ == TurnState::RIGHT_SUCCEEDED);
	}

	return success;
}

void Character::RemovePassedBlocks()
{
	if (passedBlocks_.Size() <= 0)
		return;

	for (auto it = passedBlocks_.Begin(); it != passedBlocks_.End(); ++it)
		(*it)->Remove();

	passedBlocks_.Clear();
}

bool Character::IsPlayedAnim(const String& name) const
{
	bool played = false;

	if (animCtrl_->IsPlaying(name))
	{
		float diff = animCtrl_->GetLength(name) - animCtrl_->GetTime(name);
		played = (diff == 0);
	}

	return played;
}