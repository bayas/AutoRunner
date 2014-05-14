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

Character::Character(Context* context) :
    LogicComponent(context),
    onGround_(false),
    okToJump_(true),
    inAirTimer_(0.0f),
	score_(0),
	turnRequest_(false),
	inTrigger_(false),
	onJumpGround_(false),
	currentPlatform_(0),
	currentSide_(CharacterSide::CENTER_SIDE),
	jumpState_(JumpState::STOP_JUMPING)
{
	// Only the physics update event is needed: unsubscribe from the rest for optimization
	SetUpdateEventMask(USE_FIXEDUPDATE|USE_POSTUPDATE);
}

void Character::RegisterObject(Context* context)
{
    context->RegisterFactory<Character>();
    
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
}

float coolDown = 0.0f;
int cntLeft = 0;
int cntRight = 0;
int cntLimit = 20;

void Character::FixedUpdate(float timeStep)
{
	if (coolDown > 0)
		coolDown -= timeStep;

    /// \todo Could cache the components for faster access instead of finding them each frame
    RigidBody* body = GetComponent<RigidBody>();
    AnimationController* animCtrl = node_->GetChild("PlayerModel")->GetComponent<AnimationController>();
    
    // Update the in air timer. Reset if grounded
    if (!onGround_)
        inAirTimer_ += timeStep;
    else
        inAirTimer_ = 0.0f;
    // When character has been in air less than 1/10 second, it's still interpreted as being on ground
    bool softGrounded = inAirTimer_ < INAIR_THRESHOLD_TIME;
    
    // Update movement & animation
    const Quaternion& rot = node_->GetRotation();
    Vector3 moveDir = Vector3::ZERO;
    const Vector3& velocity = body->GetLinearVelocity();
    // Velocity on the XZ plane
    Vector3 planeVelocity(velocity.x_, 0.0f, velocity.z_);

	if (controls_.IsDown(CTRL_FORWARD))
	{
		moveDir = Vector3::FORWARD;
		body->ApplyImpulse(rot * moveDir * (softGrounded ? MOVE_FORCE : INAIR_MOVE_FORCE));
	}

	if (controls_.IsDown(CTRL_LEFT))
	{
		cntLeft++;
		if (coolDown <= 0 && onGround_)
			coolDown = .3f;
	}

	if (controls_.IsDown(CTRL_RIGHT))
	{
		cntRight++;
		if (coolDown <= 0 && onGround_)
			coolDown = .3f;
	}

	if (coolDown <= 0 && !controls_.IsDown(CTRL_LEFT))
		cntLeft = 0;
	if (coolDown <= 0 && !controls_.IsDown(CTRL_RIGHT))
		cntRight = 0;

	if (coolDown == .3f && !inTrigger_)
	{
		if (controls_.IsDown(CTRL_LEFT) && CheckSide(CTRL_LEFT))
		{
			moveDir = Vector3::LEFT;
			body->ApplyImpulse(rot * moveDir * MOVE_SIDE_FORCE);
		}

		if (controls_.IsDown(CTRL_RIGHT) && CheckSide(CTRL_RIGHT))
		{
			moveDir = Vector3::RIGHT;
			if (!inTrigger_)
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
				animCtrl->Stop("Models/vempire_run.ani", 0.2f);
				animCtrl->Play("Models/vempire_jmpStart.ani", 0, false);
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
		Ray ray(node_->GetWorldPosition(), Vector3::DOWN);
		node_->GetScene()->GetComponent<PhysicsWorld>()->RaycastSingle(result, ray, 100.0f, FLOOR_COLLISION_MASK);
		if (result.body_)
		{
			if (result.distance_ < minJmpLoop)
			{
				if (jumpState_ == START_JUMPING)
				{
					animCtrl->Play("Models/vempire_jmpStart.ani", 0, false);
					//LOGDEBUG("Playing jump start.");
				}
				else if (jumpState_ == LOOP_JUMPING)
				{
					if (animCtrl->IsPlaying("Models/vempire_jmpLoop.ani"))
					{
						//LOGDEBUG("Stopping jump loop.");
						animCtrl->Stop("Models/vempire_jmpLoop.ani");
					}

					animCtrl->Play("Models/vempire_jmpEnd.ani", 0, false);
					//LOGDEBUG("Playing jump end.");

					if (result.distance_ < minDistance)
						jumpState_ = STOP_JUMPING;
				}
			}
			else if (jumpState_ == START_JUMPING)
			{
				//LOGDEBUG("Stopping jump start.");
				animCtrl->Stop("Models/vempire_jmpStart.ani");
				jumpState_ = LOOP_JUMPING;
			}
			else if (jumpState_ == LOOP_JUMPING)
			{
				animCtrl->Play("Models/vempire_jmpLoop.ani", 0, true);
				//LOGDEBUG("Playing jump loop.");
			}
		}
	}

	// Play walk animation if moving on ground, otherwise fade it out
	if (jumpState_ == STOP_JUMPING && softGrounded && !moveDir.Equals(Vector3::ZERO))
	{
		//LOGDEBUG("Playing run.");
		animCtrl->Play("Models/vempire_run.ani", 0, true, 0.2f);
		// Set walk animation speed proportional to velocity
		animCtrl->SetSpeed("Models/vempire_run.ani", planeVelocity.Length() * 0.3f);
	}
	else
	{
		animCtrl->Stop("Models/vempire_run.ani", 0.2f);
	}

	// Reset grounded flag for next frame
	onGround_ = false;
}

void Character::PostUpdate(float timeStep)
{
	return;
}

void Character::HandleNodeCollision(StringHash eventType, VariantMap& eventData)
{
	// Check collision contacts and see if character is standing on ground (look for a contact that has near vertical normal)
	using namespace NodeCollision;
	
	MemoryBuffer contacts(eventData[P_CONTACTS].GetBuffer());
	Node* otherNode = reinterpret_cast<Node*>(eventData[P_OTHERNODE].GetPtr());

	// Check turn point
	Variant var = otherNode->GetVar("TurnPoint");
	if (!var.IsEmpty())
	{
		turnRequest_ = var.GetBool() && (cntLeft > cntLimit || cntRight > cntLimit);
	}

	// Get coin points
	var = otherNode->GetVar("Point");
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
        if (contactPosition.y_ < (node_->GetPosition().y_ + 1.0f))
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
	Variant var = otherNode->GetVar("TurnPoint");
	if (!var.IsEmpty())
	{
		turnRequest_ = var.GetBool() && (cntLeft > cntLimit || cntRight > cntLimit);
		inTrigger_ = true;
	}

	// Check current platform.
	var = otherNode->GetVar("IsInPlatform");
	if (!var.IsEmpty())
		currentPlatform_ = otherNode->GetParent();
}

void Character::HandleNodeCollisionEnd(StringHash eventType, VariantMap& eventData)
{
	using namespace NodeCollisionEnd;

	Node* otherNode = reinterpret_cast<Node*>(eventData[P_OTHERNODE].GetPtr());
	// Check turn point
	Variant var = otherNode->GetVar("TurnPoint");
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
	auto it = runPath_.Find(side);
	if (it == runPath_.End())
		runPath_[side] = points;
	else
		it->second_.Insert(it->second_.End(), points);
}

bool Character::GetCurrentPoint(Vector3& point)
{
	auto it = runPath_.Find((unsigned int)currentSide_);
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

void Character::FollowPath(float timeStep)
{
	Vector3 nextWaypoint = Vector3::ZERO; 
	if (GetCurrentPoint(nextWaypoint)) {
		nextWaypoint.y_ = node_->GetWorldPosition().y_;
		Quaternion targetRotation;
		targetRotation.FromLookRotation(nextWaypoint - node_->GetWorldPosition());
		SmoothedTransform* smooth = node_->GetComponent<SmoothedTransform>();
		if (!targetRotation.IsNaN())
			smooth->SetTargetWorldRotation(targetRotation);
		//node_->LookAt(nextWaypoint);
	}
}

bool Character::HasTurnRequest() const
{
	if (!turnRequest_)
		return false;

	bool success = false;
	int outs = currentPlatform_->GetVar("Out").GetInt();
	if (outs > 0) {
		bool leftOut = currentPlatform_->GetVar("LeftOut").GetBool();
		bool rightOut = currentPlatform_->GetVar("RightOut").GetBool();

		if (leftOut && rightOut)
			success = true;
		else if (leftOut)
			success = cntLeft > cntLimit;
		else if (rightOut)
			success = cntRight > cntLimit;
	}

	return success;
}