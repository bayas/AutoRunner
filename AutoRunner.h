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

#include "Sample.h"

namespace Urho3D
{
	class Node;
	class Scene;
}

class Character;
class Touch;

/// This first example, maintaining tradition, prints a "Hello World" message.
/// Furthermore it shows:
///     - Using the Sample / Application classes, which initialize the Urho3D engine and run the main loop
///     - Adding a Text element to the graphical user interface
///     - Subscribing to and handling of update events
class AutoRunner : public Sample
{
    OBJECT(AutoRunner);

public:
    /// Construct.
    AutoRunner(Context* context);

    /// Setup after engine initialization and before running the main loop.
    virtual void Start();

private:
	/// Create static scene content.
	void InitScene();
	/// Create controllable character.
	void CreateCharacter();
	/// Create camera.
	void CreateCamera();
	/// Subscribe to necessary events.
	void SubscribeToEvents();
	/// Read input and moves the camera.
	void MoveCamera(float timeStep);
	/// Handle application update. Set controls to character.
	void HandleUpdate(StringHash eventType, VariantMap& eventData);
	/// Handle application post-update. Update camera position after character has moved.
	void HandlePostUpdate(StringHash eventType, VariantMap& eventData);

	/// Scene.
	SharedPtr<Scene> scene_;
	/// Camera scene node.
	SharedPtr<Node> cameraNode_;
	/// Touch utility object.
	SharedPtr<Touch> touch_;
	/// The controllable character component.
	WeakPtr<Character> character_;
	/// Camera yaw angle.
	float yaw_;
	/// Camera pitch angle.
	float pitch_;
};
