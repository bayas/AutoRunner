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
#include "DebugRenderer.h"

namespace Urho3D
{
	class Node;
	class Scene;
	class Menu;
	class Text;
}

class Character;
class Touch;

typedef PODVector<Pair<Node*, Node*>> SeperatePath;

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
	/// Create overlays.
	void CreateOverlays();
	/// Subscribe to necessary events.
	void SubscribeToEvents();
	/// Handle application fixed-update. Update character path.
	void HandleFixedUpdate(StringHash eventType, VariantMap& eventData);
	/// Handle application update. Set controls to character.
	void HandleUpdate(StringHash eventType, VariantMap& eventData);
	/// Handle application post-update. Update camera position after character has moved.
	void HandlePostUpdate(StringHash eventType, VariantMap& eventData);
	/// Handle the post-render update event.
	void HandlePostRenderUpdate(StringHash eventType, VariantMap& eventData);
	/// Handle any UI control being clicked.
	void HandleControlClicked(StringHash eventType, VariantMap& eventData);

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
	/// Flag for drawing debug geometry.
	bool drawDebug_;

	/// Game mechanics.
	void CreateUI();
	void InitGame();
	void ResetGame();
	void CreateLevel();
	void UpdatePath(bool startIn = true);
	String GetRandomCoinObjectName();

	bool isPlaying_;
	unsigned int numBlocks_;
	List<Node*> blocks_;
	Matrix3x4 lastOutWorldTransform_;
	Text* scoreText_;
	Menu* gameMenu_;
	Node* characterHead_;
	PODVector<DebugLine> lines_;
	PODVector<Sphere> spheres_;

};
