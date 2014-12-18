#pragma once
#include "Game.h"
#include "PhysicsController.h"
#include "PhysicsFactory.h"
#include <btBulletDynamicsCommon.h>

namespace BGE
{
class MyGame:
		public Game
	{
	private:

public:
	MyGame();
	~MyGame();

	float velocity;
	shared_ptr<PhysicsController> cabin;
	shared_ptr<PhysicsController> tail;
	shared_ptr<PhysicsController> leftwing;

	shared_ptr<PhysicsController> leftpropeller1;
	shared_ptr<PhysicsController> leftpropeller2;
	shared_ptr<PhysicsController> leftpropeller3;

	shared_ptr<PhysicsController> rightpropeller1;
	shared_ptr<PhysicsController> rightpropeller2;
	shared_ptr<PhysicsController> rightpropeller3;

	btHingeConstraint * rightrotor_hinge;
	btHingeConstraint * leftrotor_hinge;

	shared_ptr<PhysicsController> rightrotor;
	shared_ptr<PhysicsController> leftrotor;
		
	bool Initialise();
	void Update();
	void Cleanup();
	void CreateWall();
};
}

