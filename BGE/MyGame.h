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
	bool Initialise();
	void Update();
	void Cleanup();
	void CreateWall();
};
}

