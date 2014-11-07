#include "MyGame.h"
#include "PhysicsController.h"
#include "Sphere.h"
#include "PhysicsCamera.h"
#include "Box.h"
#include "Cylinder.h"
#include "Steerable3DController.h"
#include "Ground.h"
#include "Content.h"
#include <btBulletDynamicsCommon.h>
#include <gtc/quaternion.hpp>
#include <gtx/quaternion.hpp>
#include <gtx/euler_angles.hpp>
#include <gtx/norm.hpp>
#include "VectorDrawer.h"
#include "Utils.h"


BGE::MyGame::MyGame(void)
{
}


BGE::MyGame::~MyGame(void)
{
}

bool BGE::MyGame::Initialise()
{
	physicsFactory->CreateGroundPhysics();
	physicsFactory->CreateCameraPhysics();

	setGravity(glm::vec3(0, -9, 0));

	// wall
	for (int i = 0; i < 6; i++)
	{ 
		for (int j = 0; j < 6; j++){
			shared_ptr<PhysicsController> box = physicsFactory->CreateBox(5, 5, 5, glm::vec3(5 * j, 5 * i, 0), glm::quat());
		}
	}
	

	std::shared_ptr<Ground> ground = make_shared<Ground>();
	SetGround(ground);


	return Game::Initialise();
}

void BGE::MyGame::Update()
{

	Game::Update();
}

void BGE::MyGame::Cleanup()
{
	Game::Cleanup();
}
