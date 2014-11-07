#include "GravityController.h"
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


namespace BGE
{
BGE::GravityController::GravityController(void)
{
	gravity = glm::vec3(0, 9.8, 0);
}


BGE::GravityController::~GravityController(void)
{
}

bool BGE::GravityController::Initialise()
{
	return GameComponent::Initialise();
}

void BGE::GravityController::Update()
{
	transform->velocity += gravity * Time::deltaTime;
	transform->position += transform->velocity * Time::deltaTime;

	if (transform->position.y - transform->scale.y < 0)
	{
		transform->velocity = -transform->velocity;
		transform->position.y = transform->scale.y;
	}


}
}