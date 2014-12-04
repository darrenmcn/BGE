#pragma once
#include "GameComponent.h"
#include "PhysicsController.h"
#include "PhysicsFactory.h"
#include <btBulletDynamicsCommon.h>

namespace BGE
{
	class GravityController :
		public GameComponent
	{
	private:

	public:
		GravityController();
		~GravityController();
		bool Initialise();
		void Update();

		glm::vec3 gravity;
	};
}
