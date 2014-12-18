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

using namespace BGE;

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

	// wall
	//for (int i = 0; i < 6; i++)
	//{ 
	//	for (int j = 0; j < 6; j++){
	//		shared_ptr<PhysicsController> box = physicsFactory->CreateBox(5, 5, 5, glm::vec3(5 * j, 5 * i, 0), glm::quat());
	//	}
	//}

	setGravity(glm::vec3(0, -9, 0));
	glm::vec3 pos = glm::vec3(10, 20, 10);
	float radius = 5.2f;
	velocity = 0;

	//Colours
	glm::vec3 camo = glm::vec3((73.0f / 255.0f), (91.0f / 255.0f), (74.0f / 255.0f));
	glm::vec3 camo2 = glm::vec3((139.0f / 255.0f), (69.0f / 255.0f), (19.0f / 255.0f));
	glm::vec3 black = glm::vec3(0, 0, 0);

	// Central cabin
	cabin = physicsFactory->CreateBox(15, 10, 10, pos, glm::quat());
	btVector3 it;
	cabin->shape->calculateLocalInertia(35, it);
	cabin->rigidBody->setMassProps(35, it);
	cabin->transform->diffuse = camo;


	// leftwing
	glm::vec3 leftwingpos = glm::vec3(pos.x, pos.y + 3.5, pos.z + 7);
	leftwing = physicsFactory->CreateBox(3, 3, 4, leftwingpos, glm::quat());
	leftwing->transform->diffuse = camo2;
	btTransform lw1, lw2;
	lw1.setIdentity();
	lw2.setIdentity();
	lw1.setOrigin(btVector3(0, 0, 0));
	lw2.setOrigin(btVector3(0, 3.5, 7));
	btFixedConstraint * fixed30 = new btFixedConstraint(*leftwing->rigidBody, *cabin->rigidBody, lw1, lw2);
	dynamicsWorld->addConstraint(fixed30);

	leftrotor = physicsFactory->CreateCylinder(1.5, 3, glm::vec3(pos.x, pos.y + 11, pos.z + 11.5), glm::quat());
	leftrotor->transform->diffuse = camo;


	// tail
	glm::vec3 tailpos = glm::vec3(pos.x - 13, pos.y + 2, pos.z);
	tail = physicsFactory->CreateBox(10, 4, 4, tailpos, glm::angleAxis(-20.0f, glm::vec3(0, 0, 1)));
	tail->transform->diffuse = camo2;
	btTransform t22, t23;
	t22.setIdentity();
	t23.setIdentity();
	t22.setOrigin(btVector3(0, 0, 0));
	t23.setRotation(GLToBtQuat(glm::angleAxis(-20.0f, glm::vec3(0, 0, 1))));
	t23.setOrigin(btVector3(-13, 2, 0));

	btFixedConstraint * fixed20 = new btFixedConstraint(*tail->rigidBody, *cabin->rigidBody, t22, t23);
	dynamicsWorld->addConstraint(fixed20);

	// tailleft
	glm::vec3 tailleft_pos = glm::vec3(pos.x - 18, pos.y + 4, pos.z + 2.5);
	shared_ptr<PhysicsController> tailleft = physicsFactory->CreateBox(6, 8, 0.5, tailleft_pos, glm::quat());
	tailleft->transform->diffuse = camo;
	btTransform tl1, tl2;
	tl1.setIdentity();
	tl2.setIdentity();
	tl1.setOrigin(btVector3(0, 0, 0));
	tl2.setRotation(GLToBtQuat(glm::angleAxis(20.0f, glm::vec3(0, 0, 1))));
	tl2.setOrigin(btVector3(-6, 0, 2.5));

	btFixedConstraint * fixed22 = new btFixedConstraint(*tailleft->rigidBody, *tail->rigidBody, tl1, tl2);
	dynamicsWorld->addConstraint(fixed22);

	//tailright
	glm::vec3 tailright_pos = glm::vec3(pos.x - 18, pos.y + 4, pos.z - 2.5);
	shared_ptr<PhysicsController> tailright = physicsFactory->CreateBox(6, 8, 0.5, tailright_pos, glm::quat());
	tailright->transform->diffuse = camo;
	btTransform tr1, tr2;
	tr1.setIdentity();
	tr2.setIdentity();
	tr1.setOrigin(btVector3(0, 0, 0));
	tr2.setRotation(GLToBtQuat(glm::angleAxis(20.0f, glm::vec3(0, 0, 1))));
	tr2.setOrigin(btVector3(-6, 0, -2.5));

	btFixedConstraint * fixed23 = new btFixedConstraint(*tailright->rigidBody, *tail->rigidBody, tr1, tr2);
	dynamicsWorld->addConstraint(fixed23);




	// cockpit
	glm::vec3 cockpitpos = glm::vec3(pos.x + 8.5, pos.y, pos.z);
	shared_ptr<PhysicsController> cockpit = physicsFactory->CreateBox(2, 7, 7, cockpitpos, glm::quat());
	cockpit->transform->diffuse = camo;
	//cockpit->rigidBody->setMassProps(40, btVector3(100, 100, 100));
	btTransform t24, t25;
	t24.setIdentity();
	t25.setIdentity();
	t24.setOrigin(btVector3(0, 0, 0));
	t25.setOrigin(btVector3(8.5, 0, 0));

	btFixedConstraint * fixed21 = new btFixedConstraint(*cockpit->rigidBody, *cabin->rigidBody, t24, t25);
	dynamicsWorld->addConstraint(fixed21);

	glm::quat q = glm::angleAxis(-15.0f, glm::vec3(0, 0, 1));

	// Propeller1
	glm::vec3 first_propeller = glm::vec3(pos.x, pos.y + 11, pos.z + radius - 1);
	leftpropeller1 = physicsFactory->CreateBox(4, 0.3, 8, first_propeller, glm::angleAxis(-15.0f, glm::vec3(0, 0, 1)));
	leftpropeller1->transform->diffuse = black;
	btTransform t1, t2;
	t1.setIdentity();
	t2.setIdentity();
	t1.setOrigin(btVector3(0, 0, 0));
	t2.setRotation(GLToBtQuat(glm::angleAxis(+15.0f, glm::vec3(0, 0, 1)))); 
	t2.setOrigin(btVector3(0, 0, -(radius + 1)));

	btFixedConstraint * fixed = new btFixedConstraint(*leftpropeller1->rigidBody, *leftrotor->rigidBody, t1, t2);
	dynamicsWorld->addConstraint(fixed);


	// Propeller2 
	glm::vec3 second_propeller = glm::vec3(pos.x + (radius + 1) * glm::cos(120.0f), pos.y + 11, pos.z + 11.5 + radius * glm::sin(120.0f));
	leftpropeller2 = physicsFactory->CreateBox(4, 0.3, 8, second_propeller, glm::angleAxis(-120.0f, glm::vec3(0, 1, 0)) * glm::angleAxis(-15.0f, glm::vec3(0, 0, 1)));
	leftpropeller2->transform->diffuse = black;
	btTransform t3, t4;
	t3.setIdentity();
	t4.setIdentity();
	t3.setOrigin(btVector3(0, 0, 0));
	t4.setRotation(GLToBtQuat(glm::angleAxis(-120.0f, glm::vec3(0, 1, 0)) * glm::angleAxis(+15.0f, glm::vec3(0, 0, 1))));
	t4.setOrigin(btVector3((radius + 1) * glm::cos(120.0f), 0, radius * glm::sin(120.0f)));

	btFixedConstraint * fixed2 = new btFixedConstraint(*leftpropeller2->rigidBody, *leftrotor->rigidBody, t3, t4);
	dynamicsWorld->addConstraint(fixed2);

	// Propeller3
	glm::vec3 third_propeller = glm::vec3(pos.x - ((radius + 1) * glm::cos(120.0f)), pos.y + 11, pos.z + 11.5 + radius * glm::sin(120.0f));
	leftpropeller3 = physicsFactory->CreateBox(4, 0.3, 8, third_propeller, glm::angleAxis(120.0f, glm::vec3(0, 1, 0)) * glm::angleAxis(-15.0f, glm::vec3(0, 0, 1)));
	leftpropeller3->transform->diffuse = black;
	btTransform t5, t6;
	t5.setIdentity();
	t6.setIdentity();
	t5.setOrigin(btVector3(0, 0, 0));
	t6.setRotation(GLToBtQuat(glm::angleAxis(120.0f, glm::vec3(0, 1, 0)) * glm::angleAxis(+15.0f, glm::vec3(0, 0, 1))));
	t6.setOrigin(btVector3(-((radius + 1) * glm::cos(120.0f)), 0, radius * glm::sin(120.0f)));

	btFixedConstraint * fixed3 = new btFixedConstraint(*leftpropeller3->rigidBody, *leftrotor->rigidBody, t5, t6);
	dynamicsWorld->addConstraint(fixed3);

	//leftpost_hinge
	shared_ptr<PhysicsController> leftpost = physicsFactory->CreateBox(4, 7, 3, glm::vec3(pos.x, pos.y + 6.5, pos.z + 11.5), glm::quat());
	leftpost->transform->diffuse = camo;
	btTransform lh1, lh2;
	lh1.setIdentity();
	lh2 .setIdentity();
	lh1.setOrigin(btVector3(0, 0, 0));
	lh2.setOrigin(btVector3(0, 0, -3.5));

	btFixedConstraint * fixedlh = new btFixedConstraint(*leftwing->rigidBody, *leftpost->rigidBody, lh1, lh2);
	dynamicsWorld->addConstraint(fixedlh);

	leftrotor_hinge = new btHingeConstraint(*leftpost->rigidBody, *leftrotor->rigidBody, btVector3(0, 5, 0), btVector3(0, 0, 0), btVector3(0, 1, 0), btVector3(0, 1, 0), true);
	leftrotor_hinge->setLimit(-glm::pi<float>(), glm::pi<float>());
	dynamicsWorld->addConstraint(leftrotor_hinge);



	// rightwing
	glm::vec3 rightwingpos = glm::vec3(pos.x, pos.y + 3.5, pos.z - 7);
	shared_ptr<PhysicsController> rightwing = physicsFactory->CreateBox(3, 3, 4, rightwingpos, glm::quat());
	rightwing->transform->diffuse = camo2;
	btTransform rw1, rw2;
	rw1.setIdentity();
	rw2.setIdentity();
	rw1.setOrigin(btVector3(0, 0, 0));
	rw2.setOrigin(btVector3(0, 3.5, -7));

	btFixedConstraint * fixed31 = new btFixedConstraint(*rightwing->rigidBody, *cabin->rigidBody, rw1, rw2);
	dynamicsWorld->addConstraint(fixed31);


	shared_ptr<PhysicsController> rightpost = physicsFactory->CreateBox(4, 7, 3, glm::vec3(pos.x, pos.y + 6.5, pos.z - 11.5), glm::quat());
	rightpost->transform->diffuse = camo;
	rightrotor = physicsFactory->CreateCylinder(1.5, 3, glm::vec3(pos.x, pos.y + 11, pos.z - 11.5), glm::quat());
	rightrotor->transform->diffuse = camo;

	// Propeller4
	glm::vec3 fourth_propeller = glm::vec3(pos.x, pos.y + 11, pos.z - radius + 1);
	rightpropeller1 = physicsFactory->CreateBox(4, 0.3, 8, fourth_propeller, q);
	rightpropeller1->transform->diffuse = black;
	btTransform t7, t8;
	t7.setIdentity();
	t8.setIdentity();
	t7.setOrigin(btVector3(0, 0, 0));
	t8.setRotation(GLToBtQuat(glm::angleAxis(+15.0f, glm::vec3(0, 0, 1)))); 
	t8.setOrigin(btVector3(0, 0, (radius + 1)));

	btFixedConstraint * fixed4 = new btFixedConstraint(*rightpropeller1->rigidBody, *rightrotor->rigidBody, t7, t8);
	dynamicsWorld->addConstraint(fixed4);


	// Propeller5 
	glm::vec3 fifth_propeller = glm::vec3(pos.x + (radius + 1) * glm::cos(120.0f), pos.y + 11, pos.z - 11.5 - radius * glm::sin(120.0f));
	rightpropeller2 = physicsFactory->CreateBox(4, 0.3, 8, fifth_propeller, glm::angleAxis(120.0f, glm::vec3(0, 1, 0)) * q);
	rightpropeller2->transform->diffuse = black;
	btTransform t9, t10;
	t9.setIdentity();
	t10.setIdentity();
	t9.setOrigin(btVector3(0, 0, 0));
	t10.setRotation(GLToBtQuat(glm::angleAxis(120.0f, glm::vec3(0, 1, 0)) * glm::angleAxis(+15.0f, glm::vec3(0, 0, 1))));
	t10.setOrigin(btVector3((radius + 1) * glm::cos(120.0f), 0, -(radius * glm::sin(120.0f))));

	btFixedConstraint * fixed5 = new btFixedConstraint(*rightpropeller2->rigidBody, *rightrotor->rigidBody, t9, t10);
	dynamicsWorld->addConstraint(fixed5);

	// Propeller6
	glm::vec3 sixth_propeller = glm::vec3(pos.x - ((radius + 1) * glm::cos(120.0f)), pos.y + 11, pos.z - 11.5 - radius * glm::sin(120.0f));
	rightpropeller3 = physicsFactory->CreateBox(4, 0.3, 8, sixth_propeller, glm::angleAxis(-120.0f, glm::vec3(0, 1, 0)) * q);
	rightpropeller3->transform->diffuse = black;
	btTransform t11, t12;
	t11.setIdentity();
	t12.setIdentity();
	t11.setOrigin(btVector3(0, 0, 0));
	t12.setRotation(GLToBtQuat(glm::angleAxis(-120.0f, glm::vec3(0, 1, 0)) * glm::angleAxis(+15.0f, glm::vec3(0, 0, 1))));
	t12.setOrigin(btVector3(-((radius + 1) * glm::cos(120.0f)), 0, -(radius * glm::sin(120.0f))));

	btFixedConstraint * fixed6 = new btFixedConstraint(*rightpropeller3->rigidBody, *rightrotor->rigidBody, t11, t12);
	dynamicsWorld->addConstraint(fixed6);

	//leftpost_hinge
	btTransform rh1, rh2;
	rh1.setIdentity();
	rh2.setIdentity();
	rh1.setOrigin(btVector3(0, 0, 0));
	rh2.setOrigin(btVector3(0, 0, 3.5));

	btFixedConstraint * fixedrh = new btFixedConstraint(*rightwing->rigidBody, *rightpost->rigidBody, rh1, rh2);
	dynamicsWorld->addConstraint(fixedrh);

	rightrotor_hinge = new btHingeConstraint(*rightpost->rigidBody, *rightrotor->rigidBody, btVector3(0, 5, 0), btVector3(0, 0, 0), btVector3(0, 1, 0), btVector3(0, 1, 0), true);
	rightrotor_hinge->setLimit(-glm::pi<float>(), glm::pi<float>());
	dynamicsWorld->addConstraint(rightrotor_hinge);



	std::shared_ptr<Ground> ground = make_shared<Ground>();
	SetGround(ground);


	return Game::Initialise();
}

void BGE::MyGame::Update()
{

	leftrotor_hinge->enableAngularMotor(true, 100, velocity);
	rightrotor_hinge->enableAngularMotor(true, -100, velocity);

	glm::vec3 upforce = leftpropeller1->transform->up * velocity * 4.0f;
	cabin->rigidBody->applyCentralForce(GLToBtVector(upforce));


	if (cabin->transform->position.y > tail->transform->position.y + 5)
	{
		tail->rigidBody->applyCentralForce(GLToBtVector(tail->transform->up * velocity * 6.0f));
	}

	else if (cabin->transform->position.y < tail->transform->position.y - 5)
	{
		tail->rigidBody->applyCentralForce(GLToBtVector(-(tail->transform->up * velocity * 6.0f)));
	}

	if (leftrotor->transform->position.y > rightrotor->transform->position.y + 5)
	{
		rightrotor->rigidBody->applyCentralForce(GLToBtVector(rightrotor->transform->up * velocity * 6.0f)); 
	}

	else if (rightrotor->transform->position.y > leftrotor->transform->position.y +5)
	{
		leftrotor->rigidBody->applyCentralForce(GLToBtVector(leftrotor->transform->up * velocity * 6.0f));
	}



	if (keyState[SDL_SCANCODE_PAGEDOWN] && velocity > 0)
	{
		velocity -= 6;
	}
	if (keyState[SDL_SCANCODE_PAGEUP] && velocity < 500)
	{
		velocity += 6;	
	}

	
	Game::Update();
}

void BGE::MyGame::Cleanup()
{
	Game::Cleanup();
}
