#include "TestScene01.h"
#include "Cube.h"
#include "Gravity.h"
#include "Spring.h"
#include "Sphere.h"
#include "SimulationManager.h"
TestScene01::TestScene01()
{
	this->name = "SpringTest";
}


TestScene01::~TestScene01()
{
}

void TestScene01::initializeScene()
{
	SimulationManager::getInstance()->getSimulation().setCollisionCheck(false);

	Real damper = 50;
	Real constant = 50;
	Cube &cube01 = Cube::create();
	Gravity::create();

	Cube &cube02 = Cube::create();
	Cube &cube03 = Cube::create();
	cube01.setPosition(Vector3d(0, 1.5, 0));
	cube01.setMass(0);
	cube03.setPosition(Vector3d(0.5, -1.5, 0));
	Spring &spring = Spring::create(damper, constant, 1);
	spring.setSuspensionPoints(Vector3d(0, 0, 0), &cube01, Vector3d(0.5, 0.5, 0), &cube02);

	Spring &spring2 = Spring::create(damper, constant, 1);
	spring2.setSuspensionPoints(Vector3d(-0.5, -0.5, 0), &cube02, Vector3d(0.5, 0.5, 0), &cube03);
}
