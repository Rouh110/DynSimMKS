#include "TestScene01.h"
#include "Cube.h"
#include "Gravity.h"
#include "Spring.h"
#include "Sphere.h"
TestScene01::TestScene01()
{
}


TestScene01::~TestScene01()
{
}

void TestScene01::initializeScene()
{
	Cube &cube01 = Cube::create();
	Gravity::create();

	Cube &cube02 = Cube::create();
	Cube &cube03 = Cube::create();
	cube01.setPosition(Vector3d(0, 1.5, 0));
	cube01.setMass(0);
	cube03.setPosition(Vector3d(0, -1.5, 0));
	Spring &spring = Spring::create();
	spring.setSuspensionPoints(Vector3d(0, 0, 0), &cube01, Vector3d(0.5, 0.5, 0), &cube02);

	Spring &spring2 = Spring::create();
	spring2.setSuspensionPoints(Vector3d(-0.5, -0.5, 0), &cube02, Vector3d(0.5, 0.5, 0), &cube03);
}
