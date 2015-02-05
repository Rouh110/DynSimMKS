#include "TestScene03.h"
#include "Sphere.h"
#include "Gravity.h"
#include "Cube.h"
#include "Spring.h"
#include "SimulationManager.h"

TestScene03::TestScene03()
{
	name = "jumpingJack02";
}


void TestScene03::initializeScene()
{
	SimulationManager::getInstance()->getSimulation().setCollisionCheck(false);
	Real damper = 10;
	Real springConstant = 100;

	Gravity::create();

	Sphere & point = Sphere::create(0.1);
	point.setPosition(Vector3d(0, 4, 0));
	point.setMass(0);
	Sphere & head = Sphere::create(0.4);
	head.setPosition(Vector3d(0, 3, 0));
	Cube & shoulder = Cube::create(2, 0.5, 0.5);
	shoulder.setPosition(Vector3d(0, 2.3, 0));
	Cube & torso = Cube::create(1, 2, 0.5);
	torso.setPosition(Vector3d(0, 1, 0));

	Cube & rightArm = Cube::create(0.5, 1, 0.5);
	rightArm.setPosition(Vector3d(0.8, 1.5, 0));

	Cube & leftArm = Cube::create(0.5, 1, 0.5);
	leftArm.setPosition(Vector3d(-0.8, 1.5, 0));

	Cube & rightUnderArm = Cube::create(0.5, 1, 0.5);
	rightUnderArm.setPosition(Vector3d(0.8, 0.4, 0));

	Cube & leftUnderArm = Cube::create(0.5, 1, 0.5);
	leftUnderArm.setPosition(Vector3d(-0.8, 0.4, 0));

	Cube & rightUpperLeg = Cube::create(0.5, 1, 0.5);
	rightUpperLeg.setPosition(Vector3d(0.3, -0.7, 0));

	Cube & leftUpperLeg = Cube::create(0.5, 1, 0.5);
	leftUpperLeg.setPosition(Vector3d(-0.3, -0.7, 0));

	Cube & rightShin = Cube::create(0.5, 1, 0.5);
	rightShin.setPosition(Vector3d(0.3, -1.9, 0));

	Cube & leftShin = Cube::create(0.5, 1, 0.5);
	leftShin.setPosition(Vector3d(-0.3, -1.9, 0));


	// Springs to connet

	Spring & spring1 = Spring::create(damper, springConstant * 2, 1);
	spring1.setSuspensionPoints(Vector3d(0, 0, 0), &point, Vector3d(0, 0.4, 0), &head);

	Spring & spring2 = Spring::create(damper*0.5, springConstant * 10, 1);
	spring2.setSuspensionPoints(Vector3d(0, -0.4, 0), &head, Vector3d(0, 0.25, 0), &shoulder);

	Spring & spring3 = Spring::create(damper, springConstant, 1);
	spring3.setSuspensionPoints(Vector3d(0.3, -0.25, 0), &shoulder, Vector3d(0.3, 1, 0), &torso);
	Spring & spring4 = Spring::create(damper, springConstant, 1);
	spring4.setSuspensionPoints(Vector3d(-0.3, -0.25, 0), &shoulder, Vector3d(-0.3, 1, 0), &torso);

	Spring & spring5 = Spring::create(damper, springConstant, 1);
	spring5.setSuspensionPoints(Vector3d(0.8, -0.25, 0), &shoulder, Vector3d(0, 0.5, 0), &rightArm);

	Spring & spring6 = Spring::create(damper, springConstant, 1);
	spring6.setSuspensionPoints(Vector3d(-0.8, -0.25, 0), &shoulder, Vector3d(0, 0.5, 0), &leftArm);

	Spring & spring7 = Spring::create(damper, springConstant, 1);
	spring7.setSuspensionPoints(Vector3d(0, -0.5, 0), &rightArm, Vector3d(0, 0.5, 0), &rightUnderArm);

	Spring & spring8 = Spring::create(damper, springConstant, 1);
	spring8.setSuspensionPoints(Vector3d(0, -0.5, 0), &leftArm, Vector3d(0, 0.5, 0), &leftUnderArm);

	Spring & spring9 = Spring::create(damper, springConstant, 1);
	spring9.setSuspensionPoints(Vector3d(0.3, -1, 0), &torso, Vector3d(0, 0.5, 0), &rightUpperLeg);

	Spring & spring10 = Spring::create(damper, springConstant, 1);
	spring10.setSuspensionPoints(Vector3d(-0.3, -1, 0), &torso, Vector3d(0, 0.5, 0), &leftUpperLeg);

	Spring & spring11 = Spring::create(damper, springConstant, 1);
	spring11.setSuspensionPoints(Vector3d(0.0, -0.5, 0), &rightUpperLeg, Vector3d(0, 0.5, 0), &rightShin);

	Spring & spring12 = Spring::create(damper, springConstant, 1);
	spring12.setSuspensionPoints(Vector3d(0.0, -0.5, 0), &leftUpperLeg, Vector3d(0, 0.5, 0), &leftShin);




	// Pendular to make it interisting
	Sphere &pendular = Sphere::create(0.2);
	pendular.setPosition(Vector3d(1, 4, 3));

	Spring & spring13 = Spring::create(damper, springConstant, 1);
	spring13.setSuspensionPoints(Vector3d(0.0, -0.5, 0), &leftShin, Vector3d(0, 0, 0), &pendular);
}

TestScene03::~TestScene03()
{
}
