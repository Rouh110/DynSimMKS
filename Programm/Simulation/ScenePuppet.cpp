#include "ScenePuppet.h"
#include "Sphere.h"
#include "Gravity.h"
#include "Cube.h"
#include "Spring.h"
#include "SphericalJoint.h"
#include "SimulationManager.h"

ScenePuppet::ScenePuppet()
{
	name = "WoodPuppet";
}


ScenePuppet::~ScenePuppet()
{
}

void ScenePuppet::initializeScene()
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


	// Joints to connet
	SphericalJoint & joint01 = SphericalJoint::create(&point,&head, Vector3d(0,4,0));

	SphericalJoint & joint02 = SphericalJoint::create(&head, &shoulder, Vector3d(0, 2.5, 0));

	SphericalJoint & joint03 = SphericalJoint::create(&shoulder, &torso, Vector3d(0.3, 2, 0));
	SphericalJoint & joint04 = SphericalJoint::create(&shoulder, &torso, Vector3d(-0.3, 2, 0));


	SphericalJoint & joint05 = SphericalJoint::create(&shoulder, &rightArm, Vector3d(0.8, 2, 0));

	SphericalJoint & joint06 = SphericalJoint::create(&shoulder, &leftArm, Vector3d(-0.8, 2, 0));

	SphericalJoint & joint07 = SphericalJoint::create(&rightArm, &rightUnderArm, Vector3d(0.8, 0.9, 0));

	SphericalJoint & joint08 = SphericalJoint::create(&leftArm, &leftUnderArm, Vector3d(-0.8, 0.9, 0));

	SphericalJoint & joint09 = SphericalJoint::create(&torso, &rightUpperLeg, Vector3d(0.3, 0.0, 0));

	SphericalJoint & joint10 = SphericalJoint::create(&torso, &leftUpperLeg, Vector3d(-0.3, 0.0, 0));

	SphericalJoint & joint11 = SphericalJoint::create(&rightUpperLeg, &rightShin, Vector3d(0.3, -1.2, 0));

	SphericalJoint & joint12 = SphericalJoint::create(&leftUpperLeg, &leftShin, Vector3d(-0.3, -1.2, 0));
	




	// Pendular to make it interisting
	Sphere &pendular = Sphere::create(0.2);
	pendular.setPosition(Vector3d(1, 4, 3));

	Spring & spring13 = Spring::create(damper, springConstant, 1);
	spring13.setSuspensionPoints(Vector3d(0.0, -0.5, 0), &leftShin, Vector3d(0, 0, 0), &pendular);
}
