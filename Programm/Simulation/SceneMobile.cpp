#include "SceneMobile.h"
#include "Cube.h"
#include "Gravity.h"
#include "SphericalJoint.h"
#include "Spring.h"
#include "Sphere.h"

SceneMobile::SceneMobile()
{
	name = "Mobile";
}


SceneMobile::~SceneMobile()
{
}


void SceneMobile::initializeScene()
{
	Real y = 2;

	Gravity::create();

	Cube & cube01 = Cube::create();
	cube01.setMass(0);
	cube01.setPosition(Vector3d(0, y, 0));
	Cube & cube02 = Cube::create();
	cube02.setPosition(Vector3d(1, y, 0));
	Cube & cube03 = Cube::create();
	cube03.setPosition(Vector3d(2, y, 0));

	Cube & cube04 = Cube::create();
	cube04.setPosition(Vector3d(3, y, 0.5));
	Cube & cube05 = Cube::create();
	cube05.setPosition(Vector3d(4, y, 0));
	Cube & cube06 = Cube::create(2, 0.5, 0.5);
	cube06.setPosition(Vector3d(5, 1 + y, 0));
	Cube & cube07 = Cube::create(2, 0.5, 0.5);
	cube07.setPosition(Vector3d(5, -1 + y, 0));


	SphericalJoint &joint01 = SphericalJoint::create(&cube01, &cube02, Vector3d(0.5, -0.5 + y, 0));
	SphericalJoint &joint02 = SphericalJoint::create(&cube02, &cube03, Vector3d(1.5, -0.5 + y, 0.0));

	SphericalJoint &joint03 = SphericalJoint::create(&cube03, &cube04, Vector3d(2.5, -0.5 + y, 0));
	SphericalJoint &joint04 = SphericalJoint::create(&cube04, &cube05, Vector3d(3.5, 0.0 + y, 0));

	SphericalJoint &joint05 = SphericalJoint::create(&cube06, &cube05, Vector3d(4, 1 + y, 0));
	SphericalJoint &joint06 = SphericalJoint::create(&cube07, &cube05, Vector3d(4, -1 + y, 0));


	Cube & cube00 = Cube::create();
	cube00.setPosition(Vector3d(4, 1 + y, 0));
	cube00.setMass(0);


	//Spring & spring01 = Spring::create();
	//spring01.setSuspensionPoints(Vector3d(0, 0, 0), &cube00, Vector3d(0, 0.5,0), &cube03);
}
