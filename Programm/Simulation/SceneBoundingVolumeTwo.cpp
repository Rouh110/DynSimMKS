#include "SceneBoundingVolumeTwo.h"
#include "Gravity.h"
#include "SphericalJoint.h"

SceneBoundingVolumeTwo::SceneBoundingVolumeTwo()
{
}


SceneBoundingVolumeTwo::~SceneBoundingVolumeTwo()
{
}
void SceneBoundingVolumeTwo::initializeScene(){
timer = 0;
Real damper = 10;
Real springConstant = 100;

Gravity::create();


Sphere &bouncingball = Sphere::create(0.4);
bouncingball.setPosition(Eigen::Vector3d(0, 3, 0));
bouncingball.setMass(0);
Sphere &bouncingballtwo = Sphere::create(0.4);
bouncingballtwo.setPosition(Eigen::Vector3d(0, 0, 0));
Sphere &bouncingballthree = Sphere::create(0.4);
bouncingballthree.setPosition(Eigen::Vector3d(0, -2, 0));
Sphere &bouncingballfour = Sphere::create(0.4);
bouncingballfour.setPosition(Eigen::Vector3d(0, -4, 0));
Sphere &bouncingballfive = Sphere::create(0.4);
bouncingballfive.setPosition(Eigen::Vector3d(-1, -3, 0));


Sphere &bouncingballsix = Sphere::create(0.4);
bouncingballsix.setPosition(Eigen::Vector3d(3, 3, 0));
bouncingballsix.setMass(0);
Sphere &bouncingballseven = Sphere::create(0.4);
bouncingballseven.setPosition(Eigen::Vector3d(3, 0, 0));
Sphere &bouncingballeight = Sphere::create(0.4);
bouncingballeight.setPosition(Eigen::Vector3d(3, -2, 0));
Sphere &bouncingballnine = Sphere::create(0.4);
bouncingballnine.setPosition(Eigen::Vector3d(3, -4, 0));
Sphere &bouncingballten = Sphere::create(0.4);
bouncingballten.setPosition(Eigen::Vector3d(4, -3, 0));


SphericalJoint & joint01 = SphericalJoint::create(&bouncingball, &bouncingballtwo, Vector3d(0, 1.5, 0));
SphericalJoint & joint02 = SphericalJoint::create(&bouncingballtwo, &bouncingballthree, Vector3d(0, -1, 0));
SphericalJoint & joint03 = SphericalJoint::create(&bouncingballthree, &bouncingballfour, Vector3d(0, -3, 0));
SphericalJoint & joint04 = SphericalJoint::create(&bouncingballfour, &bouncingballfive, Vector3d(0, -5, 0));

SphericalJoint & joint05 = SphericalJoint::create(&bouncingballsix, &bouncingballseven, Vector3d(3, 1.5, 0));
SphericalJoint & joint06 = SphericalJoint::create(&bouncingballseven, &bouncingballeight, Vector3d(3, -1, 0));
SphericalJoint & joint07 = SphericalJoint::create(&bouncingballeight, &bouncingballnine, Vector3d(3,-3, 0));
SphericalJoint & joint08 = SphericalJoint::create(&bouncingballnine, &bouncingballten, Vector3d(3, -5, 0));
}
void SceneBoundingVolumeTwo::update(Real currentTime)
{
	int timeFactor = 5;
	Real impulseStrength = 0.1;

	// set Timer
	if (timer < (int)(currentTime / timeFactor))
	{
		timer++;
	}

}