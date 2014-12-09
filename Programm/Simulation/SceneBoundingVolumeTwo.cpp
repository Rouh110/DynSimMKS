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
Gravity::create();
bouncingball = &Sphere::create(0.4);
bouncingball->setPosition(Eigen::Vector3d(0, 3, 0));
bouncingball->setMass(1);

boundBouncingBall = new BoundingVolume(Eigen::Vector3d(0, 3, 0), 0.4);
BoundingVolumeTree* tree = new BoundingVolumeTree();
BoundingVolumeTreeNode* node = new BoundingVolumeTreeNode();
node->setBoundingVolume(boundBouncingBall);
tree->setRoot(node);

//SphericalJoint & joint01 = SphericalJoint::create(&point, &head, Vector3d(0, 4, 0));
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