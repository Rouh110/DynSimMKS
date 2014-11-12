#include "SphericalJoint.h"
#include "Visualization\MiniGL.h"

using namespace IBDS;

SphericalJoint::SphericalJoint()
{
}


SphericalJoint::~SphericalJoint()
{
}

void SphericalJoint::getCurrentError(Vector3d & out_Error)
{
	Vector3d globalPointA;
	Vector3d globalPointB;
	getGlobalPointA(globalPointA);
	getGlobalPointB(globalPointB);

	out_Error = globalPointB - globalPointA;
}

void SphericalJoint::render()
{
	Vector3d globalPointA;
	Vector3d globalPointB;
	getGlobalPointA(globalPointA);
	getGlobalPointB(globalPointB);

	MiniGL::drawSphere(&globalPointA,0.2,MiniGL::blue);
	MiniGL::drawVector(rigidBodyA->getPosition(), globalPointA,4,MiniGL::darkblue);
	MiniGL::drawVector(rigidBodyB->getPosition(), globalPointB, 4, MiniGL::darkblue);
}


SphericalJoint & SphericalJoint::create()
{
	SphericalJoint * joint = new SphericalJoint();
	joint->addToObjectManager();
	return *joint;
}

SphericalJoint & SphericalJoint::create(RigidBody * rigidBodyA, RigidBody *rigidBodyB, const Vector3d & jointPoint)
{
	SphericalJoint * joint = new SphericalJoint();
	joint->setJointPoints(rigidBodyA, rigidBodyB, jointPoint);
	joint->addToObjectManager();
	return *joint;
}
