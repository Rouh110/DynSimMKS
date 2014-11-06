#include "Spring.h"


Spring::Spring()
: damper(1),
springConst(1),
springLength(1),
rigidBodyA(0),
rigidBodyB(0)
{
}


Spring::~Spring()
{
}

void Spring::setDamper(Real damper)
{
	this->damper = damper;
}
void Spring::setSpringConstant(Real springConst)
{
	this->springConst = springConst;
}
void Spring::setSpringLength(Real length)
{
	this->springLength = length;
}

void Spring::setSuspensionPointA(const Vector3d & localPosition, RigidBody * rigidBody)
{
	this->rigidBodyA = rigidBody;
	this->suspensionPointA = localPosition;
}

void Spring::setSuspensionPointB(const Vector3d & localPosition, RigidBody * rigidBody)
{
	this->rigidBodyB = rigidBody;
	this->suspensionPointB = localPosition;
}

void Spring::addForces(Real time)
{
	if (rigidBodyA == 0 || rigidBodyB == 0)
	{
		return;
	}
	
	Vector3d globalPositionA = rigidBodyA->getRotation()._transformVector(suspensionPointA)+rigidBodyA->getPosition();
	Vector3d globalPositionB = rigidBodyB->getRotation()._transformVector(suspensionPointB)+rigidBodyB->getPosition();

	Vector3d forceA;
	Vector3d forceB;

	calculateForces(globalPositionA, rigidBodyA->getVelocity(), globalPositionB, rigidBodyB->getVelocity(), forceA, forceB);

	rigidBodyA->addForce(forceA, suspensionPointA);
	rigidBodyA->addForce(forceB, suspensionPointB);

}

void Spring::calculateForces(const Vector3d & globalPositionA, const Vector3d & velocityA, const Vector3d & globalPositionB, const Vector3d & velocityB, Vector3d out_forceA, Vector3d out_forceB) const
{
	Vector3d d = globalPositionA - globalPositionB;
	Vector3d d_normal = d.normalized();
	Real length = d.norm();
	out_forceA = d_normal*(springConst*(length - springLength) + damper*((velocityA - velocityB).dot(d_normal)));
	out_forceB = out_forceA*-1;

}