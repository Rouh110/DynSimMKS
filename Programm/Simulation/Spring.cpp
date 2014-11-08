#include "Spring.h"


Spring::Spring()
: damper(20),
springConst(50),
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

void Spring::setSuspensionPoints(const Vector3d & localPositionA, RigidBody * rigidBodyA, const Vector3d & localPositionB, RigidBody * rigidBodyB, bool setLength )
{
	setSuspensionPointA(localPositionA, rigidBodyA);
	setSuspensionPointB(localPositionB, rigidBodyB);

	if (setLength)
	{
		Vector3d globalPosA;
		Vector3d globalPosB;

		getGlobalPositionA(globalPosA);
		getGlobalPositionB(globalPosB);
		setSpringLength((globalPosB - globalPosA).norm());
	}

}

void Spring::computeForce(Real time)
{
	
	if (rigidBodyA == 0 || rigidBodyB == 0)
	{
		return;
	}
	
	Vector3d globalPositionA;// = rigidBodyA->getRotation()._transformVector(suspensionPointA) + rigidBodyA->getPosition();
	Vector3d globalPositionB;// = rigidBodyB->getRotation()._transformVector(suspensionPointB) + rigidBodyB->getPosition();
	getGlobalPositionA(globalPositionA);
	getGlobalPositionB(globalPositionB);

	Vector3d forceA;
	Vector3d forceB;

	calculateForces(globalPositionA, rigidBodyA->getVelocity(), globalPositionB, rigidBodyB->getVelocity(), forceA, forceB);

	//rigidBodyA->addForce(forceA);
	//rigidBodyA->addTorqe((globalPositionA - rigidBodyA->getPosition()).cross(forceA));
	
	//rigidBodyB->addForce(forceB);
	//rigidBodyB->addTorqe((globalPositionB-rigidBodyB->getPosition()).cross(forceB));

	//rigidBodyA->addForce(forceA, suspensionPointA);
	//rigidBodyB->addForce(forceB, suspensionPointB);

	rigidBodyA->addForceAtGlobalPosition(forceA, globalPositionA);
	rigidBodyB->addForceAtGlobalPosition(forceB, globalPositionB);

	//printf("pointA")
	
}

void Spring::calculateForces(const Vector3d & globalPositionA, const Vector3d & velocityA, const Vector3d & globalPositionB, const Vector3d & velocityB, Vector3d &out_forceA, Vector3d &out_forceB) const
{
	Vector3d d = globalPositionA - globalPositionB;
	Vector3d d_normal = d.normalized();
	Real length = d.norm();
	Real value = (springConst*(length - springLength) + damper*((velocityA - velocityB).dot(d_normal)));
	out_forceB = (springConst*(length - springLength) + damper*((velocityA - velocityB).dot(d_normal)))*d_normal;
	out_forceA = out_forceB*-1;

}

void Spring::getGlobalPositionA(Vector3d & out_position) const
{
	out_position = rigidBodyA->getRotation()._transformVector(suspensionPointA) + rigidBodyA->getPosition();
}

void Spring::getGlobalPositionB(Vector3d & out_position) const
{
	out_position = rigidBodyB->getRotation()._transformVector(suspensionPointB) + rigidBodyB->getPosition();
}

Spring & Spring::create()
{
	Spring * spring = new Spring();
	spring->addToObjectManager();
	return *spring;
}

Spring & Spring::create(Real damper, Real springConst, Real springLength)
{
	Spring *spring = new Spring();
	spring->setDamper(damper);
	spring->setSpringConstant(springConst);
	spring->setSpringLength(springLength);
	spring->addToObjectManager();
	return *spring;
}

#include "Visualization\MiniGL.h"
using namespace IBDS;
void Spring::render()
{
	Vector3d vec1;
	Vector3d vec2;

	getGlobalPositionA(vec1);
	getGlobalPositionB(vec2);

	MiniGL::drawVector(vec1, vec2, 2, MiniGL::darkGreen);
}