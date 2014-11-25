#include "IJoint.h"
#include "SimulationManager.h"

IJoint::IJoint()
{
}


IJoint::~IJoint()
{
}


void IJoint::setJointPoints(RigidBody * rigidBodyA, RigidBody *rigidBodyB, const Vector3d & jointPoint)
{
	this->rigidBodyA = rigidBodyA;
	this->rigidBodyB = rigidBodyB;

	pointA = rigidBodyA->getRotation().inverse()._transformVector(jointPoint - rigidBodyA->getPosition());
	pointB = rigidBodyB->getRotation().inverse()._transformVector(jointPoint - rigidBodyB->getPosition());
}

void IJoint::getRas(Vector3d & out_rsa)
{
	out_rsa = rigidBodyA->getRotation()._transformVector(pointA);
}


void IJoint::getRbs(Vector3d & out_rsb)
{
	out_rsb = rigidBodyB->getRotation()._transformVector(pointB);
}

void IJoint::getLocalPointA(Vector3d & out_point)
{
	out_point = pointA;
}
void IJoint::getLocalPointB(Vector3d & out_point)
{
	out_point = pointB;
}

void IJoint::getGlobalPointA(Vector3d & out_pointA)
{
	if (rigidBodyA != 0 )
	{
		out_pointA = rigidBodyA->getRotation()._transformVector(pointA) + rigidBodyA->getPosition();
	}
	else
	{
		out_pointA = pointA;
	}
}

void IJoint::getGlobalPointB(Vector3d & out_pointB)
{
	if (rigidBodyB != 0)
	{
		out_pointB = rigidBodyB->getRotation()._transformVector(pointB) + rigidBodyB->getPosition();
	}
	else
	{
		out_pointB = pointB;
	}
	
}

RigidBody * IJoint::getRigidBodyA()
{
	return rigidBodyA;
}

RigidBody * IJoint::getRigidBodyB()
{
	return rigidBodyB;
}


void IJoint::addToObjectManager()
{
	SimulationManager::getInstance()->getObjectManager().addObject(this);
}




