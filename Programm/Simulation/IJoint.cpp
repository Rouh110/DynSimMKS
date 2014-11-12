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

void IJoint::getGlobalPointA(Vector3d & out_pointA)
{
	out_pointA = rigidBodyA->getRotation()._transformVector(pointA) + rigidBodyA->getPosition();
}

void IJoint::getGlobalPointB(Vector3d & out_pointB)
{
	out_pointB = rigidBodyB->getRotation()._transformVector(pointB) + rigidBodyB->getPosition();
}

void IJoint::addToObjectManager()
{
	SimulationManager::getInstance()->getObjectManager().addObject(this);
}

