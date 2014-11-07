#include "ObjectManager.h"

ObjectManager::ObjectManager()
{
}


ObjectManager::~ObjectManager()
{
	printf("deleted\n");
	resetObjectManager();
}

void ObjectManager::addObject(RigidBody *rigidBody)
{
	rigidBodies.push_back(rigidBody);
}

void ObjectManager::addObject(IForce *force)
{
	forces.push_back(force);
}

const std::vector<RigidBody*> & ObjectManager::getRigidBodies() const
{
	return rigidBodies;
}

const std::vector<IForce*> & ObjectManager::getForces() const
{
	return forces;
}

void ObjectManager::resetObjectManager()
{
	//delete all Forces
	for (std::vector<IForce*>::iterator it = forces.begin(); it != forces.end(); ++it)
	{
		delete (*it);
	}
	forces.clear();

	//delete all RigidBodys
	for (std::vector<RigidBody*>::iterator it = rigidBodies.begin(); it != rigidBodies.end(); ++it)
	{
		delete (*it);
	}
	rigidBodies.clear();
}
