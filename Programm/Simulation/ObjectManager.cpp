#include "ObjectManager.h"

ObjectManager::ObjectManager()
{
}


ObjectManager::~ObjectManager()
{
	printf("deleted\n");
	for (std::vector<RigidBody*>::iterator it = rigidBodies.begin(); it != rigidBodies.end(); ++it)
	{
		delete (*it);
	}
	rigidBodies.clear();
}

void ObjectManager::addObject(RigidBody *rigidBody)
{
	rigidBodies.push_back(rigidBody);
}

const std::vector<RigidBody*> & ObjectManager::getRigidBodies()
{
	return rigidBodies;
}
