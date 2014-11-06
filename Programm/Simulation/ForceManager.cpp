#include "ForceManager.h"


ForceManager::ForceManager()
{

}


ForceManager::~ForceManager()
{
	printf("deleted\n");
	
	for (std::vector<RigidBody*>::iterator it = forcedBodies.begin(); it != forcedBodies.end(); ++it)
	{
		delete (*it);
	}
	forcedBodies.clear();
}
const std::vector<RigidBody*> & ForceManager::getForceBodies()
{
	return forcedBodies;
}
void ForceManager::addObject(RigidBody *forcedBody){
	forcedBodies.push_back(forcedBody);
}