#include "GlobalForce.h"
#include "SimulationManager.h"


GlobalForce::GlobalForce()
{
}


GlobalForce::~GlobalForce()
{
}

void GlobalForce::computeForce(Real time)
{
	for each (RigidBody * rigidBody in SimulationManager::getInstance()->getObjectManager().getRigidBodies())
	{
		if (!rigidBody->isStatic())
			computeForce(rigidBody, time);
	}
}
