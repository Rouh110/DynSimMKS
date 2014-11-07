#include "IForce.h"
#include "Simulation\SimulationManager.h"

IForce::IForce()
{
}


IForce::~IForce()
{
}


void IForce::addToObjectManager()
{
	SimulationManager::getInstance()->getObjectManager().addObject(this);
}

void IForce::render()
{

}
