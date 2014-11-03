#include "SimulationManager.h"

SimulationManager * SimulationManager::m_instance = 0;
SimulationManager::SimulationManager()
{
}


SimulationManager::~SimulationManager()
{
}

SimulationManager * SimulationManager::getInstance()
{
	if (0 == m_instance)
	{
		m_instance = new SimulationManager();
	}
	return m_instance;
}

ObjectManager & SimulationManager::getObjectManager()
{
	return objManager;
}

Simulation & SimulationManager::getSimulation()
{
	return simulation;
}
