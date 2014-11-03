#pragma once
#include "ObjectManager.h";
#include "Simulation.h"
class SimulationManager
{
private:
	static SimulationManager * m_instance;
	ObjectManager objManager;
	Simulation simulation;
public:
	static SimulationManager * getInstance();

	ObjectManager & getObjectManager();
	Simulation & getSimulation();

	~SimulationManager();
private:
	SimulationManager();
	SimulationManager(SimulationManager&);
	
};

