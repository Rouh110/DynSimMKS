#pragma once
#include "ObjectManager.h";
#include "Simulation.h"

/*
Singelton.
Used to give access to several classes for the simulation.
*/
class SimulationManager
{
private:
	static SimulationManager * m_instance;
	ObjectManager objManager;
	Simulation simulation;
public:
	/*
	Returns the Instance of the Simulation Manager
	*/
	static SimulationManager * getInstance();

	ObjectManager & getObjectManager();
	Simulation & getSimulation();

	~SimulationManager();
private:
	SimulationManager();
	SimulationManager(SimulationManager&);
	
};

