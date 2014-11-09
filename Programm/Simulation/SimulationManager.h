#pragma once
#include "ObjectManager.h"
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

	/*Returns the ObjectManger */
	ObjectManager & getObjectManager();

	/*Returns the Simulator*/
	Simulation & getSimulation();

	~SimulationManager();
private:
	/*private constructor to prevent uncontrolled instanciation of the singelton*/
	SimulationManager();
	/*Empty copy constructor to prevent the duplication of the singelton*/
	SimulationManager(SimulationManager&);
	
};

