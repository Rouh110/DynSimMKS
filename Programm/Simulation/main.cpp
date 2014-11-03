/*
 * IBDS - Impulse-Based Dynamic Simulation Library
 * Copyright (c) 2003-2008 Jan Bender http://www.impulse-based.de
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * Jan Bender - Jan.Bender@impulse-based.de
 */


#include "Common/Config.h"
#include "Visualization/MiniGL.h"
#include "GL/glut.h"
#include "TimeManager.h"
#include "Common/StringTools.h"
#include "Common/timing.h"
#include <Eigen/Dense>

// Enable memory leak detection
#ifdef _DEBUG
	#define new DEBUG_NEW 
#endif

using namespace IBDS;
using namespace Eigen;
using namespace std;

void timeStep ();
void buildModel ();
void render ();
void cleanup();

#include "Simulation/SimMath.h"
#include "Simulation/Cube.h";
#include "Simulation/SimulationManager.h";
#include "Simulation/TestClass.h"

// main 
int main( int argc, char **argv )
{
	REPORT_MEMORY_LEAKS
	USE_TIMESTEP_TIMING(Timing::m_dontPrintTimes = true;);

	// OpenGL
	MiniGL::init (argc, argv, 1024, 768, 0, 0, "Simulation");
	MiniGL::initLights ();
	MiniGL::setClientIdleFunc (50, timeStep);		

	buildModel ();

	MiniGL::setClientSceneFunc(render);			
	MiniGL::setViewport (40.0f, 1.0f, 100.0f, Vector3d (0.0, 1.0, 10.0), Vector3d (0.0, 0.0, 0.0));

	glutMainLoop ();	

	cleanup ();

	USE_TIMESTEP_TIMING(printAverageTimes());
	
	return 0;
}

void cleanup()
{
	delete SimulationManager::getInstance();
}

void timeStep ()
{
	START_TIMING("timeStep");
	TimeManager *tm = TimeManager::getCurrent ();
	const Real h = tm->getTimeStepSize();

	// Simulation code

	//add globalForces

	for each (RigidBody* rigidBody in SimulationManager::getInstance()->getObjectManager().getRigidBodies())
	{
		Vector3d force(0.1,0,0);
		rigidBody->addForce(force, Vector3d(0,1,1));
		rigidBody->addForce(force*-1);
	}

	//update Simulation
	SimulationManager::getInstance()->getSimulation().update(h);
	tm->setTime(tm->getTime() + h);

	STOP_TIMING_AVG;
}

void buildModel ()
{
	TimeManager::getCurrent ()->setTimeStepSize (0.01);
	// Create simulation model
	Cube * cube = new Cube(1.0, 5.0, 1.0);
	//cube->setRotation(Quaterniond(1,1,0.5,0).normalized());
	SimulationManager::getInstance()->getObjectManager().addObject(cube);
	//RigidBody * r = new RigidBody();	
	//TestClass *p = new TestClass();
	//TestClass test;
	//delete test;

	
}

double i = 0;
void render ()
{
	MiniGL::coordinateSystem();
	
	// Draw simulation model

	Vector3d pos;	
	Cube *c;

	for each (RigidBody* rigidBody in SimulationManager::getInstance()->getObjectManager().getRigidBodies())
	{
		c = (Cube*)rigidBody;
		pos = c->getPosition();
		//c->setRotation(q);
		//c->setPosition(Vector3d(i,0,0));
		MiniGL::drawCube(&pos, &(c->getRotation().toRotationMatrix()), c->getWidth(), c->getHeight(), c->getDepth(), MiniGL::cyan);
	}

	MiniGL::drawTime( TimeManager::getCurrent ()->getTime ());

}

