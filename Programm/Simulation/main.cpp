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


#include "Simulation/Spring.h"
#include "Common/Config.h"
#include "Visualization/MiniGL.h"
#include "GL/glut.h"
#include "TimeManager.h"
#include "Common/StringTools.h"
#include "Common/timing.h"
#include <Eigen/Dense>

#include "Simulation/SimMath.h"
#include "Simulation/Cube.h"
#include "Simulation/Sphere.h"
#include "Simulation/SimulationManager.h"
#include "Simulation/BoundingVolume.h"
#include "Simulation/BoundingVolumeTreeNode.h"
#include "Simulation/BoundingVolumeTree.h"
#include "TestScene01.h"
#include "TestScene02.h"
#include "TestScene03.h"
#include "ImpulseTest.h"
#include "SceneMobile.h"
#include "ScenePuppet.h"
#include "SceneBoundingVolumeTwo.h"
#include <list>


// Enable memory leak detection
#ifdef _DEBUG
	#define new DEBUG_NEW 
#endif

using namespace IBDS;
using namespace Eigen;
using namespace std;

int currentSceneID = 0;
vector<IScene*> scenes;
bool drawVolumes = false;
int volumeDrawDepth = 0;
void timeStep ();
void buildModel ();
void render ();
void cleanup();
void addScenes();
void addScene(IScene* scene);
void resetSim();



void resetSim() {
	// Delete all created objects
	SimulationManager::getInstance()->getObjectManager().resetObjectManager();
	// Reset time
	TimeManager::getCurrent()->setTime(0.0);

	// Re-initialize scene 
	buildModel();
}

void TW_CALL setSceneCB(const void *value, void *clientData) {
	// Set the new scene and reset the simulation
	int integration = *(const int *)(value);
	currentSceneID = integration;
	resetSim();
}

void TW_CALL getSceneCB(void *value, void *clientData) {
	// Return the current scene
	*(int *)(value) = currentSceneID;
}

void TW_CALL setApproximationCB(const void *value, void *clientData) {
	int integration = *(const int *)(value);
	switch (integration) {
	case 0:
		SimulationManager::getInstance()->getSimulation().setApproximationMethod(Simulation::EXPLICIT_EULER );
		break;

	case 1:
		SimulationManager::getInstance()->getSimulation().setApproximationMethod(Simulation::RUNGE_KUTTA_4);
		break;

	default:
		break;
	}
}

void TW_CALL getApproximationCB(void *value, void *clientData) {
	Simulation::ApproximationMethod method = SimulationManager::getInstance()->getSimulation().getApproximationMethod();
	*(int *)(value) = int(method);
}

void TW_CALL setTimeStepCB(const void *value, void *clientData) {
	// Set the new time step value
	float timeStep = *(const float *)(value);
	TimeManager::getCurrent()->setTimeStepSize(timeStep);
}
void TW_CALL getTimeStepCB(void *value, void *clientData) {
	// Get the current time step value
	float timeStep = TimeManager::getCurrent()->getTimeStepSize();
	*(float *)(value) = float(timeStep);
}

void TW_CALL setTreeDepthCB(const void *value, void *clientData) {
	// Set the new time step value
	int _volumeDrawDepth = *(const int *)(value);
	volumeDrawDepth = _volumeDrawDepth;
}
void TW_CALL getTreeDepthCB(void *value, void *clientData) {
	// Get the current time step value
	int _volumeDrawDepth = volumeDrawDepth;
	*(int *)(value) = int(_volumeDrawDepth);
}

void TW_CALL setTreeDrawCB(const void *value, void *clientData) {
	// Set the new time step value
	bool _drawVolumes = *(const bool *)(value);
	drawVolumes = _drawVolumes;
}
void TW_CALL getTreeDrawCB(void *value, void *clientData) {
	// Get the current time step value
	bool _drawVolumes = drawVolumes;
	*(bool *)(value) = bool(_drawVolumes);
}

void TW_CALL resetSimBTN(void *clientData) {
	// Call rest sim function
	resetSim();
}

void TW_CALL resetCamBTN(void *clientData) {
	// Reset Viewport
	MiniGL::setViewport(40.0f, 1.0f, 100.0f, Vector3d(0.0, 1.0, 10.0), Vector3d(0.0, 0.0, 0.0));
}

void addScene(IScene* scene)
{
	scenes.push_back(scene);
}


// main 
int main( int argc, char **argv )
{
	REPORT_MEMORY_LEAKS
	USE_TIMESTEP_TIMING(Timing::m_dontPrintTimes = true;);

	// OpenGL
	MiniGL::init (argc, argv, 1024, 768, 0, 0, "Simulation");
	MiniGL::initLights ();
	MiniGL::setClientIdleFunc (50, timeStep);		

	//add the all existing Scenes into the scenes List
	addScenes();

	// set tweakBar

	// Scene Switch
	TwEnumVal *sceneEV = new TwEnumVal[scenes.size()];
	int i = 0;
	for each (IScene* scene in scenes)
	{
		sceneEV[i] = { i, scene->getName().data() };
		i++;
	}
	// Create a type for the enum sceneEV
	TwType sceneType = TwDefineEnum("SceneType", sceneEV, scenes.size());
	TwAddVarCB(MiniGL::m_tweakBar, "Scene", sceneType, setSceneCB, getSceneCB, NULL, "");
	// Approximation Switch
	TwEnumVal approximationEV[] = { { 0, "Explicit Euler" }, { 1, "Runge Kutta 4" } };
	// Create a type for the enum shapeEV
	TwType approximationType = TwDefineEnum("ApproximationType", approximationEV, 2);
	TwAddVarCB(MiniGL::m_tweakBar, "Approximation Method", approximationType, setApproximationCB, getApproximationCB, NULL, "");
	// Create value to increase/decrease timestepsize during runtime
	// min: 0.01 , step size: 0.01
	TwAddVarCB(MiniGL::m_tweakBar, "Time Step", TW_TYPE_FLOAT, setTimeStepCB, getTimeStepCB, NULL, "min=0.01 step=0.01");
	// Create button to reset camera
	TwAddButton(MiniGL::m_tweakBar, "Reset Camera", resetCamBTN, NULL, "");
	// Create reset button 
	TwAddButton(MiniGL::m_tweakBar, "Reset Simulation", resetSimBTN, NULL, "");

	// Set Tree Depth
	TwAddVarCB(MiniGL::m_tweakBar, "Tree Depth", TW_TYPE_INT32, setTreeDepthCB, getTreeDepthCB, NULL, "min=0 max=6 step=1");
	// Set Tree Drawn
	TwAddVarCB(MiniGL::m_tweakBar, "Tree Draw", TW_TYPE_BOOL32, setTreeDrawCB, getTreeDrawCB, NULL, "");

	buildModel ();	
	
	MiniGL::setClientSceneFunc(render);			
	MiniGL::setViewport (40.0f, 1.0f, 100.0f, Vector3d (0.0, 1.0, 10.0), Vector3d (0.0, 0.0, 0.0));

	glutMainLoop ();	

	cleanup ();
	delete[] sceneEV;
	USE_TIMESTEP_TIMING(printAverageTimes());
	
	return 0;
}

void cleanup()
{
	SimulationManager::getInstance()->getObjectManager().resetObjectManager();
	//delete all scenes
	for (std::vector<IScene*>::iterator it = scenes.begin(); it != scenes.end(); ++it)
	{
		delete (*it);
	}
	scenes.clear();

	delete SimulationManager::getInstance();
}

void timeStep ()
{
	START_TIMING("timeStep");
	TimeManager *tm = TimeManager::getCurrent ();
	const Real h = tm->getTimeStepSize();

	//update Simulation

	scenes[currentSceneID]->update(tm->getTime());
	
	SimulationManager::getInstance()->getSimulation().update(h);
	tm->setTime(tm->getTime() + h);

	STOP_TIMING_AVG;
}

void addScenes()
{
	addScene(new TestScene01());
	addScene(new TestScene02());
	addScene(new TestScene03());
	addScene(new ImpulseTest());
	addScene(new SceneMobile());
	addScene(new ScenePuppet());
	addScene(new SceneBoundingVolumeTwo());
}

void buildModel ()
{
	TimeManager::getCurrent ()->setTimeStepSize (0.01);

	//SimulationManager::getInstance()->getSimulation().setApproximationMethod(Simulation::RUNGE_KUTTA_4);
	// Create simulation model
	
	
	// Init the current scene
	scenes[currentSceneID]->initializeScene();

}

double i = 0;
void render ()
{
	MiniGL::coordinateSystem();
	
	// Draw simulation model

	Vector3d pos;	
	Cube *c;
	Sphere *s;
	for each (BoundingVolume*  bv in SimulationManager::getInstance()->getSimulation().getCollidedBoundingVolumes()){
		MiniGL::drawVector(bv->contactPoint, bv->contactNormal, 2, MiniGL::black);
	}
	for each (RigidBody* rigidBody in SimulationManager::getInstance()->getObjectManager().getRigidBodies())
	{
		c = dynamic_cast<Cube*>(rigidBody);
		//c = (Cube*)(rigidBody);
		if (c != 0)
		{
			pos = c->getPosition();
			//c->setRotation(q);
			//c->setPosition(Vector3d(i,0,0));
			MiniGL::drawCube(&pos, &(c->getRotation().inverse().toRotationMatrix()), c->getWidth(), c->getHeight(), c->getDepth(), MiniGL::cyan);
		}
		else
		{
			s = dynamic_cast<Sphere*>(rigidBody);
			
			if (s != 0)
			{
				pos = s->getPosition();
				BoundingVolume * boundSphere = s->getVolumeTree()->getRoot()->getBoundingVolume();
				MiniGL::drawVector(boundSphere->contactPoint, boundSphere->contactPoint, 2,MiniGL::black);
				MiniGL::drawSphere(&pos, s->getRadius(), MiniGL::red);
			}

		}
	}
	
	for each (IForce  *  force in SimulationManager::getInstance()->getObjectManager().getForces())
	{
		force->render();
	}


	for each (IJoint  *  joint in SimulationManager::getInstance()->getObjectManager().getJoints())
	{
		joint->render();
	}

	if (drawVolumes)
	{
		int currentDepth = 0;
		list<int> state;
		const BoundingVolumeTreeNode* node;
		BoundingVolume* volume;
		for each (RigidBody* rigidBody in SimulationManager::getInstance()->getObjectManager().getRigidBodies())
		{
			state.push_back(0);
			node = rigidBody->getVolumeTree()->getRoot();
			currentDepth = 0;
			while (state.size() > 0)
			{
				if (currentDepth == volumeDrawDepth || node->isLeave())
				{
					//Draw Volume
					volume = node->getBoundingVolume();
					MiniGL::drawSphere(&(rigidBody->toGlobalSpace(volume->m)),volume->r,MiniGL::gray);
					//clime up the volume tree
					node = node->getParent();
					state.pop_back();
					currentDepth--;
					if (state.size() > 0)
						state.back() = state.back() + 1;
					
				}
				else if (state.back() >= node->numberOfChildren())
				{
					node = node->getParent();
					state.pop_back();
					currentDepth--;
					if (state.size() > 0)
						state.back() = state.back() + 1;
					
				}
				else
				{
					node = node->getChild(state.back());
					state.push_back(0);
					currentDepth++;
				}

			}


		}
	}

	MiniGL::drawTime( TimeManager::getCurrent ()->getTime ());

}


