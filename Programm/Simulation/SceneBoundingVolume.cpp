#include "SceneBoundingVolume.h"
#include "Sphere.h"
#include "Gravity.h"
#include "Simulation/Spring.h"
#include "Common/Config.h"
#include "Visualization/MiniGL.h"
#include "GL/glut.h"
#include "TimeManager.h"
#include "Common/StringTools.h"
#include "Common/timing.h"

SceneBoundingVolume::SceneBoundingVolume()
{
}


SceneBoundingVolume::~SceneBoundingVolume()
{
}
void SceneBoundingVolume::initializeScene(){
	timer = 0;
	Gravity::create();
	bouncingball = &Sphere::create(0.4);
	bouncingball->setPosition(Eigen::Vector3d(0, 3, 0));
	bouncingball->setMass(1);
	boundBouncingBall = new BoundingVolume(Eigen::Vector3d(0,3,0), 0.4);	
}
void SceneBoundingVolume::update(Real currentTime)
{
	int timeFactor = 5;
	Real impulseStrength = 0.1;
	
	// set Timer
	if (timer < (int)(currentTime / timeFactor))
	{
		timer++;
	}
	boundBouncingBall->m = bouncingball->getPosition();
	if (boundBouncingBall->collisionTestYAxis()){
		Eigen::Vector3d collisionPoint = Eigen::Vector3d(0, 0, 0);
		std::cout << "ConteactNormal axis " << bouncingball->contactNormal << " \n" << std::flush;
		boundBouncingBall->collisionCalcYAxis(bouncingball->contactNormal, collisionPoint);
	}
}