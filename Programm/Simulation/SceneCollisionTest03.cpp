#include "SceneCollisionTest03.h"
#include "SimulationManager.h"

SceneCollisionTest03::SceneCollisionTest03()
{
	name = "Collision Gravity";
}


SceneCollisionTest03::~SceneCollisionTest03()
{
}

void SceneCollisionTest03::initializeScene()
{
	SimulationManager::getInstance()->getSimulation().setContactConstant(0.01);
	SimulationManager::getInstance()->getSimulation().setCollisionCheck(true);
	SimulationManager::getInstance()->getSimulation().setcheckYCollition(true);

	Gravity::create();

	Real x1 = 0;
	Real x2 = 3;
	Real y1 = 0.5;
	Real y2 = 3;

	Cube &cube01 = Cube::create(1,1,1);
	cube01.setPosition(Vector3d(x1, y1+0.2, 0));
	
	Cube &cube02 = Cube::create(3,0.2,3);
	cube02.setPosition(Vector3d(x1, y2, 0));
	Cube &cube03 = Cube::create(1,0.2,1);
	cube03.setPosition(Vector3d(x1, (y2+1.5), 0));
	Cube &cube04 = Cube::create(1,0.2,2.5);
	cube04.setPosition(Vector3d(x1, (y2+3), 0));
	
	
	Cube &cube05 = Cube::create(2,0.4,2);
	cube05.setPosition(Vector3d(x1, (y2+4.5), 0));
	Cube &cube06 = Cube::create(2,0.2,1.5);
	cube06.setPosition(Vector3d(x1, (y2+6), 0));
	
	Cube &cube07 = Cube::create();
	cube07.setPosition(Vector3d(x2, y1, 0));

	Sphere &sphere01 = Sphere::create(0.5);
	sphere01.setPosition(Vector3d(x2+1.5,y1,0));

}

void SceneCollisionTest03::update(Real currentTime)
{

}
