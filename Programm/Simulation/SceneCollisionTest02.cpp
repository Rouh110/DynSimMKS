#include "SceneCollisionTest02.h"
#include "SimulationManager.h"

SceneCollisionTest02::SceneCollisionTest02()
{
	name = "collision test 02";
}


SceneCollisionTest02::~SceneCollisionTest02()
{
}


void SceneCollisionTest02::initializeScene()
{
	SimulationManager::getInstance()->getSimulation().setContactConstant(0);
	SimulationManager::getInstance()->getSimulation().setCollisionCheck(true);
	SimulationManager::getInstance()->getSimulation().setcheckYCollition(true);

	timer = 0;
	next = false;

	Real y = 3;
	Real y2 = 5;
	Real x1 = -2.8;
	Real x2 = -0.67;
	Real x3 = 1.16;
	Real x4 = 3;

	// base line
	//cube01 = &Cube::create();
	//cube01->setPosition(Vector3d(x1,y,0));
	//cube02 = &Cube::create();
	//cube02->setPosition(Vector3d(x2,y,0));
	sphere01 = &Sphere::create(0.5);
	sphere01->setPosition(Vector3d(x3, y, 0));
	sphere04 = &Sphere::create(0.5);
	sphere04->setPosition(Vector3d(x2, y, 0));
	
	Sphere &sphere05 = Sphere::create(0.5);
	sphere05.setPosition(Vector3d(x2, y - 1, 0));
	Sphere &sphere06 = Sphere::create(0.5);
	sphere06.setPosition(Vector3d(x2, y - 2, 0));

	//cube04 = &Cube::create();
	//cube04->setPosition(Vector3d(x4, y, 0));
	//cube05 = &Cube::create();
	//cube05->setPosition(Vector3d(x4, y2, 0));
	// fall line
	//cube03 = &Cube::create();
	//cube03->setPosition(Vector3d(x1, y2, 0));

	sphere02 = &Sphere::create(0.5);
	sphere02->setPosition(Vector3d(x2, y2, 0));
	sphere07 = &Sphere::create(0.5);
	sphere07->setPosition(Vector3d(x2, y2+1, 0));
	sphere03 = &Sphere::create(0.5);
	sphere03->setPosition(Vector3d(x3, y2, 0));
	
	cube01 = &Cube::create();
	cube01->setPosition(Vector3d(x1,y,0));
	cube02 = &Cube::create();
	cube02->setPosition(Vector3d(x1,y2,0));

	cube03 = &Cube::create(10,0.5,10);
	cube03->setMass(0);
	cube03->setPosition(Vector3d(-2, -5, -2));

}

void SceneCollisionTest02::update(Real currentTime)
{
	int timeFactor = 5;
	//Real impulseStrength = 0.5;

	// set Timer
	if (timer < (int)(currentTime / timeFactor))
	{
		timer++;
		next = true;
	}

	if (next)
	{
		switch (timer)
		{
		case 1:
			cube02->addRasImpuls(Vector3d(0, -0.5, 0.2), Vector3d(0.5, 0.5, 0.5));
			break;
		case 2:
			sphere02->addRasImpuls(Vector3d(0, -0.1, 0), Vector3d(0, 0, 0));
			sphere07->addRasImpuls(Vector3d(0, -0.1, 0), Vector3d(0, 0, 0));
			break;
		case 3:
			sphere03->addRasImpuls(Vector3d(0, -3, 0), Vector3d(0, 0, 0));
			break;
		case 4:
			//cube05->addRasImpuls(Vector3d(0, -impulseStrength, 0), Vector3d(1, 0, 1));
			break;
		case 10:
			// reset();
			break;
		}

		next = false;
	}
}