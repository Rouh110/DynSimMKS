#include "SceneCollisionTest01.h"


SceneCollisionTest01::SceneCollisionTest01()
{
	name = "collision test 01";
}


SceneCollisionTest01::~SceneCollisionTest01()
{
}


void SceneCollisionTest01::initializeScene()
{
	timer = 0;
	next = false;

	Real y = 1;
	Real y2 = 3;
	Real x1 = -2.8;
	Real x2 = -0.67;
	Real x3 = 1.16;
	Real x4 = 3;

	// base line
	cube01 = &Cube::create();
	cube01->setPosition(Vector3d(x1,y,0));
	cube02 = &Cube::create();
	cube02->setPosition(Vector3d(x2,y,0));
	sphere01 = &Sphere::create(0.5);
	sphere01->setPosition(Vector3d(x3, y, 0));


	cube04 = &Cube::create();
	cube04->setPosition(Vector3d(x4, y, 0));
	cube05 = &Cube::create();
	cube05->setPosition(Vector3d(x4, y2, 0));
	// fall line
	cube03 = &Cube::create();
	cube03->setPosition(Vector3d(x1, y2, 0));
	sphere02 = &Sphere::create(0.5);
	sphere02->setPosition(Vector3d(x2, y2, 0));
	sphere03 = &Sphere::create(0.5);
	sphere03->setPosition(Vector3d(x3, y2, 0));
	

}

void SceneCollisionTest01::update(Real currentTime)
{
	int timeFactor = 5;
	Real impulseStrength = 0.1;

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
			cube03->addRasImpuls(Vector3d(0, -impulseStrength, 0), Vector3d(0, 0, 0));
			break;
		case 2:
			sphere02->addRasImpuls(Vector3d(0, -impulseStrength, 0), Vector3d(0, 0, 0));
			break;
		case 3:
			sphere03->addRasImpuls(Vector3d(0, -impulseStrength, 0), Vector3d(0, 0, 0));
			break;
		case 4:
			cube05->addRasImpuls(Vector3d(0, -impulseStrength, 0), Vector3d(1, 0, 1));
			break;
		case 10:
			// reset();
			break;
		}

		next = false;
	}
}