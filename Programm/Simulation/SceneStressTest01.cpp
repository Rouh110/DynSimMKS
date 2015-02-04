#include "SceneStressTest01.h"
#include "Cube.h"
#include "Gravity.h"
#include "Spring.h"
#include "Sphere.h"
SceneStressTest01::SceneStressTest01()
{
	this->name = "SceneStressTest01";
}


SceneStressTest01::~SceneStressTest01()
{
}

void SceneStressTest01::initializeScene()
{

	Gravity::create();

	Sphere* list[200];

	for (int i = 0; i < 200; ++i)
	{
		list[i] = &Sphere::create(0.5);
		float x, y, z;
		x = rand() % 5;
		y = (rand() % 10) + 10;
		z = rand() % 5;
		list[i]->setPosition(Vector3d(x, y, z));
		list[i]->addForce(Vector3d(x, y, z));
	}
}
