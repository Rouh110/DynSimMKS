#include "SceneStressTest02.h"
#include "Cube.h"
#include "Gravity.h"
#include "Spring.h"
#include "Sphere.h"
SceneStressTest02::SceneStressTest02()
{
	this->name = "SceneStressTest02";
}


SceneStressTest02::~SceneStressTest02()
{
}

void SceneStressTest02::initializeScene()
{

	Gravity::create();

	Sphere* list[625];

	for (int i = 0; i < 25; ++i)
	{
		for (int j = 0; j < 25; ++j)
		{
			list[i] = &Sphere::create(0.5);
			float x, y, z;
			x = -12.5f + i;
			y = 0;
			z = -12.5f + j;
			list[i]->setPosition(Vector3d(x, y, z));
		}
	}

	Sphere* list2[200];

	for (int i = 0; i < 200; ++i)
	{
		list2[i] = &Sphere::create(0.5);
		float x, y, z;
		x = rand() % 25;
		x = x - 12.5f + (float)(rand() % 100) / 100.0f;
		y = (rand() % 10) + 5;
		z = rand() % 25;
		z = z - 12.5f + (float)(rand() % 100) / 100.0f;
		list2[i]->setPosition(Vector3d(x, y, z));
		list2[i]->addForce(Vector3d(x, y, z));
	}
}
