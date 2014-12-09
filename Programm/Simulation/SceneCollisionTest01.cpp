#include "SceneCollisionTest01.h"

#include "Cube.h"

SceneCollisionTest01::SceneCollisionTest01()
{
	name = "collision test 01";
}


SceneCollisionTest01::~SceneCollisionTest01()
{
}


void SceneCollisionTest01::initializeScene()
{
	Cube &cube01 = Cube::create();
	Cube &cube02 = Cube::create();
	cube02.setPosition(Eigen::Vector3d(1,0,0));
}