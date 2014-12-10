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
	cube01.setPosition(Eigen::Vector3d(0, 3, 0));
	cube02.setPosition(Eigen::Vector3d(1,3,0));

	Cube &cube03 = Cube::create();
	cube03.setPosition(Eigen::Vector3d(0, 0.3, 0));
}