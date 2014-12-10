#include "SceneCollisionTest01.h"

#include "Cube.h"
#include "Sphere.h"

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
	cube02.setPosition(Eigen::Vector3d(1+(0.3),0,0));

	Cube &cube03 = Cube::create();
	cube03.setPosition(Eigen::Vector3d(0, 2, 0));
	Sphere &sphere01 = Sphere::create(0.5);
	sphere01.setPosition(Eigen::Vector3d(0.9, 2, 0));

}