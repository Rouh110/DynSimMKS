#pragma once
#include "IScene.h"
#include "Cube.h"
#include "Sphere.h"
using namespace Eigen;
class SceneCollisionTest02 : public IScene
{
protected:
	Cube * cube01;
	Cube * cube02;
	Cube * cube03;
	Cube * cube04;
	Cube * cube05;

	Sphere * sphere01;
	Sphere * sphere02;
	Sphere * sphere03;
	Sphere * sphere04;

	int timer;
	bool next;
	
public:
	SceneCollisionTest02();
	~SceneCollisionTest02();

	void initializeScene();
	void update(Real currentTime);
};

