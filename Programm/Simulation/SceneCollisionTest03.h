#pragma once

#include "IScene.h"
#include "IScene.h"
#include "Cube.h"
#include "Sphere.h"
#include "Gravity.h"

class SceneCollisionTest03 : public IScene
{
public:
	SceneCollisionTest03();
	~SceneCollisionTest03();

	void initializeScene();
	void update(Real currentTime);
};

