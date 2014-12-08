#pragma once
#include "IScene.h"
#include "Sphere.h"
#include "BoundingVolume.h"
class SceneBoundingVolume :
	public IScene
{
private: 
	int timer = 0;
	BoundingVolume  *boundBouncingBall;
	Sphere *bouncingball;
public:

	SceneBoundingVolume();
	~SceneBoundingVolume();

	virtual void initializeScene();
	virtual void update(Real currentTime);
	void reset();
};

