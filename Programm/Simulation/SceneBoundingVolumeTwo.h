#pragma once
#include "Simulation\IScene.h"
#include "Simulation\Sphere.h"
#include "Simulation\BoundingVolume.h"
class SceneBoundingVolumeTwo :
	public IScene
{
private:
	int timer = 0;



public:
	SceneBoundingVolumeTwo();
	~SceneBoundingVolumeTwo();
	virtual void initializeScene();
	virtual void update(Real currentTime);
};

