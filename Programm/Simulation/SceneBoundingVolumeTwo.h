#pragma once
#include "Simulation\IScene.h"
#include "Simulation\Sphere.h"
#include "Simulation\BoundingVolume.h"
class SceneBoundingVolumeTwo :
	public IScene
{
private:
	int timer = 0;
	BoundingVolume  *boundBouncingBall;
	Sphere *bouncingball;
	Sphere *bouncingballtwo;
	Sphere *bouncingballthree;
	Sphere *bouncingballfour;
	Sphere *bouncingballfive;

	Sphere *bouncingballsix;
	Sphere *bouncingballseven;
	Sphere *bouncingballeight;
	Sphere *bouncingballnine;
	Sphere *bouncingballten;


public:
	SceneBoundingVolumeTwo();
	~SceneBoundingVolumeTwo();
	virtual void initializeScene();
	virtual void update(Real currentTime);
};

