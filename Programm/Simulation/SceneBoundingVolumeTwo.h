#pragma once
#include "C:\Users\mcburn_laptop\git\DynSimMKS\Programm\Simulation\IScene.h"
#include "C:\Users\mcburn_laptop\git\DynSimMKS\Programm\Simulation\Sphere.h"
#include "C:\Users\mcburn_laptop\git\DynSimMKS\Programm\Simulation\BoundingVolume.h"
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

