#pragma once
#include "IScene.h"
#include "Cube.h"
class ImpulseTest :
	public IScene
{
private:
	int timer;
	Cube *cube01;
	Cube *cube02;
	Cube *cube03;
	Cube *cube04;
	Cube *cube05;

	bool next;
public:
	ImpulseTest();
	~ImpulseTest();

	/*Initializes the scene. That means adding all the Object to the Simulator and setting up the setting at the begining.*/
	virtual void initializeScene();

	/*Updates the scene*/
	virtual void update(Real currentTime);

	void reset();
};

