#pragma once
#include "IScene.h"
class ImpulseTest :
	public IScene
{
public:
	ImpulseTest();
	~ImpulseTest();

	/*Initializes the scene. That means adding all the Object to the Simulator and setting up the setting at the begining.*/
	virtual void initializeScene();

	/*Updates the scene*/
	virtual void update(Real currentTime);
};

