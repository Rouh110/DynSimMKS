#pragma once
#include "Common\Config.h"

/*Represents a Scene in the Simulator*/
class IScene
{
public:
	IScene();
	~IScene();

	/*Initializes the scene. That means adding all the Object to the Simulator and setting up the setting at the begining.*/
	virtual void initializeScene() = 0;

	/*Updates the scene*/
	virtual void update(Real currentTime){};
};

