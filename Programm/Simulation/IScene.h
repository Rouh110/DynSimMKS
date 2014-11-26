#pragma once
#include "Common\Config.h"
#include <string>
using namespace std;
/*Represents a Scene in the Simulator*/
class IScene
{
protected:
	string name = "DefaultScene";
public:
	IScene();
	~IScene();

	/*Initializes the scene. That means adding all the Object to the Simulator and setting up the setting at the begining.*/
	virtual void initializeScene() = 0;

	/*Updates the scene*/
	virtual void update(Real currentTime){};

	/* Returns the name of the scene.*/
	virtual const string & getName(){ return name; };
};

