#pragma once

#include "Common\Config.h"

/*
Interface for all forces.
*/
class IForce
{
public:
	IForce();
	~IForce();

	/*
	Calculates the Force at the given time and adds it to all the RigidBody it effects.
	*/
	virtual void computeForce(Real time) = 0;

	/*Will be called by the render loop.
	Override this if you want to draw the Force*/
	virtual void render();

protected:
	/*adds the force to the ObjectManager.*/
	void addToObjectManager();
};

