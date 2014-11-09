#pragma once
#include "IForce.h"
#include "Simulation\RigidBody.h"

/*
Interface for all Forces that effects all rigidbodys in the scene
*/
class GlobalForce :
	public IForce
{
public:
	GlobalForce();
	~GlobalForce();

	/*
	Calculates the force at the given time and add it to all the Riggidbody in the Scene.
	*/
	virtual void computeForce(Real time);

protected:
	/*
	Calculates the force for the given rigidbody at the given time.
	This function will be called for every rigidbody in the scene every time when the force will be computed except for static rigidbodys.
	*/
	virtual void computeForce(RigidBody *rigidBody, Real time) = 0;
};

