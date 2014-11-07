#pragma once
#include "IForce.h"
#include "Simulation\RigidBody.h"

class GlobalForce :
	public IForce
{
public:
	GlobalForce();
	~GlobalForce();

	virtual void computeForce(Real time);

protected:
	virtual void computeForce(RigidBody *rigidBody, Real time) = 0;
};

