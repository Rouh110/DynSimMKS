#pragma once

#include "RigidBody.h"
/*
Represents a Cube as an RigidBody.
*/
class Cube : public RigidBody
{
protected:
	Real width;
	Real height;
	Real depth;

public:
	Cube(Real width, Real height, Real depth);
	~Cube();

	Real getWidth();
	Real getHeight();
	Real getDepth();

protected:
	/*
	Calculates and sets the inertia tensor and inverted inertia Tesor form the current with height and depth
	*/
	void calculateTensor();
};

