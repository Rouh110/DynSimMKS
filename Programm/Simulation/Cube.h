#pragma once

#include "RigidBody.h"

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
	void calculateTensor();
};

