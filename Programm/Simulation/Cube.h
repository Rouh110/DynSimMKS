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
public:

	/*Creates a Cuboit with the given Prameter and adds it to the ObjectManager.
	Returns a Referenze to the created Cuboit.*/
	static Cube & Cube::create(Real width = 1, Real height = 1, Real depth = 1);
};

