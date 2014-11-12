#pragma once
#include "IJoint.h"

/*
Represents a Spherical Joint.
*/
class SphericalJoint :
	public IJoint
{
public:
	SphericalJoint();
	~SphericalJoint();

	virtual void getCurrentError(Vector3d & out_Error);

	/*
	Renders the Spherical Joint
	*/
	virtual void render();

	static SphericalJoint & create();
	static SphericalJoint & create(RigidBody * rigidBodyA, RigidBody *rigidBodyB, const Vector3d & jointPoint);
};

