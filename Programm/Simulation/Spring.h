#pragma once

#include "Simulation/RigidBody.h"
using namespace Eigen;

class Spring
{
private:
	RigidBody *rigidBodyA;
	Vector3d suspensionPointA;
	RigidBody *rigidBodyB;
	Vector3d suspensionPointB;

	Real damper;
	Real springConst;
	Real springLength;




public:
	Spring();
	~Spring();

	void setDamper(Real damper);
	void setSpringConstant(Real springConst);
	void setSpringLength(Real length);

	void setSuspensionPointA(const Vector3d & localPosition, RigidBody * rigidBody);
	void setSuspensionPointB(const Vector3d & localPosition, RigidBody * rigidBody);
	void addForces(Real time);

	void calculateForces(const Vector3d & globalPositionA, const Vector3d & velocityA, const Vector3d & globalPositionB, const Vector3d & velocityB, Vector3d out_forceA, Vector3d out_forceB) const;

protected:
	void getGlobalPositionA(Vector3d & out_position) const;
	void getGlobalPositionB(Vector3d & out_position) const;
};

