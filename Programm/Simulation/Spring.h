#pragma once

#include "Simulation/RigidBody.h"
#include "Simulation\IForce.h"

using namespace Eigen;

class Spring :  public IForce
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

	void setSuspensionPoints(const Vector3d & localPositionA, RigidBody * rigidBodyA, const Vector3d & localPositionB, RigidBody * rigidBodyB, bool setLength = true);

	void computeForce(Real time);

	void calculateForces(const Vector3d & globalPositionA, const Vector3d & velocityA, const Vector3d & globalPositionB, const Vector3d & velocityB, Vector3d & out_forceA, Vector3d & out_forceB) const;

	static Spring & create();
	static Spring & create(Real damper, Real springConst, Real springLength);

	void render();
protected:
	void getGlobalPositionA(Vector3d & out_position) const;
	void getGlobalPositionB(Vector3d & out_position) const;
};

