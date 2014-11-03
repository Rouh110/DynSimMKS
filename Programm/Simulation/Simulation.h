#pragma once
#include "Eigen/Dense"
#include "RigidBody.h"

using namespace Eigen;

class Simulation
{
public:
	enum ApproximationMethod {EXPLICIT_EULER, RUNGE_KUTTA_4};
protected:
	ApproximationMethod approximationMethod;
	unsigned int iterationCount;
public:
	Simulation();
	~Simulation();

	void setApproximationMethod(ApproximationMethod method);
	void update(Real h);

protected:
	void resetForces();
	void simulateExplicitEuler(Real h);
	void simulateRungeKutta4(Real h);

	inline void calculateXdot(const RigidBody *rigidBody, Vector3d & result) const;
	inline void calculateXdot(const Vector3d & velocity, Vector3d & result) const;
	inline void calculateVdot(const RigidBody * rigidBody, Real time, Vector3d & result) const;
	inline void calculateVdot(Real mass, const Vector3d & force, Vector3d & result) const;
	inline void calculateQdot(const RigidBody * rigidBody, Quaterniond &result) const;
	inline void calculateQdot(const Quaterniond &q, const Vector3d & angularVelocity, Quaterniond &result) const;
	inline void calculateWdot(const RigidBody * rigidBody, Real time, Vector3d & result) const;
	inline void calculateWdot(const Vector3d & angularVelocity, const Vector3d & inertiaTensor, const Vector3d & invertedInertiaTensor, const Vector3d & torque, Vector3d & result) const;

};

