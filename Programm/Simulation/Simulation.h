#pragma once
#include "Eigen/Dense"
#include "RigidBody.h"

using namespace Eigen;

/*
Simulates all non-static RigidBodys
*/
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

	/*
	Sets the Method that will be used for calculation the next step.
	*/
	void setApproximationMethod(ApproximationMethod method);
	ApproximationMethod getApproximationMethod();

	/*Updates the Simualtion*/
	void update(Real h);

protected:
	/*
	resets the Forces for all RigidBodys
	*/
	void resetForces();

	/*
	Compute the simulation with explicit Euler 
	*/
	void simulateExplicitEuler(Real h);

	/*
	Compute the simulation with Runge Kutta
	*/
	void simulateRungeKutta4(Real h);

	/*
	Calculate the derivation of the position.
	*/
	inline void calculateXdot(const RigidBody *rigidBody, Vector3d & result) const;
	inline void calculateXdot(const Vector3d & velocity, Vector3d & result) const;

	/*
	Calculate the derivation of the velocity.
	*/
	inline void calculateVdot(const RigidBody * rigidBody, Real time, Vector3d & result) const;
	inline void calculateVdot(Real mass, const Vector3d & force, Vector3d & result) const;

	/*
	Calculate the derivation of the Rotiaiton.
	*/
	inline void calculateQdot(const RigidBody * rigidBody, Quaterniond &result) const;
	inline void calculateQdot(const Quaterniond &q, const Vector3d & angularVelocity, Quaterniond &result) const;

	/*
	Calculate the derivation of the Angular velocity.
	*/
	inline void calculateWdot(const RigidBody * rigidBody, Real time, Vector3d & result) const;
	inline void calculateWdot(const Vector3d & angularVelocity, const Matrix3d & inertiaTensor, const Matrix3d & invertedInertiaTensor, const Vector3d & torque, Vector3d & result) const;

	void computeAllForces(Real time);

};

