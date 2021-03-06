#pragma once
#include "Eigen/Dense"
#include "RigidBody.h"
#include "Sphere.h"
#include "Cube.h"
#include "Common\Config.h"
#include <vector>
#include <list>

using namespace Eigen;
using namespace std;
/*
Simulates all non-static RigidBodys
*/
class Simulation
{
public:
	enum ApproximationMethod {EXPLICIT_EULER, RUNGE_KUTTA_4};
protected:
	ApproximationMethod approximationMethod;
	bool collisionCheck = true;
	unsigned int iterationCount;
	list<BoundingVolume*> collidedBoundingVolumes;

public:
	Simulation();
	~Simulation();

	/*
	Sets the Method that will be used for calculation the next step.
	*/
	void setApproximationMethod(ApproximationMethod method);

	/*Returns the Approximation method currently used, to compute the Forces.*/
	ApproximationMethod getApproximationMethod();

	/*
	Method for collision check
	*/
	void setCollisionCheck(bool check);
	bool getCollisionCheck();

	/*Updates the Simualtion*/
	void update(Real h);

	const list<BoundingVolume*> & getCollidedBoundingVolumes() const;
	void resetBoundingVolumeList();
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

	/*Simulate the joints for one time step with the predictor-corrector procedure*/
	void simulateJointsPredictorCorrector(Real h, std::vector<Matrix3d*> &inverseKs, Real acceptedError, Real maxError = 0);

	/* Compute one step with predictor-corrector procedure for all joint*/
	void computeJoints(Real h);

	/*
	Calculate the derivation of the position.
	*/
	inline void calculateXdot(const RigidBody *rigidBody, Vector3d & result) const;
	inline void calculateXdot(const Vector3d & velocity, Vector3d & result) const;

	/*
	Calculate the derivation of the velocity.
	*/
	inline void calculateVdot(const RigidBody * rigidBody, Vector3d & result) const;
	inline void calculateVdot(Real mass, const Vector3d & force, Vector3d & result) const;

	/*
	Calculate the derivation of the Rotiaiton.
	*/
	inline void calculateQdot(const RigidBody * rigidBody, Quaterniond &result) const;
	inline void calculateQdot(const Quaterniond &q, const Vector3d & angularVelocity, Quaterniond &result) const;

	/*
	Calculate the derivation of the Angular velocity.
	*/
	inline void calculateWdot(const RigidBody * rigidBody, Vector3d & result) const;
	inline void calculateWdot(const Vector3d & angularVelocity, const Matrix3d & inertiaTensor, const Matrix3d & invertedInertiaTensor, const Vector3d & torque, Vector3d & result) const;

	/*
	Calculate the matrix K vor the given RigidBody.
	*/
	inline void calculateK(const RigidBody & rigidBody, const Vector3d & ras, const Vector3d & rbs,Matrix3d & result);
	
	/*
	Calculate the matrix K vor the given RigidBody.
	*/
	inline void calculateK(const RigidBody & rigidBody, const Matrix3d & ras, const Matrix3d & rbs, Matrix3d & result);

	/*Computes every force currenty in the scene*/
	void computeAllForces(Real time);

	/*Compute the impulse for all RigidBodys
	Resets the Impulse in each RigidBody
	static RigidBodys will be ignored*/
	void computeImpulse();

	/* Compute the impulse for the given RigidBody
	Resets the Impulse in the RigidBody.
	static RigidBodys will be ignored.*/
	void computeImpulse(RigidBody * rigidBody);

	/*set up the inverse of K and stor it in the given vector.
	Don't forget to delete all K*/
	void setUpKInverse(std::vector<Matrix3d*> &out_KInverseList);

	/*Calculates the the corrector impulses for every Joint and add them to the rigidBodys*/
	void computeAllJoint(Real h,const std::vector<Matrix3d*> &KInverses, Real acceptedError,Real & out_maxError);

	/**/
	void computeVeloctyCorrection();

	void checkCollision();
	void checkCollision(Cube* rigidBodyA, Cube* rigidBodyB);
	void checkCollision(Sphere* sphereA, Sphere* sphereB);
	void checkCollision(Sphere* sphere, Cube* cube);

	void collisionCalc(RigidBody* rigidBodyA, BoundingVolume* volumeA, RigidBody* rigidBodyB, BoundingVolume* volumeB);
	
	void checkCollisionWithYAxis(RigidBody* rigidBody);

	/*
	bool collisionTestYAxis(RigidBody* rigidBody, BoundingVolume* boundingVolume);
	bool collisionTest(RigidBody* rigidBodyA, BoundingVolume* volumeA, RigidBody* rigidBodyB, BoundingVolume* volumeB);
	void collisionCalc(RigidBody* rigidBodyA, BoundingVolume* volumeA, RigidBody* rigidBodyB, BoundingVolume* volumeB);
	void collisionCalcYAxis(RigidBody* rigidBody, BoundingVolume* boundingVolume);
	*/
};

