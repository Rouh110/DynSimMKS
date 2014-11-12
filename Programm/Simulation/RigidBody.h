#pragma once

#include "Common/Config.h"
#include <Eigen/Dense>


/*
Represents an Rigidbody
The Rigidbody will be Static if the mass is set to 0.
*/
class RigidBody
{
protected:
	Real mass;
	Eigen::Vector3d inertiaTensor;
	Eigen::Vector3d invertedInertiaTensor;
	Eigen::Vector3d position;
	Eigen::Vector3d velocity;
	Eigen::Vector3d angulaVelocity;
	Eigen::Vector3d torque;
	Eigen::Vector3d force;
	Eigen::Quaterniond rotation;
public:
	RigidBody();
	~RigidBody();

	bool isStatic() const;

	Real getMass() const;

	void getInertiaTensor(Eigen::Matrix3d &out_Tensor) const;
	void getInvertedInertiaTensor(Eigen::Matrix3d & out_InvertedTensor) const;

	const Eigen::Vector3d& getPosition() const;
	const Eigen::Vector3d& getVelocity() const;
	const Eigen::Vector3d& getAngulaVelocity()const;
	const Eigen::Vector3d& getTorque() const;
	const Eigen::Vector3d& getForce() const;
	const Eigen::Quaterniond& getRotation() const;
	
	/*
	Sets the mass. The Rigidbody will be static if the mass is 0.
	*/
	void setMass(Real mass);
	/*Sets the position of the RigidBody*/
	void setPosition(const Eigen::Vector3d &position);
	/*Sets the velocity of the rigidbody*/
	void setVelocity(const Eigen::Vector3d &velocity);
	/*Sets the angula velocity of the rigid body*/
	void setAngulaVelocity(const Eigen::Vector3d &angulaVelocity);
	/*
	Adds an Force for the Rigidbody at the given position in global space.
	*/
	void addForceAtGlobalPosition(const Eigen::Vector3d &force, const Eigen::Vector3d &position);
	/*
	Adds an Force for the RigidBody at the given position in local space.
	The Force vector must be in global space.
	*/
	void addForce(const Eigen::Vector3d &force, const Eigen::Vector3d &position);

	/*Adds a force to the RigidBody at the center of mass. So there will be no torque*/
	void addForce(const Eigen::Vector3d &force);

	/*Adds torque to the rigidbody*/
	void addTorqe(const Eigen::Vector3d & torque);

	
	/*Adds an Impulse to the RigidBody.*/
	void addRasImpuls(const Eigen::Vector3d & impulse, const Eigen::Vector3d & ras);

	/*
	Resets the fores to zero.
	*/
	void resetForces();
	/*Sets the rotation of the RigidBody. The Rotation must be normalized !!!!*/
	void setRotation(const Eigen::Quaterniond & rotation);

	

protected:
	/*adds the RigidBody to the ObjectManager*/
	void addToObjectManager();
	/*Calulates and sets the inertiaTensor and the inverted inertia tensor of the RigidBody. */
	virtual void calculateTensor() = 0;

};

