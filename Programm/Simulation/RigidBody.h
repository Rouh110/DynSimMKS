#pragma once

#include "Common/Config.h"
#include <Eigen/Dense>



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
	const Eigen::Vector3d& getInertiaTensor() const;
	const Eigen::Vector3d& getInvertedInertiaTensor() const;
	const Eigen::Vector3d& getPosition() const;
	const Eigen::Vector3d& getVelocity() const;
	const Eigen::Vector3d& getAngulaVelocity()const;
	const Eigen::Vector3d& getTorque() const;
	const Eigen::Vector3d& getForce() const;
	const Eigen::Quaterniond& getRotation() const;
	

	void setMass(Real mass);
	void setPosition(const Eigen::Vector3d &position);
	void setVelocity(const Eigen::Vector3d &velocity);
	void setAngulaVelocity(const Eigen::Vector3d &angulaVelocity);
	void addForceAtGlobalPosition(const Eigen::Vector3d &force, const Eigen::Vector3d &position);
	void addForce(const Eigen::Vector3d &force, const Eigen::Vector3d &position);
	void addForce(const Eigen::Vector3d &force);
	void resetForces();
	void setRotation(const Eigen::Quaterniond & rotation);

};

