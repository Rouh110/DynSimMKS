#include "RigidBody.h"
#include "Simulation/SimMath.h"

using namespace IBDS;

RigidBody::RigidBody()
{
	mass = 1;
	inertiaTensor.setZero();
	invertedInertiaTensor.setZero();
	position.setZero();
	velocity.setZero();
	angulaVelocity.setZero();
	torque.setZero();
	force.setZero();
	rotation.setIdentity();
}


RigidBody::~RigidBody()
{
}

bool RigidBody::isStatic() const
{
	
	return std::abs(mass) <= SimMath::eps;
}

Real RigidBody::getMass() const
{
	return mass;
}
const Eigen::Vector3d& RigidBody::getInertiaTensor() const
{
	return inertiaTensor;
}
const Eigen::Vector3d& RigidBody::getInvertedInertiaTensor() const
{
	return invertedInertiaTensor;
}
const Eigen::Vector3d& RigidBody::getPosition() const
{
	return position;
}
const Eigen::Vector3d& RigidBody::getVelocity() const
{
	return velocity;
}
const Eigen::Vector3d& RigidBody::getAngulaVelocity() const
{
	return angulaVelocity;
}
const Eigen::Vector3d& RigidBody::getTorque() const
{
	return this->torque;
}
const Eigen::Vector3d& RigidBody::getForce() const
{
	return force;
}
const Eigen::Quaterniond& RigidBody::getRotation() const
{
	return rotation;
}


void RigidBody::setMass(Real mass)
{
	this->mass = mass;
}
void RigidBody::setPosition(const Eigen::Vector3d &position)
{
	this->position = position;
}
void RigidBody::setVelocity(const Eigen::Vector3d &velocity)
{
	this->velocity = velocity;
}
void RigidBody::setAngulaVelocity(const Eigen::Vector3d &angulaVelocity)
{
	this->angulaVelocity = angulaVelocity;
}
void RigidBody::addForce(const Eigen::Vector3d &force)
{
	this->force += force;
}
void RigidBody::resetForces()
{
	force.setZero();
	torque.setZero();
}

void RigidBody::setRotation(const Eigen::Quaterniond & rotation)
{
	this->rotation = rotation;
}

void RigidBody::addForceAtGlobalPosition(const Eigen::Vector3d &force, const Eigen::Vector3d &position)
{
	addForce(force, (position - this->position));
}

void RigidBody::addForce(const Eigen::Vector3d &force, const Eigen::Vector3d &localposition)
{
	torque += (localposition).cross(force);
	addForce(force);
}