#include "RigidBody.h"
#include "Simulation/SimMath.h"
#include "SimulationManager.h"

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
	impulse.setZero();
	torqueImpulse.setZero();
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

void RigidBody::getInertiaTensor(Eigen::Matrix3d &out_Tensor) const
{
	out_Tensor = rotation.toRotationMatrix()*(inertiaTensor.asDiagonal()* rotation.toRotationMatrix().transpose());
}
void RigidBody::getInvertedInertiaTensor(Eigen::Matrix3d &out_InvertedTensor) const
{
	out_InvertedTensor = rotation.toRotationMatrix()*(invertedInertiaTensor.asDiagonal()* rotation.toRotationMatrix().transpose());
}

const Eigen::Vector3d& RigidBody::getPosition() const
{
	return position;
}
const Eigen::Vector3d& RigidBody::getVelocity() const
{
	return velocity;
}


void RigidBody::getVelocityOfLocalPoint(const Eigen::Vector3d & point, Eigen::Vector3d & out_velocity) const
{
	out_velocity = velocity + angulaVelocity.cross(point);
}

void RigidBody::getVelocityOfGlobalPoint(const Eigen::Vector3d & point, Eigen::Vector3d & out_velocity) const
{
	getVelocityOfLocalPoint(point-position,out_velocity);
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

const Eigen::Vector3d& RigidBody::getImpulse() const
{
	return impulse;
}
const Eigen::Vector3d& RigidBody::getTorqueImpulse() const
{
	return torqueImpulse;
}

const BoundingVolumeTree* RigidBody::getVolumeTree() const
{
	return &(this->volumeTree);
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


void RigidBody::addRasImpuls(const Vector3d & impulse, const Vector3d & ras)
{
	Matrix3d invertedInertiaTensor;
	getInvertedInertiaTensor(invertedInertiaTensor);
	this->impulse += impulse;
	torqueImpulse += invertedInertiaTensor * (ras.cross(impulse));
	/*
	setVelocity(velocity+ (1.0/mass * impulse));
	Matrix3d invertedInertiaTensor;
	getInvertedInertiaTensor(invertedInertiaTensor);
	setAngulaVelocity(angulaVelocity + (invertedInertiaTensor * (ras.cross(impulse))));
	*/
}

void RigidBody::resetForces()
{
	force.setZero();
	torque.setZero();
}

void RigidBody::resetImpulse()
{
	impulse.setZero();
	torqueImpulse.setZero();
}

void RigidBody::setRotation(const Eigen::Quaterniond & rotation)
{
	this->rotation = rotation;
}

void RigidBody::addForceAtGlobalPosition(const Eigen::Vector3d &force, const Eigen::Vector3d &position)
{
	torque += (position - this->position).cross(force);
	addForce(force);
}

void RigidBody::addForce(const Eigen::Vector3d &force, const Eigen::Vector3d &localposition)
{
	torque += localposition.cross(rotation.inverse()._transformVector(force));
	addForce(force);
}

void RigidBody::addTorqe(const Eigen::Vector3d & torque)
{
	this->torque += torque;
}

void RigidBody::addToObjectManager()
{
	SimulationManager::getInstance()->getObjectManager().addObject(this);
}