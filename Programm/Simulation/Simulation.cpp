#include "Simulation.h"
#include"Simulation\TimeManager.h"
#include "SimulationManager.h"

using namespace IBDS;

Simulation::Simulation()
: approximationMethod(EXPLICIT_EULER),
iterationCount(1)
{
}


Simulation::~Simulation()
{
}

void Simulation::setApproximationMethod(ApproximationMethod method)
{
	approximationMethod = method;
}

Simulation::ApproximationMethod Simulation::getApproximationMethod()
{
	return approximationMethod;
}

void Simulation::update(Real h)
{
	switch (approximationMethod)
	{
	case EXPLICIT_EULER:
		simulateExplicitEuler(h);
		break;
	case RUNGE_KUTTA_4:
		simulateRungeKutta4(h);
		break;
	}
	resetForces();
	
}

void Simulation::resetForces()
{
	for each (RigidBody* rigidBody in SimulationManager::getInstance()->getObjectManager().getRigidBodies())
	{
		rigidBody->resetForces();
	}
}

void addQuaternions(const Eigen::Quaterniond &q1, const Eigen::Quaterniond &q2, Eigen::Quaterniond &result)
{
	result.w() = q1.w() + q2.w();
	result.x() = q1.x() + q2.x();
	result.y() = q1.y() + q2.y();
	result.z() = q1.z() + q2.z();
}

void scaleQuaternion(Real factor, const Eigen::Quaterniond &q, Eigen::Quaterniond &result)
{
	result.w() = q.w() * factor;
	result.x() = q.x() * factor;
	result.y() = q.y() * factor;
	result.z() = q.z() * factor;

}

void multDiagonalMatrix2Vector(const Eigen::Vector3d & diagMatrix,const Eigen::Vector3d & vector, Eigen::Vector3d &result)
{
	result.x() = diagMatrix.x()*vector.x();
	result.y() = diagMatrix.y()*vector.y();
	result.z() = diagMatrix.z()*vector.z();
}

void Simulation::simulateExplicitEuler(Real h)
{
	Eigen::Quaterniond q;
	Eigen::Quaterniond w;
	w.w() = 0;
	Eigen::Vector3d angularVel;
	Eigen::Vector3d tempVector;
	Eigen::Quaterniond tempQuaternion;

	Real time = TimeManager::getCurrent()->getTime();

	for each (RigidBody* rigidBody in SimulationManager::getInstance()->getObjectManager().getRigidBodies())
	{
		
		if (rigidBody->isStatic())
			continue;

		// calculate X
		calculateXdot(rigidBody, tempVector);
		rigidBody->setPosition(rigidBody->getPosition() + h* tempVector);

		// calculate v
		calculateVdot(rigidBody, time, tempVector);
		rigidBody->setVelocity(rigidBody->getVelocity() + (h*tempVector));

		// calculate q
		calculateQdot(rigidBody, tempQuaternion);
		scaleQuaternion(h, tempQuaternion, tempQuaternion);
		addQuaternions(rigidBody->getRotation(), tempQuaternion, tempQuaternion);
		rigidBody->setRotation(tempQuaternion.normalized());

		// calcualte w
		calculateWdot(rigidBody, time, tempVector);
		rigidBody->setAngulaVelocity(rigidBody->getAngulaVelocity() + (h*tempVector));
		
		/*
		
		if (rigidBody->isStatic())
			continue;

		rigidBody->setPosition(rigidBody->getPosition() + h*rigidBody->getVelocity());
		rigidBody->setVelocity(rigidBody->getVelocity() + (h / rigidBody->getMass())*(rigidBody->getForce()));

		angularVel = rigidBody->getAngulaVelocity();
		w.x() = angularVel.x();
		w.y() = angularVel.y();
		w.z() = angularVel.z();

		q = (w*rigidBody->getRotation());
		scaleQuaternion(h / 2, q, q);

		addQuaternions(rigidBody->getRotation(), q, q);
		q.normalize();
		rigidBody->setRotation(q);

		printf("Torque: %f, %f, %f \n", rigidBody->getTorque().x(), rigidBody->getTorque().y(), rigidBody->getTorque().z());
		printf("AngulaVel: %f, %f, %f \n", rigidBody->getAngulaVelocity().x(), rigidBody->getAngulaVelocity().y(), rigidBody->getAngulaVelocity().z());

		//Calcualte angula velocity;
		multDiagonalMatrix2Vector(rigidBody->getInertiaTensor(), angularVel, tempVector);
		multDiagonalMatrix2Vector(rigidBody->getInvertedInertiaTensor(), rigidBody->getTorque() - angularVel.cross(tempVector), tempVector);
		rigidBody->setAngulaVelocity(angularVel + h*tempVector);
		*/

	}
		
	
}

void Simulation::simulateRungeKutta4(Real h)
{

}


void Simulation::calculateXdot(const RigidBody * rigidBody, Vector3d & result) const
{
	calculateXdot(rigidBody->getVelocity(), result);
}

void Simulation::calculateXdot(const Vector3d & velocity, Vector3d & result) const
{
	result = velocity;
}

void Simulation::calculateVdot(const RigidBody *rigidBody, Real time,  Vector3d & result) const
{
	calculateVdot(rigidBody->getMass(), rigidBody->getForce(), result);
}

void Simulation::calculateVdot(Real mass, const Vector3d & force,  Vector3d & result) const
{
	result = (1 / mass)*force;
}

void Simulation::calculateQdot(const RigidBody * rigidBody, Quaterniond &result) const
{
	calculateQdot(rigidBody->getRotation(), rigidBody->getAngulaVelocity(), result);
}

void Simulation::calculateQdot(const Quaterniond &q, const Vector3d & angularVelocity, Quaterniond &result) const
{
	result.w() = 0;
	result.x() = angularVelocity.x();
	result.y() = angularVelocity.y();
	result.z() = angularVelocity.z();

	result = (result*q);
	scaleQuaternion(0.5, result, result);
}

void Simulation::calculateWdot(const RigidBody * rigidBody, Real time, Vector3d & result) const
{
	calculateWdot(rigidBody->getAngulaVelocity(), rigidBody->getInertiaTensor(), rigidBody->getInvertedInertiaTensor(), rigidBody->getTorque(), result);
}

void Simulation::calculateWdot(const Vector3d & angularVelocity, const Vector3d & inertiaTensor, const Vector3d & invertedInertiaTensor, const Vector3d & torque, Vector3d & result) const
{
	multDiagonalMatrix2Vector(inertiaTensor, angularVelocity, result);
	multDiagonalMatrix2Vector(invertedInertiaTensor, torque - angularVelocity.cross(result), result);
}


