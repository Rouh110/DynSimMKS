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
void addVector3d(const Eigen::Vector3d &q1, const Eigen::Vector3d &q2, Eigen::Vector3d &result)
{
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
	Eigen::Quaterniond q;
	Eigen::Quaterniond w;
	w.w() = 0;
	Eigen::Vector3d angularVel;
	Eigen::Quaterniond tempQuaternion;
	Eigen::Quaterniond tempQuaternionW;
	Eigen::Vector3d kvone;
	Eigen::Vector3d kvtwo;
	Eigen::Vector3d kvthree;
	Eigen::Vector3d kvfour;
	Eigen::Vector3d kxone;
	Eigen::Vector3d kxtwo;
	Eigen::Vector3d kxthree;
	Eigen::Vector3d kxfour;
	Eigen::Vector3d kwone;
	Eigen::Vector3d kwtwo;
	Eigen::Vector3d kwthree;
	Eigen::Vector3d kwfour;
	Eigen::Quaterniond kqone;
	Eigen::Quaterniond kqtwo;
	Eigen::Quaterniond kqthree;
	Eigen::Quaterniond kqfour;
	Eigen::Quaterniond kuttaResult_Q;
	Eigen::Vector3d kuttaResult_W;
	Eigen::Vector3d kuttaResult_V;
	Eigen::Vector3d kuttaResult_X;
	Real time = TimeManager::getCurrent()->getTime();
	
	for each (RigidBody* rigidBody in SimulationManager::getInstance()->getObjectManager().getRigidBodies())
	{

		if (rigidBody->isStatic())
			continue;

		// calculate X
		kxone = h * rigidBody->getVelocity();
		calculateVdot(rigidBody,time, kvone);
		kvone = h * kvone;
		kxtwo = h * (1 / 2 * kvone + rigidBody->getVelocity());
		calculateVdot(rigidBody,(1/2 * h)+time, kvtwo);
		kvtwo = h * kvtwo;
		kxthree = h * (1 / 2 * kvtwo + rigidBody->getVelocity());
		calculateVdot(rigidBody, (1 / 2 * h) + time, kvthree);
		kvthree = h * kvthree;
		kxfour = h * (1 / 2 * kvthree + rigidBody->getVelocity());
		calculateVdot(rigidBody,h+time, kvfour);
		kvfour = h * kvfour;
		Eigen::Vector3d tmp;
		Eigen::Vector3d tmp2;
		Eigen::Vector3d tmp3;

		addVector3d(kvone, (2 * kvtwo), tmp);
		addVector3d((2 * kvthree), kvfour, tmp2);
		addVector3d(tmp, tmp2, tmp3);
		tmp3 = 1 / 6 * tmp3;
		addVector3d(tmp3, rigidBody->getVelocity(), kuttaResult_V);
		
		addVector3d(kxone, (2 * kxtwo), tmp);
		addVector3d((2 * kxthree), kxfour, tmp2);
		addVector3d(tmp, tmp2, tmp3);
		tmp3 = 1 / 6 * tmp3;
		addVector3d(tmp3, rigidBody->getPosition(), kuttaResult_X);
		 
		rigidBody->setPosition(rigidBody->getPosition() + kuttaResult_X);

		// calculate v
		rigidBody->setVelocity(rigidBody->getVelocity() + kuttaResult_V);

		// calculate q
	//	calculateQdot(rigidBody, tempQuaternion);
//		scaleQuaternion(h, tempQuaternion, tempQuaternion);
//		addQuaternions(rigidBody->getRotation(), tempQuaternion, tempQuaternion);
//		rigidBody->setRotation(tempQuaternion.normalized());

		//w(t_0 + h) = w_0 + 1 / 6(k_w1 + 2 * k_w2 + 2 * k_w3 + k_w4)
		//	k_w1 = h*(J^-1 (T_0 - w_0x(J*w_0)))
		// k_w2 = h*(J^-1 (T(t_0 + h / 2) - (w_0 + 0.5*k_w1)x(J*(w_0 + 0.5*k_w1)))
		// k_w3 = h*(J^-1 (T(t_0 + h / 2) - (w_0 + 0.5*k_w2)x(J*(w_0 + 0.5*k_w2))))
		//k_w4 =  h*( J^-1 (T(t_0+h)-(w_0+k_w3)x(J*(w_0+k_w3))))
		/*
		[05.11.2014 23:15:33] Jan: k_q1 = h * 0.5 * w_0*q_0

 Jan: k_q2 = h*0.5((w_0+0.5*k_w1)*(q_0+0.5*k_q1))
[05.11.2014 23:20:30] Jan: k_q3 = h*0.5((w_0+0.5*k_w2)*(q_0+0.5*k_q2))
[05.11.2014 23:21:01] Jan: k_q4 = h*0.5((w_0+k_w3)*(q_0+k_q3))
		
				*/
		
		Quaterniond wsnake(rigidBody->getTorque().x(), rigidBody->getTorque().y(), rigidBody->getTorque().z(), 0);
		//____________________________________kwone________________________________________________________
		multDiagonalMatrix2Vector(rigidBody->getInertiaTensor(), rigidBody->getAngulaVelocity(), tmp);
		addVector3d(rigidBody->getTorque(), -1*(rigidBody->getAngulaVelocity().cross(tmp)), tmp);
		multDiagonalMatrix2Vector(rigidBody->getInvertedInertiaTensor(), tmp, tmp);
		kwone = h *  tmp;
		//____________________________________kqone________________________________________________________
		tempQuaternion = wsnake * rigidBody->getRotation();
		scaleQuaternion(1/2 * h, tempQuaternion, kqone);
		//____________________________________kwtwo__________________________________________________________
		multDiagonalMatrix2Vector(rigidBody->getInertiaTensor(), (rigidBody->getAngulaVelocity() + 1 / 2 * kwone), tmp);
		addVector3d(rigidBody->getTorque(), -1 * ((rigidBody->getAngulaVelocity() + 1 / 2 * kwone).cross(tmp)), tmp);
		multDiagonalMatrix2Vector(rigidBody->getInvertedInertiaTensor(), tmp, tmp);
		kwtwo = h * tmp;
		//____________________________________kqtwo________________________________________________________
		scaleQuaternion(1 / 2, kqone, tempQuaternion); 
		addQuaternions(rigidBody->getRotation(), tempQuaternion, tempQuaternion);
		tempQuaternionW.x() = kwone.x();
		tempQuaternionW.y() = kwone.y();
		tempQuaternionW.z() = kwone.z();
		tempQuaternionW.w() = 0;
		scaleQuaternion(1 / 2, tempQuaternionW, tempQuaternionW);
		addQuaternions(wsnake, tempQuaternionW, tempQuaternionW);
		scaleQuaternion(1 / 2 * h, tempQuaternion * tempQuaternionW, kqtwo);
		//____________________________________kwthree________________________________________________________
		multDiagonalMatrix2Vector(rigidBody->getInertiaTensor(), (rigidBody->getAngulaVelocity() + (1 / 2 * kwtwo)), tmp);
		addVector3d(rigidBody->getTorque(), -1 * ((rigidBody->getAngulaVelocity() + 1 / 2 * kwtwo).cross(tmp)), tmp);
		multDiagonalMatrix2Vector(rigidBody->getInvertedInertiaTensor(), tmp, tmp);
		kwthree = h * tmp;
		//____________________________________kqthree________________________________________________________
		scaleQuaternion(1 / 2, kqtwo, tempQuaternion);
		addQuaternions(rigidBody->getRotation(), tempQuaternion, tempQuaternion);
		tempQuaternionW.x() = kwtwo.x();
		tempQuaternionW.y() = kwtwo.y();
		tempQuaternionW.z() = kwtwo.z();
		tempQuaternionW.w() = 0;
		scaleQuaternion(1 / 2, tempQuaternionW, tempQuaternionW);
		addQuaternions(wsnake, tempQuaternionW, tempQuaternionW);
		scaleQuaternion(1 / 2 * h, tempQuaternion * tempQuaternionW, kqthree);
		//____________________________________kwfour________________________________________________________
		multDiagonalMatrix2Vector(rigidBody->getInertiaTensor(), (rigidBody->getAngulaVelocity() + kwthree), tmp);
		addVector3d(rigidBody->getTorque(), -1 * ((rigidBody->getAngulaVelocity() + kwthree).cross(tmp)), tmp);
		multDiagonalMatrix2Vector(rigidBody->getInvertedInertiaTensor(), tmp, tmp);
		kwfour = h * tmp;
		//____________________________________kqfour________________________________________________________
		addQuaternions(rigidBody->getRotation(), tempQuaternion, tempQuaternion);
		tempQuaternionW.x() = kwthree.x();
		tempQuaternionW.y() = kwthree.y();
		tempQuaternionW.z() = kwthree.z();
		tempQuaternionW.w() = 0;
		addQuaternions(wsnake, tempQuaternionW, tempQuaternionW);
		scaleQuaternion(1 / 2 * h, tempQuaternion * tempQuaternionW, kqfour);
		//____________________________________qresult________________________________________________________
		scaleQuaternion(2, kqtwo, tempQuaternion);
		addQuaternions(kqone, tempQuaternion, tempQuaternion);
		scaleQuaternion(2, kqthree, tempQuaternionW);
		addQuaternions(tempQuaternionW, kqfour, tempQuaternionW);
		addQuaternions(tempQuaternion, tempQuaternionW, tempQuaternion);
		scaleQuaternion(1 / 6, tempQuaternion, tempQuaternion);
		addQuaternions(rigidBody->getRotation(), tempQuaternion, kuttaResult_Q);
		rigidBody->setRotation(kuttaResult_Q);
		//____________________________________wresult________________________________________________________
		addVector3d(kwone, (2 * kwtwo), tmp);
		addVector3d((2 * kwthree), kwfour, tmp2);
		addVector3d(tmp, tmp2, tmp3);
		tmp3 = 1 / 6 * tmp3;
		addVector3d(tmp3, rigidBody->getAngulaVelocity(), kuttaResult_W);
		rigidBody->setAngulaVelocity(kuttaResult_W);
	}



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


