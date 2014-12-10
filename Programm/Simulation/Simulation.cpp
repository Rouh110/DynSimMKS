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

const list<BoundingVolume*> & Simulation::getCollidedBoundingVolumes() const
{
	return collidedBoundingVolumes;
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

	//simulateJointsPredictorCorrector(h);
	resetForces();
	checkCollision();

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


void Simulation::setUpKInverse(std::vector<Matrix3d*> &out_KInverseList)
{
	
	RigidBody* rigidBodyA;
	RigidBody* rigidBodyB;
	Matrix3d Kaa;
	Matrix3d Kbb;

	Vector3d tmp;
	Matrix3d *inverseK;

	for each (IJoint* joint in SimulationManager::getInstance()->getObjectManager().getJoints())
	{
		rigidBodyA = joint->getRigidBodyA();
		rigidBodyB = joint->getRigidBodyB();

		//setup inverse K
		inverseK = new Matrix3d();

		if (!rigidBodyA->isStatic())
		{
			joint->getRas(tmp);
			calculateK(*rigidBodyA, tmp, tmp, Kaa);
		}
		else
		{
			Kaa = Matrix3d::Zero();
		}

		if (!rigidBodyB->isStatic())
		{
			joint->getRbs(tmp);
			calculateK(*rigidBodyB, tmp, tmp, Kbb);
		}
		else
		{
			Kbb = Matrix3d::Zero();
		}


		(*inverseK) = (Kaa + Kbb).inverse();

		out_KInverseList.push_back(inverseK);

	}
	
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

	Real damper = (1.0 - 0.00001);
	Real maxError = 0;
	Real delta = 0.00001;
	std::vector<Matrix3d *> inverseKs;
	
	//compute the impulse that applyed to the rigidbody bedween updates
	computeImpulse();

	//Set up K inverse for Joint correction
	setUpKInverse(inverseKs);

	//compute forces for Eulerstep.
	computeAllForces(time);

	//complete Euler step
	for each (RigidBody* rigidBody in SimulationManager::getInstance()->getObjectManager().getRigidBodies())
	{

		if (rigidBody->isStatic())
			continue;

		// calculate X
		calculateXdot(rigidBody, tempVector);
		rigidBody->setPosition(rigidBody->getPosition() + h* tempVector);

		// calculate v
		calculateVdot(rigidBody, tempVector);
		rigidBody->setVelocity(rigidBody->getVelocity() + (h*tempVector));

		// calculate q
		calculateQdot(rigidBody, tempQuaternion);
		scaleQuaternion(h, tempQuaternion, tempQuaternion);
		addQuaternions(rigidBody->getRotation(), tempQuaternion, tempQuaternion);
		rigidBody->setRotation(tempQuaternion.normalized());

		// calcualte w
		calculateWdot(rigidBody, tempVector);
		rigidBody->setAngulaVelocity(rigidBody->getAngulaVelocity() + (h*tempVector));

	}
	

	computeAllJoint(h, inverseKs, delta, maxError);

	simulateJointsPredictorCorrector(h,inverseKs,delta,maxError);

	// clean up
	for each (Matrix3d * matrix in inverseKs)
	{

		delete matrix;
	}
	
	
}


struct State
{
	Vector3d position;
	Vector3d velocity;
	Vector3d angularVelocity;
	Quaterniond rotation;

};

void Simulation::simulateRungeKutta4(Real h)
{

	std::vector<State*> initialStates;
	std::vector<State*> results;

	Real time = TimeManager::getCurrent()->getTime();


	Real damper = (1.0 - 0.00001);
	Real maxError = 0;
	Real delta = 0.00001;
	std::vector<Matrix3d *> inverseKs;

	//compute the impulse that applyed to the rigidbody bedween updates
	computeImpulse();

	//Set up K inverse for Joint correction
	setUpKInverse(inverseKs);

	//k1
	
	Vector3d xdot;
	Vector3d vdot;
	Vector3d wdot;
	Quaterniond qdot;

	State tmpState;
	State *initialState;
	State *resultState;

	int i = 0;
	computeAllForces(time);
	for each (RigidBody* rigidBody in SimulationManager::getInstance()->getObjectManager().getRigidBodies())
	{
		if (rigidBody->isStatic())
			continue;

		//Initialze the States and result

		State *initialState = new State();

		initialState->position = rigidBody->getPosition();
		initialState->velocity = rigidBody->getVelocity();
		initialState->angularVelocity = rigidBody->getAngulaVelocity();
		initialState->rotation = rigidBody->getRotation();
		initialStates.push_back(initialState);

		resultState = new State();
		results.push_back(resultState);


		// calculates the derivatives
		calculateXdot(rigidBody, xdot);
		calculateVdot(rigidBody, vdot);
		calculateQdot(rigidBody, qdot);
		calculateWdot(rigidBody, wdot);

		// set result = k_1
		resultState->position = h*xdot;
		resultState->velocity = h*vdot;
		scaleQuaternion(h,qdot,qdot);
		resultState->rotation = qdot;
		resultState->angularVelocity = h*wdot;
		
		
		//setting up the rigidbody for next calculation
		rigidBody->setPosition((initialState->position + 0.5 * resultState->position));
		rigidBody->setVelocity((initialState->velocity + 0.5 * resultState->velocity));
		scaleQuaternion(0.5, qdot, qdot);
		addQuaternions(initialState->rotation, qdot, qdot);
		rigidBody->setRotation(qdot.normalized());
		rigidBody->setAngulaVelocity((initialState->angularVelocity + 0.5 * resultState->angularVelocity));

		rigidBody->resetForces();
		i++;

	}
	
	//computeAllJoint(h/2, inverseKs, delta, maxError);
	//simulateJointsPredictorCorrector(h/2, inverseKs, delta, maxError);

	//k2
	i = 0;
	computeAllForces(time+0.5*h);
	for each (RigidBody* rigidBody in SimulationManager::getInstance()->getObjectManager().getRigidBodies())
	{
		if (rigidBody->isStatic())
			continue;

		// calculates the derivatives
		calculateXdot(rigidBody, xdot);
		calculateVdot(rigidBody, vdot);
		calculateQdot(rigidBody, qdot);
		calculateWdot(rigidBody, wdot);

		// calculate k_2
		tmpState.position = h*xdot;
		tmpState.velocity = h*vdot;
		scaleQuaternion(h, qdot, qdot);
		tmpState.rotation = qdot;
		tmpState.angularVelocity = h*wdot;

		//setting up the rigidbody for next calculation
		initialState = initialStates[i];
		rigidBody->setPosition((initialState->position + 0.5 * tmpState.position));
		rigidBody->setVelocity((initialState->velocity + 0.5 * tmpState.velocity));
		scaleQuaternion(0.5, qdot, qdot);
		addQuaternions(initialState->rotation, qdot, qdot);
		rigidBody->setRotation(qdot.normalized());
		rigidBody->setAngulaVelocity((initialState->angularVelocity + 0.5 * tmpState.angularVelocity));
		rigidBody->resetForces();

		// calculates result= k_1 + 2* k_2
		resultState = results[i];
		resultState->position += (tmpState.position * 2);
		resultState->velocity += (tmpState.velocity * 2);
		scaleQuaternion(2, tmpState.rotation, qdot);
		addQuaternions(resultState->rotation, qdot, qdot);
		resultState->rotation = qdot;
		resultState->angularVelocity += (tmpState.angularVelocity * 2);
		
		
		rigidBody->resetForces();
		i++;

	}

	//computeAllJoint(h/2, inverseKs, delta, maxError);
	//simulateJointsPredictorCorrector(h/2, inverseKs, delta, maxError);
	
	//k3
	computeAllForces(time + 0.5*h);
	i = 0;
	for each (RigidBody* rigidBody in SimulationManager::getInstance()->getObjectManager().getRigidBodies())
	{
		if (rigidBody->isStatic())
			continue;

		// calculates the derivatives
		calculateXdot(rigidBody, xdot);
		calculateVdot(rigidBody, vdot);
		calculateQdot(rigidBody, qdot);
		calculateWdot(rigidBody, wdot);

		// calculate k_3
		tmpState.position = h*xdot;
		tmpState.velocity = h*vdot;
		scaleQuaternion(h, qdot, qdot);
		tmpState.rotation = qdot;
		tmpState.angularVelocity = h*wdot;

		//setting up the rigidbody for next calculation
		initialState = initialStates[i];
		rigidBody->setPosition((initialState->position + tmpState.position));
		rigidBody->setVelocity((initialState->velocity + tmpState.velocity));
		//scaleQuaternion(1, qdot, qdot);
		addQuaternions(initialState->rotation, qdot, qdot);
		rigidBody->setRotation(qdot.normalized());
		rigidBody->setAngulaVelocity((initialState->angularVelocity + tmpState.angularVelocity));
		rigidBody->resetForces();

		// calculates result= k_1 + 2* k_2 + 2* k_3
		resultState = results[i];
		resultState->position += (tmpState.position * 2);
		resultState->velocity += (tmpState.velocity * 2);
		scaleQuaternion(2, tmpState.rotation, qdot);
		addQuaternions(resultState->rotation, qdot, qdot);
		resultState->rotation = qdot;
		resultState->angularVelocity += (tmpState.angularVelocity * 2);
		

		rigidBody->resetForces();
		i++;

	}
	
	//computeAllJoint(h, inverseKs, delta, maxError);
	//simulateJointsPredictorCorrector(h, inverseKs, delta, maxError);

	//k4
	computeAllForces(time + h);
	i = 0;
	Real factor = 1.0/6.0;
	
	for each (RigidBody* rigidBody in SimulationManager::getInstance()->getObjectManager().getRigidBodies())
	{
		if (rigidBody->isStatic())
			continue;

		// calculates the derivatives
		calculateXdot(rigidBody, xdot);
		calculateVdot(rigidBody, vdot);
		calculateQdot(rigidBody, qdot);
		calculateWdot(rigidBody, wdot);

		// calculate k_4
		tmpState.position = h*xdot;
		tmpState.velocity = h*vdot;
		scaleQuaternion(h, qdot, qdot);
		tmpState.rotation = qdot;
		tmpState.angularVelocity = h*wdot;

		resultState = results[i];
		initialState = initialStates[i];

		// calculates the result= 1/6 * (k_1 + 2* k_2 + 2* k_3 + k4)
		tmpState.position = factor*(resultState->position + (tmpState.position));
		tmpState.velocity = factor*(resultState->velocity + (tmpState.velocity));
		addQuaternions(resultState->rotation, tmpState.rotation, qdot);
		scaleQuaternion(factor, qdot, qdot);
		tmpState.rotation = qdot;
		tmpState.angularVelocity = factor*(resultState->angularVelocity + tmpState.angularVelocity);

		// sets the end result = z_0 + end result
		rigidBody->setPosition((initialState->position + tmpState.position));
		rigidBody->setVelocity((initialState->velocity + tmpState.velocity));
		addQuaternions(initialState->rotation, tmpState.rotation, qdot);
		rigidBody->setRotation(qdot.normalized());
		rigidBody->setAngulaVelocity((initialState->angularVelocity + tmpState.angularVelocity));
		rigidBody->resetForces();

		delete resultState;
		delete initialState;

		i++;

	}

	computeAllJoint(h, inverseKs, delta, maxError);

	simulateJointsPredictorCorrector(h, inverseKs, delta, maxError);

	// clean up
	for each (Matrix3d * matrix in inverseKs)
	{

		delete matrix;
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

void Simulation::calculateVdot(const RigidBody *rigidBody,  Vector3d & result) const
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

void Simulation::calculateWdot(const RigidBody * rigidBody, Vector3d & result) const
{
	Matrix3d inertiaTensor;
	Matrix3d invertedInertiaTensor;
	rigidBody->getInertiaTensor(inertiaTensor);
	rigidBody->getInvertedInertiaTensor(invertedInertiaTensor);

	calculateWdot(rigidBody->getAngulaVelocity(), inertiaTensor, invertedInertiaTensor, rigidBody->getTorque(), result);
}

void Simulation::calculateWdot(const Vector3d & angularVelocity, const Matrix3d & inertiaTensor, const Matrix3d & invertedInertiaTensor, const Vector3d & torque, Vector3d & result) const
{
	
	result = inertiaTensor *angularVelocity;
	result = invertedInertiaTensor *(torque - angularVelocity.cross(result));
	//multDiagonalMatrix2Vector(inertiaTensor, angularVelocity, result);
	//multDiagonalMatrix2Vector(invertedInertiaTensor, torque - angularVelocity.cross(result), result);
}




void Simulation::simulateJointsPredictorCorrector(Real h, std::vector<Matrix3d*> &inverseKs ,Real acceptedError,Real maxError)
{
	
	Vector3d deltaVel;
	Vector3d deltaAngularVel;
	Quaterniond q;

	int maxIterations = 100;
	int n = 0;
	
	/*should be between 1 and 0*/
	Real damper = 0.00000;
	Real damperFactor = 1 - damper;
	
	bool dampingJoint = false;
	

	while (maxError > acceptedError && n <= maxIterations)
	{
		dampingJoint = true;
		/*
		for each (RigidBody *rigidBody in SimulationManager::getInstance()->getObjectManager().getRigidBodies())
		{
			if (rigidBody->isStatic())
				continue;
			deltaVel = (1.0 / rigidBody->getMass() * rigidBody->getImpulse());
			deltaAngularVel = rigidBody->getTorqueImpulse();

			//compute the impulse for the Rigidbody
			rigidBody->setVelocity(rigidBody->getVelocity() + deltaVel);
			rigidBody->setAngulaVelocity(rigidBody->getAngulaVelocity() + deltaAngularVel);

			//set new values
			rigidBody->setPosition(rigidBody->getPosition() + (h* deltaVel));
			calculateQdot(rigidBody->getRotation(), deltaAngularVel, q);
			scaleQuaternion(h, q, q);
			addQuaternions(rigidBody->getRotation(), q, q);
			rigidBody->setRotation(q.normalized());

			rigidBody->resetImpulse();
		}
		*/

		computeAllJoint(h, inverseKs, acceptedError, maxError);

		n++;
	}

	printf("position correction iteration: %i :: max error: %f\n", n, maxError);

	dampingJoint = true;
	if (dampingJoint)
	{
		for each (RigidBody *rigidBody in SimulationManager::getInstance()->getObjectManager().getRigidBodies())
		{
			rigidBody->setAngulaVelocity(rigidBody->getAngulaVelocity() * damperFactor);
			rigidBody->setVelocity(rigidBody->getVelocity() *damperFactor);
		}
	}

	computeVeloctyCorrection();
	
}

// 0 -az ay
// az 0 -ax
//-ay ax 0
void getCrossMatrix(const Vector3d & vector ,Matrix3d & result)
{
	result <<	0, -vector.z(), vector.y(),
				vector.z(),	0, -vector.x(),
				-vector.y(), vector.x(),0;
}

void Simulation::calculateK(const RigidBody & rigidBody, const Vector3d & ras, const Vector3d & rbs, Matrix3d & result)
{
	
	Matrix3d ras_m;
	Matrix3d rbs_m;
	getCrossMatrix(ras, ras_m);
	getCrossMatrix(rbs, rbs_m);

	calculateK(rigidBody, ras_m, rbs_m, result);
	
}

void Simulation::calculateK(const RigidBody & rigidBody, const Matrix3d & ras, const Matrix3d & rbs, Matrix3d & result)
{
	Matrix3d invertedInertiaTensor;
	rigidBody.getInvertedInertiaTensor(invertedInertiaTensor);
	Real mass = rigidBody.getMass();

	result = ((1.0 / mass)*Matrix3d::Identity()) - ras*(invertedInertiaTensor*rbs);
}

void Simulation::computeAllForces(Real time)
{
	for each (IForce* force in SimulationManager::getInstance()->getObjectManager().getForces())
	{
		force->computeForce(time);
	}
}

void Simulation::computeImpulse()
{
	
	for each (RigidBody * rigidBody in  SimulationManager::getInstance()->getObjectManager().getRigidBodies())
	{
		computeImpulse(rigidBody);
	}
	
}

void Simulation::computeImpulse(RigidBody * rigidBody)
{
	if (rigidBody->isStatic())
		return;

	rigidBody->setVelocity(rigidBody->getVelocity() + ((1.0 / rigidBody->getMass()) * rigidBody->getImpulse()));
	rigidBody->setAngulaVelocity(((rigidBody->getAngulaVelocity()) + (rigidBody->getTorqueImpulse())));

	rigidBody->resetImpulse();
	//getInvertedInertiaTensor(invertedInertiaTensor);
	//setAngulaVelocity(angulaVelocity + (invertedInertiaTensor * (ras.cross(impulse))));
}


void Simulation::computeAllJoint(Real h, const std::vector<Matrix3d*> &KInverses, Real acceptedError, Real & out_maxError)
{
	
	RigidBody *rigidBodyA;
	RigidBody *rigidBodyB;
	out_maxError = 0.0;
	Matrix3d * inverseK;

	Vector3d error;
	Real norm;

	Vector3d tmp;
	Vector3d impulse;
	Real factor = 1.0 / h;

	Quaterniond q;
	Vector3d deltaVel;
	Vector3d deltaAngularVel;

	Vector3d vel;
	Vector3d angvel;

	int i = 0;
	for each (IJoint* joint in SimulationManager::getInstance()->getObjectManager().getJoints())
	{
		rigidBodyA = joint->getRigidBodyA();
		rigidBodyB = joint->getRigidBodyB();


		inverseK = KInverses[i];


		joint->getCurrentError(error);

		norm = std::abs(error.norm());

		if (out_maxError < norm)
		{
			out_maxError = norm;
		}


		//if (norm <= acceptedError)
			//continue;

		// calculate impulse
		impulse = (*inverseK)* (factor * error);

		

		if (!rigidBodyA->isStatic())
		{
			joint->getRas(tmp);
			rigidBodyA->addRasImpuls(impulse, tmp);
			//rigidBodyA->setVelocity(rigidBodyA->getVelocity()*damper);
			//rigidBodyA->setAngulaVelocity(rigidBodyA->getAngulaVelocity()*damper);
		}



		if (!rigidBodyB->isStatic())
		{
			joint->getRbs(tmp);
			rigidBodyB->addRasImpuls(impulse * -1, tmp);
			//rigidBodyB->setVelocity(rigidBodyB->getVelocity()*damper);
			//rigidBodyB->setAngulaVelocity(rigidBodyB->getAngulaVelocity()*damper);				
		}


		
		if (!rigidBodyA->isStatic())
		{
			deltaVel = (1.0 / rigidBodyA->getMass() * rigidBodyA->getImpulse());
			deltaAngularVel = rigidBodyA->getTorqueImpulse();

			//compute the impulse for the Rigidbody
			rigidBodyA->setVelocity(rigidBodyA->getVelocity() + deltaVel);
			rigidBodyA->setAngulaVelocity(rigidBodyA->getAngulaVelocity() + deltaAngularVel);

			//set new values
			rigidBodyA->setPosition(rigidBodyA->getPosition() + (h* deltaVel));
			calculateQdot(rigidBodyA->getRotation(), deltaAngularVel, q);
			scaleQuaternion(h, q, q);
			addQuaternions(rigidBodyA->getRotation(), q, q);
			rigidBodyA->setRotation(q.normalized());

			rigidBodyA->resetImpulse();
		}
		
		if (!rigidBodyB->isStatic())
		{
			deltaVel = (1.0 / rigidBodyB->getMass() * rigidBodyB->getImpulse());
			deltaAngularVel = rigidBodyB->getTorqueImpulse();

			//compute the impulse for the Rigidbody
			rigidBodyB->setVelocity(rigidBodyB->getVelocity() + deltaVel);
			rigidBodyB->setAngulaVelocity(rigidBodyB->getAngulaVelocity() + deltaAngularVel);

			//set new values
			rigidBodyB->setPosition(rigidBodyB->getPosition() + (h* deltaVel));
			calculateQdot(rigidBodyB->getRotation(), deltaAngularVel, q);
			scaleQuaternion(h, q, q);
			addQuaternions(rigidBodyB->getRotation(), q, q);
			rigidBodyB->setRotation(q.normalized());

			rigidBodyB->resetImpulse();
		}
			
		


		i++;

	}
	
}


void Simulation::computeVeloctyCorrection()
{
	std::vector < Matrix3d* > kInverses;
	setUpKInverse(kInverses);
	Matrix3d *kInverse;
	Vector3d impulse;
	Vector3d tmpVector;
	Vector3d u_a;
	Vector3d u_b;

	int i;
	int maxIteration = 1000;
	double acceptedError =  0.00001;
	
	double maxError = 0;
	double currentError;

	int iteration = 0;
	

	do
	{
		maxError = 0;
		i = 0;
		for each (IJoint* joint in SimulationManager::getInstance()->getObjectManager().getJoints())
		{
			kInverse = kInverses[i];
			
			joint->getRas(tmpVector);
			joint->getRigidBodyA()->getVelocityOfLocalPoint(tmpVector,u_a);
			joint->getRbs(tmpVector);
			joint->getRigidBodyB()->getVelocityOfLocalPoint(tmpVector, u_b);
		
			impulse = (*kInverse) * (u_a - u_b);
			
			currentError = abs(impulse.norm());
			//printf("currentError: %f \n", currentError);
			if (currentError > maxError)
				maxError = currentError;

			joint->getRas(tmpVector);
			joint->getRigidBodyA()->addRasImpuls(impulse*-1, tmpVector);

			joint->getRbs(tmpVector);
			joint->getRigidBodyB()->addRasImpuls(impulse*1, tmpVector);

			computeImpulse(joint->getRigidBodyA());
			computeImpulse(joint->getRigidBodyB());

			i++;
		}
		/*
		for each (RigidBody * rigidBody in SimulationManager::getInstance()->getObjectManager().getRigidBodies())
		{
			computeImpulse(rigidBody);
		}
		*/
		iteration++;
	} while (maxError > acceptedError && iteration < maxIteration);

	printf("interations for velocity correction: %i\n",iteration);
	printf("Max Velocity Error: %f\n", maxError);

	// clean up
	for each (Matrix3d * matrix in kInverses)
	{

		delete matrix;
	}
}




void Simulation::checkCollision()
{
	vector<RigidBody*> rigidbodysToCheck = SimulationManager::getInstance()->getObjectManager().getRigidBodies();
	collidedBoundingVolumes.clear();
	Cube * cubeA;
	Cube* cubeB;
	Sphere *sphereA;
	Sphere *sphereB;
	for each (RigidBody* ridgedBodyA in SimulationManager::getInstance()->getObjectManager().getRigidBodies())
	{
		rigidbodysToCheck.erase(rigidbodysToCheck.begin());

		checkCollisionWithYAxis(ridgedBodyA);

		cubeA = dynamic_cast<Cube*>(ridgedBodyA);
		sphereA = dynamic_cast<Sphere*>(ridgedBodyA);

		for each (RigidBody* ridgedBodyB in rigidbodysToCheck)
		{
			cubeB = dynamic_cast<Cube*>(ridgedBodyB);
			sphereB = dynamic_cast<Sphere*>(ridgedBodyB);

			if (sphereA != 0 || sphereB != 0)
			{
				if (sphereA != 0)
				{
					if (sphereB != 0)
					{
						checkCollision(sphereA, sphereB);
					}
					else
					{
						checkCollision(sphereA, cubeB);
					}
					
				}
				else
				{
					checkCollision(sphereB, cubeA);
				}
			}
			else
			{

				checkCollision(cubeA, cubeB);
			}
			
		}
	}	
}


void Simulation::checkCollision(Sphere* sphereA, Sphere* sphereB)
{

	BoundingVolume *volumeA = sphereA->getVolumeTree()->getRoot()->getBoundingVolume();
	BoundingVolume *volumeB = sphereB->getVolumeTree()->getRoot()->getBoundingVolume();
	if (!volumeA->collisionTest(sphereA->toGlobalSpace(volumeA->m), volumeB, sphereB->toGlobalSpace(volumeB->m)))
	{
		return;
	}

	collisionCalc(sphereA, volumeA, sphereB, volumeB);
}
void Simulation::checkCollision(Sphere* sphere, Cube* cube)
{
	BoundingVolume * sphereVolume = sphere->getVolumeTree()->getRoot()->getBoundingVolume();
	BoundingVolumeTreeNode * cubeNode = cube->getVolumeTree()->getRoot();

	if (!sphereVolume->collisionTest(sphere->toGlobalSpace(sphereVolume->m), cubeNode->getBoundingVolume(), cube->toGlobalSpace(cubeNode->getBoundingVolume()->m)))
	{
		return;
	}

	list<int> state;
	state.push_back(0);

	while (true)
	{
		if (!cubeNode->isLeave())
		{
			bool collide = false;

			// find a child with collition
			BoundingVolumeTreeNode *child;
			while (state.back() < cubeNode->numberOfChildren())
			{
				child = cubeNode->getChild(state.back());
				collide = sphereVolume->collisionTest(sphere->toGlobalSpace(sphereVolume->m), child->getBoundingVolume(), cube->toGlobalSpace(child->getBoundingVolume()->m));
				
				if (collide)
				{
					// go down
					cubeNode = child;
					state.push_back(0);
					break;
				}

				state.back() += 1;
			}

			if (!collide)
			{
				// go up
				cubeNode = cubeNode->getParent();
				state.pop_back();
				if (state.size() > 0)
				{
					state.back() += 1;
				}
				else
				{
					break;
				}
				
			}
		}
		else // node is leave
		{
			// real collision
			collisionCalc(sphere, sphereVolume, cube, cubeNode->getBoundingVolume());
			
			//sphereVolume->collisionCalc(sphere->toGlobalSpace(sphereVolume->m), cubeNode->getBoundingVolume(), cube->toGlobalSpace(cubeNode->getBoundingVolume()->m));
			//collidedBoundingVolumes.push_back(sphereVolume);
			//collidedBoundingVolumes.push_back(cubeNode->getBoundingVolume());

			// go up
			cubeNode = cubeNode->getParent();
			state.pop_back();
			if (state.size() > 0)
			{
				state.back() += 1;
			}
			else
			{
				break;
			}
		}
		
	}
}

void Simulation::checkCollision(Cube* rigidBodyA, Cube* rigidBodyB)
{
	if (!rigidBodyA->getVolumeTree()->getRoot()->getBoundingVolume()->collisionTest(rigidBodyA->toGlobalSpace(rigidBodyA->getVolumeTree()->getRoot()->getBoundingVolume()->m), rigidBodyB->getVolumeTree()->getRoot()->getBoundingVolume(), rigidBodyB->toGlobalSpace(rigidBodyB->getVolumeTree()->getRoot()->getBoundingVolume()->m)))
	{
		return;
	}

	RigidBody* rigidA;
	RigidBody* rigidB;

	const BoundingVolumeTree *treeA;
	const BoundingVolumeTree *treeB;
	if (rigidBodyA->getVolumeTree()->getRoot()->getBoundingVolume()->r < rigidBodyB->getVolumeTree()->getRoot()->getBoundingVolume()->r)
	{
		treeA = rigidBodyA->getVolumeTree();
		treeB = rigidBodyB->getVolumeTree();
		rigidA = rigidBodyA;
		rigidB = rigidBodyB;
	}
	else
	{
		treeA = rigidBodyB->getVolumeTree();
		treeB = rigidBodyA->getVolumeTree();
		rigidA = rigidBodyB;
		rigidB = rigidBodyA;
	}

	list<int> stateA;
	list<int> stateB;
	BoundingVolumeTreeNode * nodeA = treeA->getRoot();
	BoundingVolumeTreeNode * nodeB = treeB->getRoot();

	stateA.push_back(-1);
	stateB.push_back(0);


	while (true)
	{
		if (!nodeB->isLeave())
		{
			bool collide = false;
			if (stateA.back() > -1)
			{
				// check children of node A with child of nodeB
				while (stateA.back() < nodeA->numberOfChildren())
				{
					collide = nodeA->getChild(stateA.back())->getBoundingVolume()->collisionTest(rigidA->toGlobalSpace(nodeA->getChild(stateA.back())->getBoundingVolume()->m), nodeB->getChild(stateB.back())->getBoundingVolume(), rigidB->toGlobalSpace(nodeB->getChild(stateB.back())->getBoundingVolume()->m));
					if (collide)
					{
						// go down
						nodeA = nodeA->getChild(stateA.back());
						nodeB = nodeB->getChild(stateB.back());
						stateA.push_back(-1);
						stateB.push_back(0);
						break;
					}
					stateA.back() += 1;
				}

				if (!collide)
				{
					// switch mode
					stateA.back() = -1;
					stateB.back() += 1;
				}

			}
			else
			{
				// check children of node B with A.
				while (stateB.back() < nodeB->numberOfChildren())
				{
					collide = nodeA->getBoundingVolume()->collisionTest(rigidA->toGlobalSpace(nodeA->getBoundingVolume()->m), nodeB->getChild(stateB.back())->getBoundingVolume(), rigidB->toGlobalSpace(nodeB->getChild(stateB.back())->getBoundingVolume()->m));
					if (collide)
					{
						//switch mode
						stateA.back() = 0;
						break;
					}
					stateB.back() += 1;
				}

				if (!collide)
				{
					// go up
					nodeA = nodeA->getParent();
					nodeB = nodeB->getParent();
					stateA.pop_back();
					stateB.pop_back();

					if (stateA.size() > 0)
					{
						stateA.back() += 1;
						//stateB.back += 1;
					}
					else
					{
						// finnished
						break;
					}
				}
			}
		}
		else // in Leafe
		{
			// collision calc
			collisionCalc(rigidA,nodeA->getBoundingVolume(), rigidB, nodeB->getBoundingVolume());
			
			//nodeA->getBoundingVolume()->collisionCalc(rigidA->toGlobalSpace(nodeA->getBoundingVolume()->m), nodeB->getBoundingVolume(), rigidB->toGlobalSpace(nodeB->getBoundingVolume()->m));

			//collidedBoundingVolumes.push_back(nodeA->getBoundingVolume());
			//collidedBoundingVolumes.push_back(nodeB->getBoundingVolume());

			// go up
			nodeA = nodeA->getParent();
			nodeB = nodeB->getParent();

			stateA.pop_back();
			stateB.pop_back();

			if (stateA.size() > 0)
			{
				stateA.back() += 1;
				//stateB.back() = stateB.back() + 1;
			}
			else
			{
				// finnished
				break;
			}
		}
	}
}

void Simulation::collisionCalc(RigidBody* rigidBodyA, BoundingVolume* volumeA, RigidBody* rigidBodyB, BoundingVolume* volumeB)
{
	volumeA->collisionCalc(rigidBodyA->toGlobalSpace(volumeA->m), volumeB, rigidBodyB->toGlobalSpace(volumeB->m));


	collidedBoundingVolumes.push_back(volumeA);
	collidedBoundingVolumes.push_back(volumeB);

	// relic from the merge with commit  062c64037ef006416605299339dce43287ac9a97 [062c640]
	/*
	vector<RigidBody*> rigidbodysToCheck = SimulationManager::getInstance()->getObjectManager().getRigidBodies();
	collidedBoundingVolumes.clear();
	for each (RigidBody* ridgedBodyA in SimulationManager::getInstance()->getObjectManager().getRigidBodies())
	{
	checkCollisionWithYAxis(ridgedBodyA);

	rigidbodysToCheck.erase(rigidbodysToCheck.begin());
	*/
}

void Simulation::checkCollisionWithYAxis(RigidBody* rigidBody)
{
	// check root collision with Y axis
	BoundingVolumeTreeNode * cubeNode = rigidBody->getVolumeTree()->getRoot();

	if (!rigidBody->getVolumeTree()->getRoot()->getBoundingVolume()->collisionTestYAxis(rigidBody->toGlobalSpace(rigidBody->getVolumeTree()->getRoot()->getBoundingVolume()->m)))
	{
		return;
	}

	list<int> state;
	state.push_back(0);

	
	while (true)
	{
		if (!cubeNode->isLeave())
		{
			bool collide = false;

			// find a child with collition
			BoundingVolumeTreeNode *child;
			while (state.back() < cubeNode->numberOfChildren())
			{
				child = cubeNode->getChild(state.back());
				collide = cubeNode->getChild(state.back())->getBoundingVolume()->collisionTestYAxis(rigidBody->toGlobalSpace(cubeNode->getChild(state.back())->getBoundingVolume()->m));
				if (collide)
				{
					// go down
					cubeNode = child;
					state.push_back(0);
					break;
				}

				state.back() += 1;
			}

			if (!collide)
			{
				// go up
				cubeNode = cubeNode->getParent();
				state.pop_back();
				if (state.size() > 0)
				{
					state.back() += 1;
				}
				else
				{
					break;
				}

			}
		}
		else // node is leave
		{
			// real collision
			cubeNode->getBoundingVolume()->collisionCalcYAxis(rigidBody->toGlobalSpace(cubeNode->getBoundingVolume()->m));
			collidedBoundingVolumes.push_back(cubeNode->getBoundingVolume());

			// go up
			cubeNode = cubeNode->getParent();
			state.pop_back();
			if (state.size() > 0)
			{
				state.back() += 1;
			}
			else
			{
				break;
			}
		}

	}

}
