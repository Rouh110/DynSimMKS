#include "Simulation.h"
#include"Simulation\TimeManager.h"
#include "SimulationManager.h"

using namespace IBDS;

Simulation::Simulation()
: approximationMethod(EXPLICIT_EULER),
currentScene(TASK1),
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


void Simulation::setCurrentScene(CurrentScene scene)
{
	currentScene = scene;
}

Simulation::CurrentScene Simulation::getCurrentScene()
{
	return currentScene;
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

	simulateJointsPredictorCorrector(h);
	

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

	computeAllForces(time);

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




void Simulation::simulateJointsPredictorCorrector(Real h)
{
	Real damper = (1.0 - 0.00001);
	Real maxError = 0;
	Real norm;
	Vector3d error;
	Matrix3d *inverseK;
	Vector3d impulse;
	RigidBody* rigidBodyA;
	RigidBody* rigidBodyB;
	Vector3d tmp;

	Real delta = 0.000001;
	std::vector<Matrix3d *> inverseKs;
	bool first = true;
	
	Real factor = 1.0 / h;

	Matrix3d Kaa;
	Matrix3d Kbb;
	Quaterniond tmpQuaternion;

	int n = 0;
	int i;
	do
	{
		maxError = 0.0;
		i = 0;
		for each (IJoint* joint in SimulationManager::getInstance()->getObjectManager().getJoints())
		{
			rigidBodyA = joint->getRigidBodyA();
			rigidBodyB = joint->getRigidBodyB();

			if (first)
			{
				
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
				

				(*inverseK) = (Kaa +Kbb).inverse();

				inverseKs.push_back(inverseK);
				first = false;
			}
			else
			{
				inverseK = inverseKs[i];
			}

			joint->getCurrentError(error);

			norm = std::abs(error.norm());
			
			printf("%f, %f\n", maxError, norm);
			if (maxError < norm)
			{
				maxError = norm;
			}
				

			if (norm <= delta)
				continue;

			// calculate impulse
			impulse = (*inverseK)* factor * error;
	
			if (!rigidBodyA->isStatic())
			{
				joint->getRas(tmp);
				rigidBodyA->addRasImpuls(impulse*1, tmp);
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
			

			i++;

		}

		//prediktor

		for each (RigidBody * rigidBody in SimulationManager::getInstance()->getObjectManager().getRigidBodies())
		{
			if (rigidBody->isStatic())
			{
				continue;
			}

			// calculate X
			calculateXdot(rigidBody, tmp);
			rigidBody->setPosition(rigidBody->getPosition() + h* tmp);

			// calculate q
			calculateQdot(rigidBody, tmpQuaternion);
			scaleQuaternion(h, tmpQuaternion, tmpQuaternion);
			addQuaternions(rigidBody->getRotation(), tmpQuaternion, tmpQuaternion);
			rigidBody->setRotation(tmpQuaternion.normalized());
		}
		
		if (maxError > delta)
			printf("jo\n");
		else
			printf("%f\n", maxError);
		n++;
	} while (n < 10);//maxError > delta);

	// clean up
	for each (Matrix3d * matrix in inverseKs)
	{

		delete matrix;
	}

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


