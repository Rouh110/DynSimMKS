#include "BoundingVolume.h"
#include <math.h>

BoundingVolume::BoundingVolume(Eigen::Vector3d mRef, double rRef)
{
	m = mRef;
	r = rRef;
}


BoundingVolume::~BoundingVolume()
{
}

bool BoundingVolume::collisionTest(const Eigen::Vector3d &globalPositionA, BoundingVolume* testVolume, const Eigen::Vector3d &globalPositionB){
	double x = globalPositionA.x() - globalPositionB.x();
	double y = globalPositionA.y() - globalPositionB.y();
	double z = globalPositionA.z() - globalPositionB.z();
	double tmp = pow(x, 2) + pow(y, 2) + pow(z, 2);
	if(tmp <= pow(this->r+testVolume->r,2)){
		return true;
	}
	return false;
}

void BoundingVolume::collisionCalc(const Eigen::Vector3d &globalPositionA, BoundingVolume* testVolume, const Eigen::Vector3d &globalPositionB){
	double x = globalPositionA.x() - globalPositionB.x();
	double y = globalPositionA.y() - globalPositionB.y();
	double z = globalPositionA.z() - globalPositionB.z();
	double tmp = std::sqrt(pow(x,2) + pow(y,2) + pow(z,2));
	
	contactNormals.push_back((globalPositionA - globalPositionB) / tmp);
	contactPoints.push_back( globalPositionA - this->r * contactNormals.back());
	testVolume->contactPoints.push_back(globalPositionB - testVolume->r * contactNormals.back()*-1);
	testVolume->contactNormals.push_back(contactNormals.back() *-1);
}

void BoundingVolume::collisionCalcBrianMitrich(const Eigen::Vector3d &globalPositionA, const Vector3d &relativeVelocityA, const Vector3d &globalPositionB, const Eigen::Vector3d &relativeVelocityB, const Matrix3d &kAA, const Matrix3d &kBB, const double &elasticityA, const double &elasticityB, Vector3d &imp){
	/*
	Brian Mitrich Collision
	*/

	// Calculate Normal: From Body B to Body A
	double x = globalPositionB.x() - globalPositionA.x();
	double y = globalPositionB.y() - globalPositionA.y();
	double z = globalPositionB.z() - globalPositionA.z();
	double tmp = std::sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
	//Eigen::Vector3d contactNormal = (globalPositionB - globalPositionA) / tmp;
	Eigen::Vector3d contactNormal = this->contactNormals.back();

	// Relative Velocity
	Eigen::Vector3d uRel = relativeVelocityA - relativeVelocityB;
	//Eigen::Vector3d uRelN = (uRel[0]*contactNormal[0] + uRel[1]*contactNormal[1] + uRel[2]*contactNormal[2]) * contactNormal;
	//double uDotRelN = (uRel[0] * contactNormal[0] + uRel[1] * contactNormal[1] + uRel[2] * contactNormal[2]);
	double uDotRelN = ((uRel.dot(contactNormal)));
	
	Eigen::Vector3d uRelN = uDotRelN * contactNormal;

	// Collision check
	//double uRelNNorm = std::sqrt(pow(uRelN[0], 2) + pow(uRelN[1], 2) + pow(uRelN[2], 2)); //?
	double episolon = std::sqrt(2 * 9.81 * 0.01);

	// Kollision
	if (uDotRelN < 0)
	{
		// bleibender Kontakt
		if (-episolon < uDotRelN && uDotRelN < episolon)
		{
			printf("kontact\n");
			Vector3d result;
			contactSolutionImpulse(kAA, kBB, uRelN, result, globalPositionA, relativeVelocityA, globalPositionB, relativeVelocityB, contactNormal);
			imp = result;
		}
		// Kollision
		else if (uDotRelN <= -episolon)
		{
			printf("colision\n");
			Vector3d result;
			collisionSolutionImpulse(kAA, kBB, uRelN, elasticityA, elasticityB ,result);
			imp = result;
		}
		// kein Kontakt
		else
		{
			// TODO
		}
	}

}
void BoundingVolume::contactSolutionImpulse(const Matrix3d& kaa, const Matrix3d& kbb, Vector3d& urel, Vector3d & result, const Vector3d &globalPositionA, const Vector3d &relativeVelocityA, const  Vector3d &globalPositionB, const  Vector3d &relativeVelocityB, Vector3d &n_t0){
	
	// get timestep
	float timeStep = IBDS::TimeManager::getCurrent()->getTimeStepSize();
	// euler schritt
	Vector3d a_th = globalPositionA + timeStep*relativeVelocityA;
	Vector3d b_th = globalPositionB + timeStep*relativeVelocityB;

	double x = globalPositionB.x() - globalPositionA.x();
	double y = globalPositionB.y() - globalPositionA.y();
	double z = globalPositionB.z() - globalPositionA.z();
	double tmp = std::sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
	Vector3d n_th = (globalPositionB - globalPositionA) / tmp;
	 
	// bestimmung der eindrungstiefe
	Vector3d newPos = (b_th - a_th);
	double dn_th = (newPos[0] * n_th[0] + newPos[1] * n_th[1] + newPos[2] * n_th[2]);
	// berechnung deltaURelN
	Vector3d deltaURelN = 1 / timeStep * dn_th * n_t0;

	// Impuls in normalenrichtung
	result = (1 / (this->contactNormals.back().transpose() * (kaa + kbb) * this->contactNormals.back()))*deltaURelN;
}
void BoundingVolume::collisionSolutionImpulse(const Matrix3d& kaa, const Matrix3d& kbb, Vector3d& urel,const double &elasticityA,const double&elasticityB, Vector3d & result){
	Vector3d deltaURelN = (-(elasticityA*elasticityB) * urel) - urel;
	result = (1 / (this->contactNormals.back().transpose() * (kaa + kbb) * this->contactNormals.back()))*deltaURelN;
}
bool BoundingVolume::collisionTestYAxis(const Eigen::Vector3d &globalPosition){
	if ((globalPosition.y() - this->r) <= 0){
		return true;
	}
	return false;
}

void BoundingVolume::collisionCalcYAxis(const Eigen::Vector3d &globalPosition){
	contactNormals.push_back(Vector3d(0, 1, 0));
	contactPoints.push_back(globalPosition - this->r * contactNormals.back());
}