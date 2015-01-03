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

void BoundingVolume::collisionCalcBrianMitrich(const Eigen::Vector3d &globalPositionA, const Vector3d &relativeVelocityA, const Vector3d &globalPositionB, const Eigen::Vector3d &relativeVelocityB){
	/*
	Brian Mitrich Collision
	*/

	// Calculate Normal: From Body B to Body A
	double x = globalPositionB.x() - globalPositionA.x();
	double y = globalPositionB.y() - globalPositionA.y();
	double z = globalPositionB.z() - globalPositionA.z();
	double tmp = std::sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
	Eigen::Vector3d contactNormal = (globalPositionB - globalPositionA) / tmp;

	// Relative Velocity
	Eigen::Vector3d uRel = relativeVelocityA - relativeVelocityB;
	//Eigen::Vector3d uRelN = (uRel[0]*contactNormal[0] + uRel[1]*contactNormal[1] + uRel[2]*contactNormal[2]) * contactNormal;
	double uDotRelN = (uRel[0] * contactNormal[0] + uRel[1] * contactNormal[1] + uRel[2] * contactNormal[2]);
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
			// TODO
		}
		// Kollision
		else if (uDotRelN <= -episolon)
		{
			// TODO
		}
		// kein Kontakt
		else
		{
			// TODO
		}
	}

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