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