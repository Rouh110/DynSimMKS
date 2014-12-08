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

bool BoundingVolume::collisionTest(BoundingVolume* testVolume){
	double x = this->m.x() - testVolume->m.x();
	double y = this->m.y() - testVolume->m.y();
	double z = this->m.z() - testVolume->m.z();
	double tmp = pow(x, 2) + pow(y, 2) + pow(z, 2);
	if(tmp <= pow(this->r+testVolume->r,2)){
		return true;
	}
	return false;
}

void BoundingVolume::collisionCalc(BoundingVolume* testVolume, Eigen::Vector3d contactNormal, Eigen::Vector3d & a, Eigen::Vector3d & b){
	double x = this->m.x() - testVolume->m.x();
	double y = this->m.y() - testVolume->m.y();
	double z = this->m.z() - testVolume->m.z();
	double tmp = std::sqrt(pow(x,2) + pow(y,2) + pow(z,2));
	
	contactNormal = (this->m - testVolume->m)/tmp;
	a = this->m - this->r * contactNormal;
	b = testVolume->m - testVolume->r * contactNormal;
}
bool BoundingVolume::collisionTestYAxis(){
	if ((this->m.y() - this->r) <= 0){
		return true;
	}
	return false;
}
void BoundingVolume::collisionCalcYAxis(Eigen::Vector3d  & contactNormal, Eigen::Vector3d & a){
	contactNormal = Eigen::Vector3d(0, 1, 0);
	a = this->m - this->r * contactNormal;
}