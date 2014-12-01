#pragma once
#include "Common/Config.h"
#include <Eigen/Dense>
class BoundingVolume
{
public:
	BoundingVolume(Eigen::Vector3d mRef, double rRef);
	~BoundingVolume();
	Eigen::Vector3d m;
	double r;
	bool collisionTestYAxis();
	bool collisionTest(BoundingVolume* testVolume);
	void collisionCalc(BoundingVolume* testVolume, Eigen::Vector3d contactNormal, Eigen::Vector3d & a, Eigen::Vector3d & b);
	void collisionCalcYAxis(Eigen::Vector3d contactNormal, Eigen::Vector3d & a);
};

