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
	Eigen::Vector3d contactPoint;
	Eigen::Vector3d contactNormal;
	bool collisionTestYAxis();
	bool collisionTest(BoundingVolume* testVolume);
	void collisionCalc(BoundingVolume* testVolume);
	void collisionCalcYAxis();
};

