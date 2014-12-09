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
	bool collisionTestYAxis(const Eigen::Vector3d &globalPosition);
	bool collisionTest(const Eigen::Vector3d &globalPositionA, BoundingVolume* testVolume, const Eigen::Vector3d &globalPositionB);
	void collisionCalc(const Eigen::Vector3d &globalPositionA, BoundingVolume* testVolume, const Eigen::Vector3d &globalPositionB);
	void collisionCalcYAxis(const Eigen::Vector3d &globalPosition);
};

