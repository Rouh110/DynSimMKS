#pragma once
#include "Common/Config.h"
#include <Eigen/Dense>
#include <vector>
#include <list>

using namespace Eigen;
using namespace std;

class BoundingVolume
{
public:
	BoundingVolume(Vector3d mRef, double rRef);
	~BoundingVolume();
	Vector3d m;
	double r;
	list<Vector3d> contactNormals;
	list<Vector3d> contactPoints;
	//Vector3d contactPoint;
	//Vector3d contactNormal;
	bool collisionTestYAxis(const Vector3d &globalPosition);
	bool collisionTest(const Vector3d &globalPositionA, BoundingVolume* testVolume, const Vector3d &globalPositionB);
	void collisionCalc(const Vector3d &globalPositionA, BoundingVolume* testVolume, const Vector3d &globalPositionB);
	void collisionCalcBrianMitrich(const Vector3d &globalPositionA, const Vector3d &relativeVelocityA, const Vector3d &globalPositionB, const Vector3d &relativeVelocityB);
	void collisionCalcYAxis(const Vector3d &globalPosition);
};

