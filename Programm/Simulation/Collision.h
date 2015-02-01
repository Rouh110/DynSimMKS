#pragma once
#include "RigidBody.h"
#include "BoundingVolume.h"
#include "Common/Config.h"
#include <Eigen/Dense>
#include <iostream>
class Collision
{
public:
	Collision::Collision(RigidBody* rigidA, RigidBody* rigidB, BoundingVolume collidedVolumeA, BoundingVolume collidedVolumeB, Vector3d contactNormalA, Vector3d contactNormalB, Vector3d contactPointA, Vector3d contactPointB, double constNormalA, double constNormalB);
	~Collision();

	RigidBody* rigidA;
	RigidBody* rigidB;
	BoundingVolume collidedVolumeA;
	BoundingVolume collidedVolumeB;
	Vector3d contactNormalA;
	Vector3d contactNormalB;
	Vector3d contactPointA;
	Vector3d contactPointB;
	Vector3d lastImpulseA;
	Vector3d lastImpulseB;
	bool contact;
	double constNormalA;
	double constNormalB;
	Vector3d uRelCA;
	Vector3d uRelNA;
	Vector3d uRelCB;
	Vector3d uRelNB;
	Vector3d relativeA;
	Vector3d relativeB;
	int collisionOrNoCollisionA;
	int collisionOrNoCollisionB;
	//void calcURelC(const Vector3d &relativeVelocityA, const Vector3d &normalA, const Eigen::Vector3d &relativeVelocityB, const double &elasticityA, const double &elasticityB, Vector3d &result);
	void calcURelN(const Vector3d &relativeVelocityA, const Vector3d &normalA, const Eigen::Vector3d &relativeVelocityB, Vector3d &result);
	void collisionOrNotCollision(const Vector3d &relativeVelocityA, const Vector3d &normalA, const Eigen::Vector3d &relativeVelocityB, int &result);
};


