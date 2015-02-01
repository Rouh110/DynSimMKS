#include "Collision.h"



Collision::Collision(RigidBody* rigidA, RigidBody* rigidB, BoundingVolume collidedVolumeA, BoundingVolume collidedVolumeB, Vector3d contactNormalA, Vector3d contactNormalB, Vector3d contactPointA, Vector3d contactPointB, double constNormalA , double constNormalB )
{
	this->collidedVolumeA = collidedVolumeA;
	this->collidedVolumeB = collidedVolumeB;
	this->rigidA = rigidA;
	this->rigidB = rigidB;
	this->contactNormalA = contactNormalA;
	this->contactNormalB = contactNormalB;
	this->contactPointA = contactPointA;
	this->contactPointB = contactPointB;
	this->constNormalA = constNormalA;
	this->constNormalB = constNormalB;

	rigidA->getVelocityOfGlobalPoint(rigidA->toGlobalSpace(contactPointA), this->relativeA);
	rigidB->getVelocityOfGlobalPoint(rigidB->toGlobalSpace(contactPointB), this->relativeB);

	calcURelN(relativeA,contactNormalA, relativeB,this->uRelNA);
	calcURelN(relativeB, contactNormalB, relativeB, this->uRelNB);

	this->uRelCA = (-(rigidA->getElasticity()*rigidB->getElasticity()))*this->uRelNA;
	this->uRelCB = (-(rigidA->getElasticity()*rigidB->getElasticity()))*this->uRelNB;
	collisionOrNotCollision(relativeA, contactNormalA, relativeB, collisionOrNoCollisionA);
	collisionOrNotCollision(relativeB, contactNormalB, relativeA, collisionOrNoCollisionB);
}


Collision::~Collision()
{
}


void Collision::calcURelN(const Vector3d &relativeVelocityA, const Vector3d &normalA, const Eigen::Vector3d &relativeVelocityB, Vector3d &result){
	Eigen::Vector3d uRel = relativeVelocityA - relativeVelocityB;
	result = (uRel.dot(normalA)*normalA);
}
void Collision::collisionOrNotCollision(const Vector3d &relativeVelocityA, const Vector3d &normalA, const Eigen::Vector3d &relativeVelocityB, int &result){
	Eigen::Vector3d uRel = relativeVelocityA - relativeVelocityB;
	double epsiolon = std::sqrt(2 * 9.81 * 0.01);
	if (-epsiolon < uRel.dot(normalA) && uRel.dot(normalA) < epsiolon)
	{
		result = 0;
	}
	else if(uRel.dot(normalA) < -epsiolon){
		result = 1;
	}
	else{
		result = 2;
	}
}