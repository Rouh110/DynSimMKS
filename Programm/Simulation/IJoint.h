#pragma once
#include "RigidBody.h"

using namespace Eigen;

/*
Interface for all Joints.
Has some baisc funktionality.
*/
class IJoint
{
protected:
	RigidBody * rigidBodyA;
	RigidBody * rigidBodyB;

	Vector3d pointA;
	Vector3d pointB;

public:
	IJoint();
	~IJoint();

	/*
	Set the rigidBodys for the Joint and the Position of the Joint.
	*/
	virtual void setJointPoints(RigidBody * rigidBodyA, RigidBody *rigidBodyB, const Vector3d & jointPoint);

	/*
	Writes the current error in to the parameter out_Error.
	*/
	virtual void getCurrentError(Vector3d & out_Error) = 0;

	/*
	Renders the Joint. Override if the Joints has a grafikal representation.
	*/
	virtual void render(){}

protected:
	/*
	returns the point A in global space.
	*/
	virtual void getGlobalPointA(Vector3d & out_pointA);

	/*
	Returns the point B in global space.
	*/
	virtual void getGlobalPointB(Vector3d & out_pointB);

	/*
	Add this Joint to the ObjectManager.
	*/
	virtual void addToObjectManager();
};

