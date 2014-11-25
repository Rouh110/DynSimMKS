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

	/*Returns the ras vector in global space*/
	virtual void getRas(Vector3d & out_rsa);

	/*Return the rbs vector in global space*/
	virtual void getRbs(Vector3d & out_rsb);

	virtual void getLocalPointA(Vector3d & out_point);

	virtual void getLocalPointB(Vector3d & out_point);

	/*
	returns the point A in global space.
	*/
	virtual void getGlobalPointA(Vector3d & out_pointA);

	/*
	Returns the point B in global space.
	*/
	virtual void getGlobalPointB(Vector3d & out_pointB);
	
	/*returns the rigid body A*/
	RigidBody * getRigidBodyA();

	/*returns the rigid body B*/
	RigidBody * getRigidBodyB();

	/*
	Renders the Joint. Override if the Joints has a grafikal representation.
	*/
	virtual void render(){}

protected:
	
	/*
	Add this Joint to the ObjectManager.
	*/
	virtual void addToObjectManager();

};

