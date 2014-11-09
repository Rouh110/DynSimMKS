#pragma once
#include <Eigen/Dense>
#include "GlobalForce.h"

using namespace Eigen;

#define DEFAULT_GRAVITY_VECTOR Vector3d(0, -9.81, 0)

/*
Represents the gravity in the scene.
The Default gravty vector is (0, -9.81, 0).
The force the gravity creates will be multyplied by the mass of each RigidBody, so the gravity accelerates all RigidBodys identicaly.
*/
class Gravity :
	public GlobalForce
{
private:
	Vector3d gravity;
 
public:
	Gravity(const Vector3d  &gravity = DEFAULT_GRAVITY_VECTOR);
	~Gravity();

	/*
	Sets the direction and strength of the gravity. The lenght of the given Vector will be the strength.
	*/
	void setGravity(const Vector3d & gravity);

protected:
	void computeForce(RigidBody *rigidBody, Real time);

public:
	/*Creates a Gravity Class with the given values and adds it to the scene.
	Returns a Referenze to the createt Gravity.*/
	static Gravity & create(const Vector3d  &gravity = DEFAULT_GRAVITY_VECTOR);

};

