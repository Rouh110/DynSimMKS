#pragma once
#include <Eigen/Dense>
#include "GlobalForce.h"

using namespace Eigen;

#define DEFAULT_GRAVITY_VECTOR Vector3d(0, -9.81, 0)

class Gravity :
	public GlobalForce
{
private:
	Vector3d gravity;
 
public:
	Gravity(const Vector3d  &gravity = DEFAULT_GRAVITY_VECTOR);
	~Gravity();

	void setGravity(const Vector3d & gravity);

protected:
	void computeForce(RigidBody *rigidBody, Real time);

public:
	static Gravity & create(const Vector3d  &gravity = DEFAULT_GRAVITY_VECTOR);

};

