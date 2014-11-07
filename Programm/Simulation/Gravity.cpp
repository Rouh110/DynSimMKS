#include "Gravity.h"


Gravity::Gravity(const Vector3d & gravity)
{
	setGravity(gravity);
}


Gravity::~Gravity()
{
}

void Gravity::setGravity(const Vector3d & gravity)
{
	 this->gravity = gravity;
}


void Gravity::computeForce(RigidBody *rigidBody, Real time)
{
	rigidBody->addForce(gravity * rigidBody->getMass());
}

Gravity & Gravity::create(const Vector3d  &gravity)
{
	Gravity * m_gravity = new Gravity(gravity);
	m_gravity->addToObjectManager();
	return *m_gravity;
}