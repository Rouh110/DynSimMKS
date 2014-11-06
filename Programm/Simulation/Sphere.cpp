#include "Sphere.h"


Sphere::Sphere(Real radius)
: RigidBody(),
radius(radius)
{
	calculateTensor();
}


Sphere::~Sphere()
{
}

Real Sphere::getRadius() const
{
	return this->radius;
}

void mach()
{

}

void Sphere::calculateTensor()
{
	Real value = 2 / 5 * (radius*radius);
	inertiaTensor.x() = value;
	inertiaTensor.y() = value;
	inertiaTensor.z() = value;

	invertedInertiaTensor.x() = 1 / value;
	invertedInertiaTensor.y() = 1 / value;
	invertedInertiaTensor.z() = 1 / value;
}