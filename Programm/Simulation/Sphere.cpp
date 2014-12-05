#include "Sphere.h"


Sphere::Sphere(Real radius)
:
radius(radius)
{
	calculateTensor();
	initializeVolumeTree();
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
	Real value = 2.0 / 5.0 * (radius*radius);
	inertiaTensor.x() = value;
	inertiaTensor.y() = value;
	inertiaTensor.z() = value;

	invertedInertiaTensor.x() = 1.0 / value;
	invertedInertiaTensor.y() = 1.0 / value;
	invertedInertiaTensor.z() = 1.0 / value;
}

Sphere & Sphere::create(Real radius)
{
	Sphere * sphere = new Sphere(radius);
	sphere->addToObjectManager();
	return *sphere;
}

void Sphere::initializeVolumeTree()
{
	BoundingVolumeTreeNode * root = new BoundingVolumeTreeNode();
	volumeTree.setRoot(root);

	root->setBoundingVolume(new BoundingVolume(Eigen::Vector3d(0,0,0),radius));
}