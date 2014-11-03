#include "Cube.h"


Cube::Cube(Real width, Real height, Real depth)
: RigidBody(),
width(width),
height(height),
depth(depth)
{
	calculateTensor();
}


Cube::~Cube()
{
}

Real Cube::getWidth()
{
	return width;
}

Real Cube::getHeight()
{
	
	return height;
}

Real Cube::getDepth()
{
	return depth;
}


void Cube::calculateTensor()
{
	inertiaTensor.x() = (height*height + depth*depth) / 12;
	inertiaTensor.y() = (width*width + depth*depth) / 12;
	inertiaTensor.z() = (width*width + height*height) / 12;

	invertedInertiaTensor.x() = 1/inertiaTensor.x();
	invertedInertiaTensor.y() = 1 / inertiaTensor.y();
	invertedInertiaTensor.z() = 1 / inertiaTensor.z();

}