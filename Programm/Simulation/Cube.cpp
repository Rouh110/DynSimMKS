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
	Real sqrHeight = height*height;
	Real sqrWidth =  width*width;
	Real sqrDepth = depth*depth;
	Real m11 = (sqrHeight + sqrDepth) / 12;
	Real m22 = (sqrWidth + sqrDepth) / 12;
	Real m33 = (sqrHeight + sqrWidth) / 12;

	inertiaTensor.x() = m11;
	inertiaTensor.y() = m22;
	inertiaTensor.z() = m33;
	
	invertedInertiaTensor.x() = 1/m11;
	invertedInertiaTensor.y() = 1/m22;
	invertedInertiaTensor.z() = 1/m33;
	

}

Cube & Cube::create(Real width , Real height , Real depth )
{
	Cube * cube = new Cube(width, height, depth);
	cube->addToObjectManager();
	return *cube;
}