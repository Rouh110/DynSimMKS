#pragma once

#include "Simulation/RigidBody.h"
#include "Common/Config.h"
#include "Visualization/MiniGL.h"
#include "GL/glut.h"
/*Represents a sphere.*/
class Sphere : public  RigidBody
{
private:
	Real radius;
public:
	
	Sphere(Real radius);
	~Sphere();

	/*Returns the radius of the sphere.*/
	Real getRadius() const;

	/*Creates a sphere with the given radius and adds it to the ObjectManager.
	Returns a Referenze to the created sphere.*/
	static Sphere & create(Real radius = 1);
protected:
	/*calculates and sets the inertia and inverted inertia tensor.*/
	void calculateTensor();
	void initializeVolumeTree();
};

