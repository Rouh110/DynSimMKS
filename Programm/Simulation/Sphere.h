#pragma once

#include "Simulation/RigidBody.h"
#include "Common/Config.h"

class Sphere : public  RigidBody
{
private:
	Real radius;
public:
	Sphere(Real radius);
	~Sphere();

	Real getRadius() const;

	static Sphere & create(Real radius = 1);
protected:
	void calculateTensor();
};

