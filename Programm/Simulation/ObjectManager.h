#pragma once
#include "RigidBody.h";
#include <vector>;

class ObjectManager
{
private:
	std::vector<RigidBody*> rigidBodies;
public:
	ObjectManager();
	~ObjectManager();
	void addObject(RigidBody *rigidBody);
	const std::vector<RigidBody*> &getRigidBodies();
		
};

