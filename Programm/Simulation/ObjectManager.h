#pragma once
#include "RigidBody.h";
#include <vector>;

/*
Manage the Objects created in the Simualtion.
It will delete all the Objects on destruction.
*/
class ObjectManager
{
private:
	std::vector<RigidBody*> rigidBodies;
public:
	ObjectManager();
	~ObjectManager();
	/*
	Adds an Object to the ObjectManager
	*/
	void addObject(RigidBody *rigidBody);

	/*
	Returns all the Rigidbodys that are currently in the Objectmanger
	*/
	const std::vector<RigidBody*> &getRigidBodies();
		
};

