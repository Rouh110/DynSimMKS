#pragma once
#include "RigidBody.h"
#include "Simulation\IForce.h"
#include <vector>

/*
Manage the Objects created in the Simualtion.
It will delete all the Objects on destruction.
*/
class ObjectManager
{
private:
	IForce *f;
	std::vector<RigidBody*> rigidBodies;
	std::vector<IForce*> forces;
public:
	ObjectManager();
	~ObjectManager();
	/*
	Adds an Object to the ObjectManager
	Don't add an Object, while using the lists.
	*/
	void addObject(RigidBody *rigidBody);
	void addObject(IForce *force);

	/*
	Returns all the Rigidbodys that are currently in the Objectmanger
	*/
	const std::vector<RigidBody*> &getRigidBodies() const;

	/*
	Returns all Forces thar are currently in the Objectmanager
	*/
	const std::vector<IForce*> &getForces() const;

	/*
	Deletes all the Object in the Objectmanager and clears all the lists.
	Don't reset the ObjetcManager while  using the lists
	*/
	void resetObjectManager();
		
};

