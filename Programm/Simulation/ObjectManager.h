#pragma once
#include "RigidBody.h"
#include "Simulation\IForce.h"
#include "Simulation\IJoint.h"
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
	std::vector<IJoint*> joints;

public:
	ObjectManager();
	~ObjectManager();
	/*
	Adds an Object to the ObjectManager
	Don't add an Object, while using the lists.
	*/
	void addObject(RigidBody *rigidBody);
	void addObject(IForce *force);
	void addObject(IJoint *joint);

	/*
	Returns all the Rigidbodys that are currently in the Objectmanger
	*/
	const std::vector<RigidBody*> &getRigidBodies() const;

	/*
	Returns all Forces that are currently in the Objectmanager
	*/
	const std::vector<IForce*> &getForces() const;

	/*
	Return all Joints that are currently in the Object Manager
	*/
	const std::vector<IJoint*> &getJoints() const;

	/*
	Deletes all the Object in the Objectmanager and clears all the lists.
	Don't reset the ObjetcManager while  using the lists
	*/
	void resetObjectManager();
		
};

