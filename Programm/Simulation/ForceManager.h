#pragma once
#include "RigidBody.h";
#include <vector>;

class ForceManager
{
private: 
	std::vector<RigidBody*> forcedBodies;
public:
	ForceManager();
	~ForceManager();
	const std::vector<RigidBody*> &getForceBodies();
	void addObject(RigidBody *forcedBody);
};

