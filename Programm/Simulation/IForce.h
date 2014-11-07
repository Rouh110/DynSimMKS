#pragma once

#include "Common\Config.h"

class IForce
{
public:
	IForce();
	~IForce();

	virtual void computeForce(Real time) = 0;

	virtual void render();

protected:
	void addToObjectManager();
};

