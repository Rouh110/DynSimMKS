#pragma once
#include "IScene.h"
/*Simple Test Scene with two cuboits and two strings*/
class TestScene01 :
	public IScene
{
public:
	TestScene01();
	~TestScene01();

	virtual void initializeScene();
};

