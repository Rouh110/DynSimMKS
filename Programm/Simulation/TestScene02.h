#pragma once
#include "IScene.h"

/*Test Scene with a jumping jack*/
class TestScene02 :
	public IScene
{
public:
	TestScene02();
	~TestScene02();

	virtual void initializeScene();
};

