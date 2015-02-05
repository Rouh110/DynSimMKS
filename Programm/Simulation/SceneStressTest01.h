#pragma once
#include "IScene.h"
/*Simple Test Scene with two cuboits and two strings*/
class SceneStressTest01 :
	public IScene
{
public:
	SceneStressTest01();
	~SceneStressTest01();
	
	virtual void initializeScene();
};

