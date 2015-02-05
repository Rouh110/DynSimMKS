#pragma once
#include "IScene.h"
/*Simple Test Scene with two cuboits and two strings*/
class SceneStressTest02 :
	public IScene
{
public:
	SceneStressTest02();
	~SceneStressTest02();
	
	virtual void initializeScene();
};

