#pragma once
#include "IScene.h"
class ScenePuppet :
	public IScene
{
public:
	ScenePuppet();
	~ScenePuppet();
	void initializeScene();
};

