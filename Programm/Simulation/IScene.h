#pragma once
class IScene
{
public:
	IScene();
	~IScene();

	virtual void initializeScene() = 0;
};

