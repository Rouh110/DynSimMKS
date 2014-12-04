#pragma once
#include <vector>
#include "BoundingVolume.h"
using namespace std;
class BoundingVolumeTreeNode
{
protected:
	BoundingVolumeTreeNode* parent;
	vector<BoundingVolumeTreeNode*> children;

	BoundingVolume * boundingVolume;

public:
	BoundingVolumeTreeNode();
	~BoundingVolumeTreeNode();

	
	bool isLeave();
	bool isRoot();

	BoundingVolume * getBoundingVolume();
	BoundingVolumeTreeNode* getParent();
	BoundingVolumeTreeNode* getChild(int i);
	int numberOfChildren();

	void addChild(BoundingVolumeTreeNode*);
	void setParent(BoundingVolumeTreeNode*);



};

