#pragma once
#include "BoundingVolumeTreeNode.h"

class BoundingVolumeTree
{
protected:
	BoundingVolumeTreeNode* root;

	void deleteTree();
public:
	BoundingVolumeTree();
	~BoundingVolumeTree();

	BoundingVolumeTreeNode* getRoot();
	void setRoot(BoundingVolumeTreeNode* root);

};

