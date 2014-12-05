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

	
	bool isLeave() const;
	bool isRoot() const;

	BoundingVolume * getBoundingVolume() const;
	BoundingVolumeTreeNode* getParent() const;
	BoundingVolumeTreeNode* getChild(int i) const;
	int numberOfChildren() const;

	void addChild(BoundingVolumeTreeNode*);
	void setParent(BoundingVolumeTreeNode*);
	void setBoundingVolume(BoundingVolume* boundingVolume);



};

