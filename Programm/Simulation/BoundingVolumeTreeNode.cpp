#include "BoundingVolumeTreeNode.h"


BoundingVolumeTreeNode::BoundingVolumeTreeNode()
	:
	parent(0),
	boundingVolume(0)
{
}


BoundingVolumeTreeNode::~BoundingVolumeTreeNode()
{
	if (boundingVolume != 0)
	{
		delete boundingVolume;
	}
		

	boundingVolume = 0;
}

bool BoundingVolumeTreeNode::isLeave() const
{
	return children.size() == 0;
}

bool BoundingVolumeTreeNode::isRoot() const
{
	return parent == 0;
}

BoundingVolume * BoundingVolumeTreeNode::getBoundingVolume() const
{
	return boundingVolume;
}
BoundingVolumeTreeNode* BoundingVolumeTreeNode::getParent() const
{
	return parent;
}

BoundingVolumeTreeNode* BoundingVolumeTreeNode::getChild(int i) const
{
	return children[i];
}

int BoundingVolumeTreeNode::numberOfChildren() const
{
	return children.size();
}

void BoundingVolumeTreeNode::addChild(BoundingVolumeTreeNode* child)
{
	children.push_back(child);
	child->setParent(this);
}

void BoundingVolumeTreeNode::setParent(BoundingVolumeTreeNode* parent)
{
	this->parent = parent;
}

void BoundingVolumeTreeNode::setBoundingVolume(BoundingVolume* boundingVolume)
{
	this->boundingVolume = boundingVolume;
}
