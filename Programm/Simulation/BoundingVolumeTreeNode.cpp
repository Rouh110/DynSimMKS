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
		delete boundingVolume;
}

bool BoundingVolumeTreeNode::isLeave()
{
	return children.size() == 0;
}

bool BoundingVolumeTreeNode::isRoot()
{
	return parent == 0;
}

BoundingVolume * BoundingVolumeTreeNode::getBoundingVolume()
{
	return boundingVolume;
}
BoundingVolumeTreeNode* BoundingVolumeTreeNode::getParent()
{
	return parent;
}

BoundingVolumeTreeNode* BoundingVolumeTreeNode::getChild(int i)
{
	return children[i];
}

int BoundingVolumeTreeNode::numberOfChildren()
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
