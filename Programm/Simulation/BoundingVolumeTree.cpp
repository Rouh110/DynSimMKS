#include "BoundingVolumeTree.h"
#include <list>

BoundingVolumeTree::BoundingVolumeTree()
	: root(0)
{
}


BoundingVolumeTree::~BoundingVolumeTree()
{
	deleteTree();
}

void BoundingVolumeTree::deleteTree()
{
	if (root != 0)
	{
		list<int> state;
		BoundingVolumeTreeNode* node = root;
		state.push_back(0);

		while (state.size() > 0)
		{
			if (state.back() < node->numberOfChildren())
			{
				node = node->getChild(state.back());
				state.push_back(0);
			}
			else
			{
				state.pop_back();
				BoundingVolumeTreeNode* tmpNode = node;
				node = node->getParent();
				delete tmpNode;

				if (state.size() > 0)
				{
					state.back() = state.back() + 1;
				}
			}
		}
	}
	

	root = 0;
}

BoundingVolumeTreeNode* BoundingVolumeTree::getRoot() const 
{
	return root;
}

void BoundingVolumeTree::setRoot(BoundingVolumeTreeNode* root)
{
	this->root = root;
}