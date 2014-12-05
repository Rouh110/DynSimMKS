#include "BoundingVolumeTree.h"


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
		vector<int> state;
		BoundingVolumeTreeNode* node = root;
		state.push_back(0);

		while (state.size() != 0)
		{
			if (state[state.size() - 1] < node->numberOfChildren())
			{
				state.push_back(0);
				node = node->getChild(state[state.size() - 1]);
			}
			else
			{
				state.pop_back();
				if (state.size() > 0)
				{
					state[state.size() - 1] = state[state.size() - 1] + 1;
					node = node->getParent();
				}
				BoundingVolumeTreeNode* tmpNode = node;
				delete tmpNode;
			}
		}
	}
	

	root = 0;
}

BoundingVolumeTreeNode* BoundingVolumeTree::getRoot()
{
	return root;
}

void BoundingVolumeTree::setRoot(BoundingVolumeTreeNode* root)
{
	this->root = root;
}