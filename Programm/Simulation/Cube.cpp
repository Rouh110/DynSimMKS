#include "Cube.h"


Cube::Cube(Real width, Real height, Real depth)
: RigidBody(),
width(width),
height(height),
depth(depth)
{
	calculateTensor();
	initializeVolumeTree();
}


Cube::~Cube()
{
}

Real Cube::getWidth()
{
	return width;
}

Real Cube::getHeight()
{
	
	return height;
}

Real Cube::getDepth()
{
	return depth;
}


void Cube::calculateTensor()
{
	Real sqrHeight = height*height;
	Real sqrWidth =  width*width;
	Real sqrDepth = depth*depth;
	Real m11 = (sqrHeight + sqrDepth) / 12;
	Real m22 = (sqrWidth + sqrDepth) / 12;
	Real m33 = (sqrHeight + sqrWidth) / 12;

	inertiaTensor.x() = m11;
	inertiaTensor.y() = m22;
	inertiaTensor.z() = m33;
	
	invertedInertiaTensor.x() = 1/m11;
	invertedInertiaTensor.y() = 1/m22;
	invertedInertiaTensor.z() = 1/m33;
	

}

struct Plane
{
	Eigen::Vector3d pos1;
	Eigen::Vector3d pos2;
	Eigen::Vector3d pos3;
	Eigen::Vector3d pos4;
};
#include <list>
BoundingVolumeTreeNode* Cube::generateSubtree(Eigen::Vector3d pos1, Eigen::Vector3d pos2, Eigen::Vector3d pos3, Eigen::Vector3d pos4)
{
	Real currentDepth = 1;
	Real width;
	Real height;
	Eigen::Vector3d m;
	Real radius;
	BoundingVolumeTreeNode* node;
	BoundingVolumeTreeNode* root;

	Plane plane;
	plane.pos1 = pos1;
	plane.pos2 = pos2;
	plane.pos3 = pos3;
	plane.pos4 = pos4;

	list<int> state;
	list<Plane> planes;

	planes.push_back(plane);


	m = (pos1 + pos2 + pos3 + pos4) / 4;
	radius = (pos1 - m).norm();
	root = new BoundingVolumeTreeNode();
	root->setBoundingVolume(new BoundingVolume(m,radius));
	node = root;
	state.push_front(0);

	while (state.size() > 0)
	{
		if (currentDepth >= getMaxTreeDepth() || state.back() > 3)
		{
			state.pop_back();
			currentDepth--;
			planes.pop_back();
			if (state.size() > 0)
			{
				state.back() = state.back() + 1;
				node = node->getParent();
			}
		}
		else
		{
			
			
			plane = planes.back();
			Plane tmp;
			switch (state.back())
			{
			case 0:
				tmp.pos1 = plane.pos1;
				tmp.pos2 = (plane.pos1+plane.pos2) / 2;
				tmp.pos3 = node->getBoundingVolume()->m;
				tmp.pos4 = (plane.pos1 + plane.pos4) / 2;
				break;
			case 1:
				tmp.pos1 = (plane.pos1 + plane.pos2) / 2;
				tmp.pos2 = plane.pos2;
				tmp.pos3 = (plane.pos2 + plane.pos3) / 2;
				tmp.pos4 = node->getBoundingVolume()->m;
				break;
			case 2:
				tmp.pos1 = node->getBoundingVolume()->m;
				tmp.pos2 = (plane.pos2 + plane.pos3) / 2;
				tmp.pos3 = plane.pos3;
				tmp.pos4 = (plane.pos3 + plane.pos4) / 2;
				break;
			case 3:
				tmp.pos1 = (plane.pos1+plane.pos4) / 2;
				tmp.pos2 = node->getBoundingVolume()->m;
				tmp.pos3 = (plane.pos3 + plane.pos4) / 2;
				tmp.pos4 = plane.pos4;
				break;
			}
			

			m = (tmp.pos1 + tmp.pos3 + tmp.pos2 + tmp.pos4) / 4;
			radius = node->getBoundingVolume()->r / 2;
			BoundingVolumeTreeNode* tmpNode = new BoundingVolumeTreeNode();
			tmpNode->setBoundingVolume(new BoundingVolume(m,radius));
			node->addChild(tmpNode);
			node = tmpNode;

			currentDepth++;
			planes.push_back(tmp);
			state.push_back(0);
		}


			
		
	}



	


	return root;

	
}

void Cube::initializeVolumeTree()
{
	Real radius = sqrt(width*width + height*height + depth*depth) / 2;
	BoundingVolumeTreeNode * root = new BoundingVolumeTreeNode();
	volumeTree.setRoot(root);
	root->setBoundingVolume(new BoundingVolume(Eigen::Vector3d(0, 0, 0), radius));

	Eigen::Vector3d pos;

	//calculate face01
	root->addChild(generateSubtree(Eigen::Vector3d(-width / 2, height / 2, depth / 2), Eigen::Vector3d(width / 2, height / 2, depth / 2), Eigen::Vector3d(width / 2, -height / 2, depth / 2), Eigen::Vector3d(-width / 2, -height / 2, depth / 2)));
	root->addChild(generateSubtree(Eigen::Vector3d(width / 2, height / 2, depth / 2), Eigen::Vector3d(width / 2, height / 2, -depth / 2), Eigen::Vector3d(width / 2, -height / 2, -depth / 2), Eigen::Vector3d(width / 2, -height / 2, depth / 2)));
	root->addChild(generateSubtree(Eigen::Vector3d(width / 2, height / 2, -depth / 2), Eigen::Vector3d(-width / 2, height / 2, -depth / 2), Eigen::Vector3d(-width / 2, -height / 2, -depth / 2), Eigen::Vector3d(width / 2, -height / 2, -depth / 2)));
	root->addChild(generateSubtree(Eigen::Vector3d(-width / 2, height / 2, -depth / 2), Eigen::Vector3d(-width / 2, height / 2, depth / 2), Eigen::Vector3d(-width / 2, -height / 2, depth / 2), Eigen::Vector3d(-width / 2, -height / 2, -depth / 2)));
	root->addChild(generateSubtree(Eigen::Vector3d(-width / 2, height / 2, -depth / 2), Eigen::Vector3d(width / 2, height / 2, -depth / 2), Eigen::Vector3d(width / 2, height / 2, depth / 2), Eigen::Vector3d(-width / 2, height / 2, depth / 2)));
	root->addChild(generateSubtree(Eigen::Vector3d(-width / 2, -height / 2, depth / 2), Eigen::Vector3d(width / 2, -height / 2, depth / 2), Eigen::Vector3d(width / 2, -height / 2, -depth / 2), Eigen::Vector3d(-width / 2, -height / 2, -depth / 2)));
}




Cube & Cube::create(Real width , Real height , Real depth )
{
	Cube * cube = new Cube(width, height, depth);
	cube->addToObjectManager();
	return *cube;
}