#pragma once
#include <Eigen/Dense>
class TestClass
{
private:
	Eigen::Vector3d inertiaTensor;
	Eigen::Vector3d invertedInertiaTensor;
	Eigen::Vector3d position;
	Eigen::Vector3d velocity;
	Eigen::Vector3d angulaVelocity;
	Eigen::Vector3d torque;
	Eigen::Vector3d force;
	Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
public:
	TestClass();
	~TestClass();
};

