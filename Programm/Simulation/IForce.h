#pragma once
#include <Eigen/Dense>



class IForce
{
public:
	void calculateForce(Eigen::Vector3d position, Eigen::Vector3d velocity, Eigen::Vector3d out_Force, Eigen::Vector3d out_Torque)=0;
};

