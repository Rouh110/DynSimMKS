#pragma once
#include <Eigen/Dense>

#include "Common/Config.h"

/*
Interface for forces in the Simulation
*/
class IForce
{
public:
	/*
	Calculates the force and the torque at the given position , velocity and time.
	*/
	virtual void calculateForce(Eigen::Vector3d position, Eigen::Vector3d velocity, Real time, Eigen::Vector3d out_Force, Eigen::Vector3d out_Torque) = 0;
};

