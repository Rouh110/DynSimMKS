#pragma once

#include "Simulation/RigidBody.h"
#include "Simulation\IForce.h"

using namespace Eigen;


/*Represents a Spring.
A spring has two suspendtion points, conected to two RigidBodys.
A spring will apply force to the two rigidbodys the RigidBodys, if the distance of the two suspension points is differnt from the normal length of the spring.*/
class Spring :  public IForce
{
private:
	RigidBody *rigidBodyA;
	Vector3d suspensionPointA;
	RigidBody *rigidBodyB;
	Vector3d suspensionPointB;

	Real damper;
	Real springConst;
	Real springLength;




public:
	Spring();
	~Spring();

	/*Sets the damper.
	The damping effect will be stronger if the number is greater*/
	void setDamper(Real damper);
	/*Stets the spring constant. 
	If the constat in larger than the spring will be stronger and 'bouncier'*/
	void setSpringConstant(Real springConst);
	/*Sets the length of the spring in the rest position.*/
	void setSpringLength(Real length);

	/*Sets the suspentionPoint A and the RigidBody A.
	The position of the suspentionPoint A must be in local space of the RigidBody A.*/
	void setSuspensionPointA(const Vector3d & localPosition, RigidBody * rigidBody);
	/*Sets the suspentionPoint B and the RigidBody B.
	The position of the suspentionPoint B must be in local space of the RigidBody B.*/
	void setSuspensionPointB(const Vector3d & localPosition, RigidBody * rigidBody);

	/*Sets the suspention points A and B and the rigidbody A and B. The supention point must be in the local space of the appropriate Rigidbody.
	If setLength is true (by default it is true) than the length of the spring will be set to the distance of the two suspention points.*/
	void setSuspensionPoints(const Vector3d & localPositionA, RigidBody * rigidBodyA, const Vector3d & localPositionB, RigidBody * rigidBodyB, bool setLength = true);

	/*Compute the force and adds it to the RigidBody A and RigidBody B*/
	void computeForce(Real time);

	/*Calculates the fores at the point A and point B the spring have with the given paremeters.
	globalPositionA     The point A in global space.
	velocityA           The velocty of point A.
	globalPositionB     The point B in global space.
	velocityB           The velocty of point B.
	out_forceA          The force the spring will be apply on point A.
	out_forceB          The force the spring will be apply on point B.*/
	void calculateForces(const Vector3d & globalPositionA, const Vector3d & velocityA, const Vector3d & globalPositionB, const Vector3d & velocityB, Vector3d & out_forceA, Vector3d & out_forceB) const;

	/*
	Creates a Spring with the given parameters and adds it to the ObjectManager.
	Returns a Referenze to the created spring.
	*/
	static Spring & create();
	static Spring & create(Real damper, Real springConst, Real springLength);

	/*
	Renders the spring.
	*/
	void render();
protected:
	/* Writes the suspendtion point A in global space into the given vector*/
	void getGlobalPositionA(Vector3d & out_position) const;
	/* Writes the suspention point B in global space into the given vector*/
	void getGlobalPositionB(Vector3d & out_position) const;
};

