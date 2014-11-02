/*
 * IBDS - Impulse-Based Dynamic Simulation Library
 * Copyright (c) 2003-2008 Jan Bender http://www.impulse-based.de
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * Jan Bender - Jan.Bender@impulse-based.de
 */

#include "Particle.h"

using namespace IBDS;
using namespace Eigen;

Particle::Particle () 
{
	m_mass = 1.0;
	m_position.setZero();
	m_velocity.setZero();
	m_acceleration.setZero();
}

Particle::~Particle () 
{	
}

Real Particle::getMass() const
{
	return m_mass;
}

void Particle::setMass( Real val )
{
	m_mass = val;
}

const Vector3d & Particle::getPosition() const
{
	return m_position;
}

void Particle::setPosition( const Vector3d &val )
{
	m_position = val;
}

const Vector3d & Particle::getVelocity() const
{
	return m_velocity;
}

void Particle::setVelocity( const Vector3d &val )
{
	m_velocity = val;
}

const Vector3d & Particle::getAcceleration() const
{
	return m_acceleration;
}

void Particle::setAcceleration( const Vector3d &val )
{
	m_acceleration = val;
}
