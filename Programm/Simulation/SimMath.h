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

#ifndef __SIMMATH_H__
#define __SIMMATH_H__

#include "Common/Config.h"
#include <Eigen/Dense>

namespace IBDS 
{
	/** Klasse für Mathematik-Hilfsfunktionen, die für 
	  * die Simulation benötigt werden.
	  */
	class SimMath
	{
	public:
		/** Epsilon wird für Tests auf 0 verwendet */
		static Real eps;
		/** Epsilon im Quadrat */
		static Real eps2;

		static Real oneNorm(const Eigen::Matrix3d &m);
		static Real infNorm(const Eigen::Matrix3d &m);
		static bool polarDecompositionRot(const Eigen::Matrix3d &M, const Real tolerance, Eigen::Matrix3d &R);
		static bool polarDecompositionRotStable(const Eigen::Matrix3d &M, const Real tolerance, Eigen::Matrix3d &R);
	};
}

#endif
