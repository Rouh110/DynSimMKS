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

#include "SimMath.h"

#define _USE_MATH_DEFINES
#include "math.h"
#include <iostream>

using namespace IBDS;
using namespace Eigen;

Real SimMath::eps = 1.0E-6;
Real SimMath::eps2 = 1.0E-12;

/** Return the one norm of the matrix.
	*/
Real SimMath::oneNorm(const Matrix3d &m) 
{
	const Real sum1 = fabs(m(0,0)) + fabs(m(1,0)) + fabs(m(2,0));
	const Real sum2 = fabs(m(0,1)) + fabs(m(1,1)) + fabs(m(2,1));
	const Real sum3 = fabs(m(0,2)) + fabs(m(1,2)) + fabs(m(2,2));
	Real maxSum = sum1;
	if (sum2 > maxSum)
		maxSum = sum2;
	if (sum3 > maxSum)
		maxSum = sum3;
	return maxSum;
}

/** Return the inf norm of the matrix.
	*/
Real SimMath::infNorm(const Matrix3d &m) 
{
	const Real sum1 = fabs(m(0,0)) + fabs(m(0,1)) + fabs(m(0,2));
	const Real sum2 = fabs(m(1,0)) + fabs(m(1,1)) + fabs(m(1,2));
	const Real sum3 = fabs(m(2,0)) + fabs(m(2,1)) + fabs(m(2,2));
	Real maxSum = sum1;
	if (sum2 > maxSum)
		maxSum = sum2;
	if (sum3 > maxSum)
		maxSum = sum3;
	return maxSum;
}


/** Perform a polar decomposition of matrix M and return the rotation matrix R.
 */
bool SimMath::polarDecompositionRot(const Matrix3d &M, const Real tolerance, Matrix3d &R)
{
	Matrix3d Mt = M.transpose();
	Real Mone = oneNorm(M);
	Real Minf = infNorm(M);
	Real Eone;
	Matrix3d MadjTt, Et;
	do 
	{
		const Vector3d &row0 = Mt.row(0);
		const Vector3d &row1 = Mt.row(1);
		const Vector3d &row2 = Mt.row(2);

		MadjTt.row(0) = row1.cross(row2);
		MadjTt.row(1) = row2.cross(row0);
		MadjTt.row(2) = row0.cross(row1);

		const Real det = Mt(0,0) * MadjTt(0,0) + Mt(0,1) * MadjTt(0,1) + Mt(0,2) * MadjTt(0,2);

		if (fabs(det) < 1.0e-12) 
		{
			std::cerr << "Determinant is zero.\n";
		}

		const Real MadjTone = oneNorm(MadjTt); 
		const Real MadjTinf = infNorm(MadjTt);

		const Real gamma = sqrt(sqrt((MadjTone*MadjTinf)/(Mone*Minf))/fabs(det));

		const Real g1 = gamma*0.5;
		const Real g2 = 0.5/(gamma*det);

		for (unsigned char i=0; i < 3; i++)
		{
			for (unsigned char j=0; j < 3; j++)
			{
				Et(i,j) = Mt(i,j);
				Mt(i,j) = g1*Mt(i,j) + g2*MadjTt(i,j);
				Et(i,j) -= Mt(i,j);
			}
		}

		Eone = oneNorm(Et);
 
		Mone = oneNorm(Mt);  
		Minf = infNorm(Mt);
	} 
	while (Eone > Mone * tolerance);

	R = Mt.transpose();

	return true;
}



/** Perform a polar decomposition of matrix M and return the rotation matrix R. This method handles the degenerated cases.
 */
bool SimMath::polarDecompositionRotStable(const Matrix3d &M, const Real tolerance, Matrix3d &R)
{
	Matrix3d Mt = M.transpose();
	Real Mone = oneNorm(M);
	Real Minf = infNorm(M);
	Real Eone;
	Matrix3d MadjTt, Et;
	do 
	{
		const Vector3d &row0 = Mt.row(0);
		const Vector3d &row1 = Mt.row(1);
		const Vector3d &row2 = Mt.row(2);

		MadjTt.row(0) = row1.cross(row2);
		MadjTt.row(1) = row2.cross(row0);
		MadjTt.row(2) = row0.cross(row1);

		Real det = Mt(0,0) * MadjTt(0,0) + Mt(0,1) * MadjTt(0,1) + Mt(0,2) * MadjTt(0,2);

		if (fabs(det) < 1.0e-12) 
		{
			Vector3d len;
			unsigned int index = 0xffffffff;
			for (unsigned int i=0; i < 3; i++)
			{
				len[i] = MadjTt.row(i).squaredNorm();
				if (len[i] > 1.0e-12)
				{
					// index of valid cross product
					// => is also the index of the vector in Mt that must be exchanged
					index = i;
					break;
				}
			}
			if (index == 0xffffffff)
			{
				R = Matrix3d::Identity();
				return true;
			}
			else
			{
				Mt.row(index) = Mt.row((index+1) % 3).cross(Mt.row((index+2) % 3));
				MadjTt.row((index+1) % 3) = Mt.row((index+2) % 3).cross(Mt.row(index));
				MadjTt.row((index+2) % 3) = Mt.row(index).cross(Mt.row((index+1) % 3));
				Matrix3d M2 = Mt.transpose();
				Mone = oneNorm(M2);
				Minf = infNorm(M2);
				det = Mt(0,0) * MadjTt(0,0) + Mt(0,1) * MadjTt(0,1) + Mt(0,2) * MadjTt(0,2);
			}

			//std::cerr << "Determinant is zero.\n";
		//	return false;
		}

		const Real MadjTone = oneNorm(MadjTt); 
		const Real MadjTinf = infNorm(MadjTt);

		const Real gamma = sqrt(sqrt((MadjTone*MadjTinf)/(Mone*Minf))/fabs(det));

		const Real g1 = gamma*0.5;
		const Real g2 = 0.5/(gamma*det);

		for (unsigned char i=0; i < 3; i++)
		{
			for (unsigned char j=0; j < 3; j++)
			{
				Et(i,j) = Mt(i,j);
				Mt(i,j) = g1*Mt(i,j) + g2*MadjTt(i,j);
				Et(i,j) -= Mt(i,j);
			}
		}

		Eone = oneNorm(Et);

		Mone = oneNorm(Mt);  
		Minf = infNorm(Mt);
	} 
	while (Eone > Mone * tolerance);

	R = Mt.transpose();

	return true;
}

