/* Copyright (c) <2003-2011> <Julio Jerez, Newton Game Dynamics>
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
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 
* 3. This notice may not be removed or altered from any source distribution.
*/

#include "dgTypes.h"
#include "dgGoogol.h"
#include "dgSmallDeterminant.h"

#define Absolute(a)  ((a) >= 0.0 ? (a) : -(a))

hacd::HaF64 Determinant2x2 (const hacd::HaF64 matrix[2][2], hacd::HaF64* const error)
{
	hacd::HaF64 a00xa11 = matrix[0][0] * matrix[1][1];
	hacd::HaF64 a01xa10 = matrix[0][1] * matrix[1][0];
	*error = Absolute(a00xa11) + Absolute(a01xa10);
	return a00xa11 - a01xa10;
}

dgGoogol Determinant2x2 (const dgGoogol matrix[2][2])
{
	dgGoogol a00xa11 (matrix[0][0] * matrix[1][1]);
	dgGoogol a01xa10 (matrix[0][1] * matrix[1][0]);
	return a00xa11 - a01xa10;
}



hacd::HaF64 Determinant3x3 (const hacd::HaF64 matrix[3][3], hacd::HaF64* const error)
{
	hacd::HaF64 sign = hacd::HaF64 (-1.0f);
	hacd::HaF64 det = hacd::HaF64 (0.0f);
	hacd::HaF64 accError = hacd::HaF64 (0.0f); 
	for (hacd::HaI32 i = 0; i < 3; i ++)  {
		hacd::HaF64 cofactor[2][2];
		for (hacd::HaI32 j = 0; j < 2; j ++) {
			hacd::HaI32 k0 = 0;
			for (hacd::HaI32 k = 0; k < 3; k ++) {
				if (k != i) {
					cofactor[j][k0] = matrix[j][k];
					k0 ++;
				}
			}
		}

		hacd::HaF64 parcialError;
		hacd::HaF64 minorDet = Determinant2x2 (cofactor, &parcialError);
		accError += parcialError * Absolute (matrix[2][i]);
		det += sign * minorDet * matrix[2][i];
		sign *= hacd::HaF64 (-1.0f);
	}

	*error = accError;
	return det;
}

dgGoogol Determinant3x3 (const dgGoogol matrix[3][3])
{
	dgGoogol negOne (hacd::HaF64 (-1.0f));
	dgGoogol sign (hacd::HaF64 (-1.0f));
	dgGoogol det = hacd::HaF64 (0.0f);
	for (hacd::HaI32 i = 0; i < 3; i ++)  {
		dgGoogol cofactor[2][2];

		for (hacd::HaI32 j = 0; j < 2; j ++) {
			hacd::HaI32 k0 = 0;
			for (hacd::HaI32 k = 0; k < 3; k ++) {
				if (k != i) {
					cofactor[j][k0] = matrix[j][k];
					k0 ++;
				}
			}
		}

		dgGoogol minorDet (Determinant2x2 (cofactor));
		det = det + sign * minorDet * matrix[2][i];
		sign = sign * negOne;
	}
	return det;
}


hacd::HaF64 Determinant4x4 (const hacd::HaF64 matrix[4][4], hacd::HaF64* const error)
{
	hacd::HaF64 sign = hacd::HaF64 (1.0f);
	hacd::HaF64 det = hacd::HaF64 (0.0f);
	hacd::HaF64 accError = hacd::HaF64 (0.0f); 
	for (hacd::HaI32 i = 0; i < 4; i ++)  {
		hacd::HaF64 cofactor[3][3];
		for (hacd::HaI32 j = 0; j < 3; j ++) {
			hacd::HaI32 k0 = 0;
			for (hacd::HaI32 k = 0; k < 4; k ++) {
				if (k != i) {
					cofactor[j][k0] = matrix[j][k];
					k0 ++;
				}
			}
		}

		hacd::HaF64 parcialError;
		hacd::HaF64 minorDet = Determinant3x3 (cofactor, &parcialError);
		accError +=  parcialError * Absolute (matrix[3][i]);
		det += sign * minorDet * matrix[3][i];
		sign *= hacd::HaF64 (-1.0f);
	}

	*error = accError;
	return det;
}


dgGoogol Determinant4x4 (const dgGoogol matrix[4][4])
{
	dgGoogol sign = hacd::HaF64 (1.0f);
	dgGoogol det = hacd::HaF64 (0.0f);
	dgGoogol negOne (hacd::HaF64 (-1.0f));
	dgGoogol accError = hacd::HaF64 (0.0f); 
	for (hacd::HaI32 i = 0; i < 4; i ++)  {
		dgGoogol  cofactor[3][3];
		for (hacd::HaI32 j = 0; j < 3; j ++) {
			hacd::HaI32 k0 = 0;
			for (hacd::HaI32 k = 0; k < 4; k ++) {
				if (k != i) {
					cofactor[j][k0] = matrix[j][k];
					k0 ++;
				}
			}
		}

		dgGoogol minorDet = Determinant3x3 (cofactor);
		det = det + sign * minorDet * matrix[3][i];
		sign = sign * negOne;
	}
	return det;
}


