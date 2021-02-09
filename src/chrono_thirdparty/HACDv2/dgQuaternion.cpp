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

#include "dgVector.h"
#include "dgMatrix.h"
#include "dgQuaternion.h"

#pragma warning(disable:4100)

dgQuaternion::dgQuaternion (const dgMatrix &matrix)
{
	enum QUAT_INDEX
	{
		X_INDEX=0,
		Y_INDEX=1,
		Z_INDEX=2
	};
	static QUAT_INDEX QIndex [] = {Y_INDEX, Z_INDEX, X_INDEX};

	hacd::HaF32 *ptr;
	hacd::HaF32 trace;
	QUAT_INDEX i;
	QUAT_INDEX j;
	QUAT_INDEX k;

	trace = matrix[0][0] + matrix[1][1] + matrix[2][2];
	if (trace > hacd::HaF32(0.0f)) {
		trace = dgSqrt (trace + hacd::HaF32(1.0f));
		m_q0 = hacd::HaF32 (0.5f) * trace;
		trace = hacd::HaF32 (0.5f) / trace;
		m_q1 = (matrix[1][2] - matrix[2][1]) * trace;
		m_q2 = (matrix[2][0] - matrix[0][2]) * trace;
		m_q3 = (matrix[0][1] - matrix[1][0]) * trace;

	} else {
		i = X_INDEX;
		if (matrix[Y_INDEX][Y_INDEX] > matrix[X_INDEX][X_INDEX]) {
			i = Y_INDEX;
		}
		if (matrix[Z_INDEX][Z_INDEX] > matrix[i][i]) {
			i = Z_INDEX;
		}
		j = QIndex [i];
		k = QIndex [j];

		trace = hacd::HaF32(1.0f) + matrix[i][i] - matrix[j][j] - matrix[k][k];
		trace = dgSqrt (trace);

		ptr = &m_q1;
		ptr[i] = hacd::HaF32 (0.5f) * trace;
		trace  = hacd::HaF32 (0.5f) / trace;
		m_q0   = (matrix[j][k] - matrix[k][j]) * trace;
		ptr[j] = (matrix[i][j] + matrix[j][i]) * trace;
		ptr[k] = (matrix[i][k] + matrix[k][i]) * trace;
	}

#ifdef _DEBUG
	dgMatrix tmp (*this, matrix.m_posit);
	dgMatrix unitMatrix (tmp * matrix.Inverse());
	for (hacd::HaI32 i = 0; i < 4; i ++) {
		hacd::HaF32 err = dgAbsf (unitMatrix[i][i] - hacd::HaF32(1.0f));
		HACD_ASSERT (err < hacd::HaF32 (1.0e-2f));
	}

	hacd::HaF32 err = dgAbsf (DotProduct(*this) - hacd::HaF32(1.0f));
	HACD_ASSERT (err < hacd::HaF32(dgEPSILON * 100.0f));
#endif
}


dgQuaternion::dgQuaternion (const dgVector &unitAxis, hacd::HaF32 Angle)
{
	hacd::HaF32 sinAng;

	Angle *= hacd::HaF32 (0.5f);
	m_q0 = dgCos (Angle);
	sinAng = dgSin (Angle);

#ifdef _DEBUG
	if (dgAbsf (Angle) > hacd::HaF32(dgEPSILON / 10.0f)) {
		HACD_ASSERT (dgAbsf (hacd::HaF32(1.0f) - unitAxis % unitAxis) < hacd::HaF32(dgEPSILON * 10.0f));
	} 
#endif
	m_q1 = unitAxis.m_x * sinAng;
	m_q2 = unitAxis.m_y * sinAng;
	m_q3 = unitAxis.m_z * sinAng;

}


dgVector dgQuaternion::CalcAverageOmega (const dgQuaternion &QB, hacd::HaF32 dt) const
{
HACD_ASSERT (0);
return dgVector (0, 0, 0, 0);
/*
	hacd::HaF32 dirMag;
	hacd::HaF32 dirMag2;
	hacd::HaF32 omegaMag;
	hacd::HaF32 dirMagInv;

	HACD_ASSERT (0);
	dgQuaternion dq (Inverse() * QB);
//	dgQuaternion dq (QB * Inverse());
	dgVector omegaDir (dq.m_q1, dq.m_q2, dq.m_q3, hacd::HaF32 (0.0f));

	dirMag2 = omegaDir % omegaDir;
	if (dirMag2	< hacd::HaF32(hacd::HaF32 (1.0e-5f) * hacd::HaF32 (1.0e-5f))) {
		return dgVector (hacd::HaF32(0.0f), hacd::HaF32(0.0f), hacd::HaF32(0.0f), hacd::HaF32(0.0f));
	}

	dirMagInv = dgRsqrt (dirMag2);
	dirMag = dirMag2 * dirMagInv;

	omegaMag = hacd::HaF32(2.0f) * dgAtan2 (dirMag, dq.m_q0) / dt;

	return omegaDir.Scale (dirMagInv * omegaMag);
*/
}


dgQuaternion dgQuaternion::Slerp (const dgQuaternion &QB, hacd::HaF32 t) const 
{
HACD_ASSERT (0);
return dgQuaternion();
/*
	hacd::HaF32 dot;
	hacd::HaF32 ang;
	hacd::HaF32 Sclp;
	hacd::HaF32 Sclq;
	hacd::HaF32 den;
	hacd::HaF32 sinAng;
	dgQuaternion Q;

	dot = DotProduct (QB);

	if ((dot + hacd::HaF32(1.0f)) > dgEPSILON) {
		if (dot < (hacd::HaF32(1.0f) - dgEPSILON) ) {
			ang = dgAcos (dot);

			sinAng = dgSin (ang);
			den = hacd::HaF32(1.0f) / sinAng;

			Sclp = dgSin ((hacd::HaF32(1.0f) - t ) * ang) * den;
			Sclq = dgSin (t * ang) * den;

		} else  {
			Sclp = hacd::HaF32(1.0f) - t;
			Sclq = t;
		}

		Q.m_q0 = m_q0 * Sclp + QB.m_q0 * Sclq;
		Q.m_q1 = m_q1 * Sclp + QB.m_q1 * Sclq;
		Q.m_q2 = m_q2 * Sclp + QB.m_q2 * Sclq;
		Q.m_q3 = m_q3 * Sclp + QB.m_q3 * Sclq;

	} else {
		Q.m_q0 =  m_q3;
		Q.m_q1 = -m_q2;
		Q.m_q2 =  m_q1;
		Q.m_q3 =  m_q0;

		Sclp = dgSin ((hacd::HaF32(1.0f) - t) * dgPI * hacd::HaF32 (0.5f));
		Sclq = dgSin (t * dgPI * hacd::HaF32 (0.5f));

		Q.m_q0 = m_q0 * Sclp + Q.m_q0 * Sclq;
		Q.m_q1 = m_q1 * Sclp + Q.m_q1 * Sclq;
		Q.m_q2 = m_q2 * Sclp + Q.m_q2 * Sclq;
		Q.m_q3 = m_q3 * Sclp + Q.m_q3 * Sclq;
	}

	dot = Q.DotProduct (Q);
	if ((dot) < hacd::HaF32(1.0f - dgEPSILON * 10.0f) ) {
		//dot = hacd::HaF32(1.0f) / dgSqrt (dot);
		dot = dgRsqrt (dot);
		Q.m_q0 *= dot;
		Q.m_q1 *= dot;
		Q.m_q2 *= dot;
		Q.m_q3 *= dot;
	}
	return Q;
*/
}


