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
#include "dgMatrix.h"
#include "dgQuaternion.h"


static dgMatrix zeroMatrix (dgVector (hacd::HaF32(0.0f), hacd::HaF32(0.0f), hacd::HaF32(0.0f), hacd::HaF32(0.0f)),
							dgVector (hacd::HaF32(0.0f), hacd::HaF32(0.0f), hacd::HaF32(0.0f), hacd::HaF32(0.0f)),
							dgVector (hacd::HaF32(0.0f), hacd::HaF32(0.0f), hacd::HaF32(0.0f), hacd::HaF32(0.0f)),
							dgVector (hacd::HaF32(0.0f), hacd::HaF32(0.0f), hacd::HaF32(0.0f), hacd::HaF32(0.0f)));

static dgMatrix identityMatrix (dgVector (hacd::HaF32(1.0f), hacd::HaF32(0.0f), hacd::HaF32(0.0f), hacd::HaF32(0.0f)),
								dgVector (hacd::HaF32(0.0f), hacd::HaF32(1.0f), hacd::HaF32(0.0f), hacd::HaF32(0.0f)),
								dgVector (hacd::HaF32(0.0f), hacd::HaF32(0.0f), hacd::HaF32(1.0f), hacd::HaF32(0.0f)),
								dgVector (hacd::HaF32(0.0f), hacd::HaF32(0.0f), hacd::HaF32(0.0f), hacd::HaF32(1.0f)));

const dgMatrix& dgGetIdentityMatrix()
{
	return identityMatrix;
}

const dgMatrix& dgGetZeroMatrix ()
{
	return zeroMatrix;
}


dgMatrix::dgMatrix (const dgQuaternion &rotation, const dgVector &position)
{
	hacd::HaF32 x2 = hacd::HaF32 (2.0f) * rotation.m_q1 * rotation.m_q1;
	hacd::HaF32 y2 = hacd::HaF32 (2.0f) * rotation.m_q2 * rotation.m_q2;
	hacd::HaF32 z2 = hacd::HaF32 (2.0f) * rotation.m_q3 * rotation.m_q3;

#ifdef _DEBUG
	hacd::HaF32 w2 = hacd::HaF32 (2.0f) * rotation.m_q0 * rotation.m_q0;
	HACD_ASSERT (dgAbsf (w2 + x2 + y2 + z2 - hacd::HaF32(2.0f)) <hacd::HaF32 (1.0e-3f));
#endif

	hacd::HaF32 xy = hacd::HaF32 (2.0f) * rotation.m_q1 * rotation.m_q2;
	hacd::HaF32 xz = hacd::HaF32 (2.0f) * rotation.m_q1 * rotation.m_q3;
	hacd::HaF32 xw = hacd::HaF32 (2.0f) * rotation.m_q1 * rotation.m_q0;
	hacd::HaF32 yz = hacd::HaF32 (2.0f) * rotation.m_q2 * rotation.m_q3;
	hacd::HaF32 yw = hacd::HaF32 (2.0f) * rotation.m_q2 * rotation.m_q0;
	hacd::HaF32 zw = hacd::HaF32 (2.0f) * rotation.m_q3 * rotation.m_q0;

	m_front = dgVector (hacd::HaF32(1.0f) - y2 - z2, xy + zw,                   xz - yw				    , hacd::HaF32(0.0f));
	m_up    = dgVector (xy - zw,                   hacd::HaF32(1.0f) - x2 - z2, yz + xw					, hacd::HaF32(0.0f));
	m_right = dgVector (xz + yw,                   yz - xw,                   hacd::HaF32(1.0f) - x2 - y2 , hacd::HaF32(0.0f));


	m_posit.m_x = position.m_x;
	m_posit.m_y = position.m_y;
	m_posit.m_z = position.m_z;
	m_posit.m_w = hacd::HaF32(1.0f);
}

dgMatrix dgMatrix::operator* (const dgMatrix &B) const
{
	const dgMatrix& A = *this;
	return dgMatrix (dgVector (A[0][0] * B[0][0] + A[0][1] * B[1][0] + A[0][2] * B[2][0] + A[0][3] * B[3][0],
							   A[0][0] * B[0][1] + A[0][1] * B[1][1] + A[0][2] * B[2][1] + A[0][3] * B[3][1],
							   A[0][0] * B[0][2] + A[0][1] * B[1][2] + A[0][2] * B[2][2] + A[0][3] * B[3][2],
							   A[0][0] * B[0][3] + A[0][1] * B[1][3] + A[0][2] * B[2][3] + A[0][3] * B[3][3]),
					 dgVector (A[1][0] * B[0][0] + A[1][1] * B[1][0] + A[1][2] * B[2][0] + A[1][3] * B[3][0],
							   A[1][0] * B[0][1] + A[1][1] * B[1][1] + A[1][2] * B[2][1] + A[1][3] * B[3][1],
							   A[1][0] * B[0][2] + A[1][1] * B[1][2] + A[1][2] * B[2][2] + A[1][3] * B[3][2],
							   A[1][0] * B[0][3] + A[1][1] * B[1][3] + A[1][2] * B[2][3] + A[1][3] * B[3][3]),
					 dgVector (A[2][0] * B[0][0] + A[2][1] * B[1][0] + A[2][2] * B[2][0] + A[2][3] * B[3][0],
							   A[2][0] * B[0][1] + A[2][1] * B[1][1] + A[2][2] * B[2][1] + A[2][3] * B[3][1],
							   A[2][0] * B[0][2] + A[2][1] * B[1][2] + A[2][2] * B[2][2] + A[2][3] * B[3][2],
							   A[2][0] * B[0][3] + A[2][1] * B[1][3] + A[2][2] * B[2][3] + A[2][3] * B[3][3]),
					 dgVector (A[3][0] * B[0][0] + A[3][1] * B[1][0] + A[3][2] * B[2][0] + A[3][3] * B[3][0],
							   A[3][0] * B[0][1] + A[3][1] * B[1][1] + A[3][2] * B[2][1] + A[3][3] * B[3][1],
							   A[3][0] * B[0][2] + A[3][1] * B[1][2] + A[3][2] * B[2][2] + A[3][3] * B[3][2],
							   A[3][0] * B[0][3] + A[3][1] * B[1][3] + A[3][2] * B[2][3] + A[3][3] * B[3][3]));
}



void dgMatrix::TransformTriplex (hacd::HaF32* const dst, hacd::HaI32 dstStrideInBytes, const hacd::HaF32* const src, hacd::HaI32 srcStrideInBytes, hacd::HaI32 count) const
{
	hacd::HaI32 dstStride = dstStrideInBytes /sizeof (hacd::HaF32);
	hacd::HaI32 srcStride = srcStrideInBytes / sizeof (hacd::HaF32);

	hacd::HaI32 dstIndex = 0;
	hacd::HaI32 srcIndex = 0;
	for (hacd::HaI32 i = 0 ; i < count; i ++ ) {
		hacd::HaF32 x = src[srcIndex + 0];
		hacd::HaF32 y = src[srcIndex + 1];
		hacd::HaF32 z = src[srcIndex + 2];
		srcIndex += srcStride;
		dst[dstIndex + 0] = x * m_front.m_x + y * m_up.m_x + z * m_right.m_x + m_posit.m_x;
		dst[dstIndex + 1] = x * m_front.m_y + y * m_up.m_y + z * m_right.m_y + m_posit.m_y;
		dst[dstIndex + 2] = x * m_front.m_z + y * m_up.m_z + z * m_right.m_z + m_posit.m_z;
		dstIndex += dstStride;
	}
}

void dgMatrix::TransformTriplex (hacd::HaF64* const dst, hacd::HaI32 dstStrideInBytes, const hacd::HaF64* const src, hacd::HaI32 srcStrideInBytes, hacd::HaI32 count) const
{
	hacd::HaI32 dstStride = dstStrideInBytes /sizeof (hacd::HaF64);
	hacd::HaI32 srcStride = srcStrideInBytes / sizeof (hacd::HaF64);

	hacd::HaI32 dstIndex = 0;
	hacd::HaI32 srcIndex = 0;
	for (hacd::HaI32 i = 0 ; i < count; i ++ ) {
		hacd::HaF64 x = src[srcIndex + 0];
		hacd::HaF64 y = src[srcIndex + 1];
		hacd::HaF64 z = src[srcIndex + 2];
		srcIndex += srcStride;
		dst[dstIndex + 0] = x * m_front.m_x + y * m_up.m_x + z * m_right.m_x + m_posit.m_x;
		dst[dstIndex + 1] = x * m_front.m_y + y * m_up.m_y + z * m_right.m_y + m_posit.m_y;
		dst[dstIndex + 2] = x * m_front.m_z + y * m_up.m_z + z * m_right.m_z + m_posit.m_z;
		dstIndex += dstStride;
	}
}


void dgMatrix::TransformTriplex (hacd::HaF64* const dst, hacd::HaI32 dstStrideInBytes, const hacd::HaF32* const src, hacd::HaI32 srcStrideInBytes, hacd::HaI32 count) const
{
	hacd::HaI32 dstStride = dstStrideInBytes /sizeof (hacd::HaF64);
	hacd::HaI32 srcStride = srcStrideInBytes / sizeof (hacd::HaF32);

	hacd::HaI32 dstIndex = 0;
	hacd::HaI32 srcIndex = 0;
	for (hacd::HaI32 i = 0 ; i < count; i ++ ) {
		hacd::HaF64 x = src[srcIndex + 0];
		hacd::HaF64 y = src[srcIndex + 1];
		hacd::HaF64 z = src[srcIndex + 2];
		srcIndex += srcStride;
		dst[dstIndex + 0] = x * m_front.m_x + y * m_up.m_x + z * m_right.m_x + m_posit.m_x;
		dst[dstIndex + 1] = x * m_front.m_y + y * m_up.m_y + z * m_right.m_y + m_posit.m_y;
		dst[dstIndex + 2] = x * m_front.m_z + y * m_up.m_z + z * m_right.m_z + m_posit.m_z;
		dstIndex += dstStride;
	}
}


void dgMatrix::TransformBBox (const dgVector& p0local, const dgVector& p1local, dgVector& p0, dgVector& p1) const
{
	dgVector box[8];

	box[0][0] = p0local[0];
	box[0][1] = p0local[1];
	box[0][2] = p0local[2];
	box[0][3] = hacd::HaF32(1.0f);

	box[1][0] = p0local[0];
	box[1][1] = p0local[1];
	box[1][2] = p1local[2];
	box[1][3] = hacd::HaF32(1.0f);

	box[2][0] = p0local[0];
	box[2][1] = p1local[1];
	box[2][2] = p0local[2];
	box[2][3] = hacd::HaF32(1.0f);

	box[3][0] = p0local[0];
	box[3][1] = p1local[1];
	box[3][2] = p1local[2];
	box[3][3] = hacd::HaF32(1.0f);

	box[4][0] = p1local[0];
	box[4][1] = p0local[1];
	box[4][2] = p0local[2];
	box[4][3] = hacd::HaF32(1.0f);

	box[5][0] = p1local[0];
	box[5][1] = p0local[1];
	box[5][2] = p1local[2];
	box[1][3] = hacd::HaF32(1.0f);

	box[6][0] = p1local[0];
	box[6][1] = p1local[1];
	box[6][2] = p0local[2];
	box[6][3] = hacd::HaF32(1.0f);

	box[7][0] = p1local[0];
	box[7][1] = p1local[1];
	box[7][2] = p1local[2];
	box[7][3] = hacd::HaF32(1.0f);

	TransformTriplex (&box[0].m_x, sizeof (dgVector), &box[0].m_x, sizeof (dgVector), 8);

	p0 = box[0];
	p1 = box[0];
	for (hacd::HaI32 i = 1; i < 8; i ++) {
		p0.m_x = GetMin (p0.m_x, box[i].m_x);
		p0.m_y = GetMin (p0.m_y, box[i].m_y);
		p0.m_z = GetMin (p0.m_z, box[i].m_z);

		p1.m_x = GetMax (p1.m_x, box[i].m_x);
		p1.m_y = GetMax (p1.m_y, box[i].m_y);
		p1.m_z = GetMax (p1.m_z, box[i].m_z);
	}
}



dgMatrix dgMatrix::Symetric3by3Inverse () const
{
	const dgMatrix& mat = *this;
	hacd::HaF64 det = mat[0][0] * mat[1][1] * mat[2][2] + 
			mat[0][1] * mat[1][2] * mat[0][2] * hacd::HaF32 (2.0f) -
			mat[0][2] * mat[1][1] * mat[0][2] -
			mat[0][1] * mat[0][1] * mat[2][2] -
			mat[0][0] * mat[1][2] * mat[1][2];

	det = hacd::HaF32 (1.0f) / det;

	hacd::HaF32 x11 = (hacd::HaF32)(det * (mat[1][1] * mat[2][2] - mat[1][2] * mat[1][2]));  
	hacd::HaF32 x22 = (hacd::HaF32)(det * (mat[0][0] * mat[2][2] - mat[0][2] * mat[0][2]));  
	hacd::HaF32 x33 = (hacd::HaF32)(det * (mat[0][0] * mat[1][1] - mat[0][1] * mat[0][1]));  

	hacd::HaF32 x12 = (hacd::HaF32)(det * (mat[1][2] * mat[2][0] - mat[1][0] * mat[2][2]));  
	hacd::HaF32 x13 = (hacd::HaF32)(det * (mat[1][0] * mat[2][1] - mat[1][1] * mat[2][0]));  
	hacd::HaF32 x23 = (hacd::HaF32)(det * (mat[0][1] * mat[2][0] - mat[0][0] * mat[2][1]));  


#ifdef _DEBUG
	dgMatrix matInv (dgVector (x11, x12, x13, hacd::HaF32(0.0f)),
				     dgVector (x12, x22, x23, hacd::HaF32(0.0f)),
					 dgVector (x13, x23, x33, hacd::HaF32(0.0f)),
					 dgVector (hacd::HaF32(0.0f), hacd::HaF32(0.0f), hacd::HaF32(0.0f), hacd::HaF32(1.0f)));

	dgMatrix test (matInv * mat);
	HACD_ASSERT (dgAbsf (test[0][0] - hacd::HaF32(1.0f)) < hacd::HaF32(0.01f));
	HACD_ASSERT (dgAbsf (test[1][1] - hacd::HaF32(1.0f)) < hacd::HaF32(0.01f));
	HACD_ASSERT (dgAbsf (test[2][2] - hacd::HaF32(1.0f)) < hacd::HaF32(0.01f));
#endif

	return dgMatrix (dgVector (x11, x12, x13, hacd::HaF32(0.0f)),
					 dgVector (x12, x22, x23, hacd::HaF32(0.0f)),
					 dgVector (x13, x23, x33, hacd::HaF32(0.0f)),
					 dgVector (hacd::HaF32(0.0f), hacd::HaF32(0.0f), hacd::HaF32(0.0f), hacd::HaF32(1.0f)));
}





dgVector dgMatrix::CalcPitchYawRoll () const
{
	const hacd::HaF32 minSin = hacd::HaF32(0.99995f);

	const dgMatrix& matrix = *this;

	hacd::HaF32 roll = hacd::HaF32(0.0f);
	hacd::HaF32 pitch  = hacd::HaF32(0.0f);
	hacd::HaF32 yaw = dgAsin (-ClampValue (matrix[0][2], hacd::HaF32(-0.999999f), hacd::HaF32(0.999999f)));

	HACD_ASSERT (dgCheckFloat (yaw));
	if (matrix[0][2] < minSin) {
		if (matrix[0][2] > (-minSin)) {
			roll = dgAtan2 (matrix[0][1], matrix[0][0]);
			pitch = dgAtan2 (matrix[1][2], matrix[2][2]);
		} else {
			pitch = dgAtan2 (matrix[1][0], matrix[1][1]);
		}
	} else {
		pitch = -dgAtan2 (matrix[1][0], matrix[1][1]);
	}

#ifdef _DEBUG
	dgMatrix m (dgPitchMatrix (pitch) * dgYawMatrix(yaw) * dgRollMatrix(roll));
	for (hacd::HaI32 i = 0; i < 3; i ++) {
		for (hacd::HaI32 j = 0; j < 3; j ++) {
			hacd::HaF32 error = dgAbsf (m[i][j] - matrix[i][j]);
			HACD_ASSERT (error < 5.0e-2f);
		}
	}
#endif

	return dgVector (pitch, yaw, roll, hacd::HaF32(0.0f));
}


/*
static inline void ROT(dgMatrix &a, hacd::HaI32 i, hacd::HaI32 j, hacd::HaI32 k, hacd::HaI32 l, hacd::HaF32 s, hacd::HaF32 tau) 
{
	hacd::HaF32 g = a[i][j]; 
	hacd::HaF32 h = a[k][l]; 
	a[i][j] = g - s * (h + g * tau); 
	a[k][l] = h + s * (g - h * tau);
}

// from numerical recipes in c
// Jacobian method for computing the eigenvectors of a symmetric matrix
void dgMatrix::EigenVectors (dgVector &eigenValues, const dgMatrix& initialGuess)
{
	hacd::HaF32 b[3];
	hacd::HaF32 z[3];
	hacd::HaF32 d[3];

	//	dgMatrix eigenVectors (initialGuess.Transpose4X4());
	//	dgMatrix &mat = *this;

	dgMatrix& mat = *this;
	dgMatrix eigenVectors (initialGuess.Transpose4X4());
	mat = initialGuess * mat * eigenVectors;


	b[0] = mat[0][0]; 
	b[1] = mat[1][1];
	b[2] = mat[2][2];

	d[0] = mat[0][0]; 
	d[1] = mat[1][1]; 
	d[2] = mat[2][2]; 

	z[0] = hacd::HaF32 (0.0f);
	z[1] = hacd::HaF32 (0.0f);
	z[2] = hacd::HaF32 (0.0f);

	hacd::HaI32 nrot = 0;
	for (hacd::HaI32 i = 0; i < 50; i++) {
		hacd::HaF32 sm = dgAbsf(mat[0][1]) + dgAbsf(mat[0][2]) + dgAbsf(mat[1][2]);

		if (sm < hacd::HaF32 (1.0e-6f)) {
			HACD_ASSERT (dgAbsf((eigenVectors.m_front % eigenVectors.m_front) - hacd::HaF32(1.0f)) < dgEPSILON);
			HACD_ASSERT (dgAbsf((eigenVectors.m_up % eigenVectors.m_up) - hacd::HaF32(1.0f)) < dgEPSILON);
			HACD_ASSERT (dgAbsf((eigenVectors.m_right % eigenVectors.m_right) - hacd::HaF32(1.0f)) < dgEPSILON);

			// order the eigenvalue vectors	
			dgVector tmp (eigenVectors.m_front * eigenVectors.m_up);
			if (tmp % eigenVectors.m_right < hacd::HaF32(0.0f)) {
				eigenVectors.m_right = eigenVectors.m_right.Scale (-hacd::HaF32(1.0f));
			}

			eigenValues = dgVector (d[0], d[1], d[2], hacd::HaF32 (0.0f));
			*this = eigenVectors.Inverse();
			return;
		}

		hacd::HaF32 thresh = hacd::HaF32 (0.0f);
		if (i < 3) {
			thresh = (hacd::HaF32)(0.2f / 9.0f) * sm;
		}


		// First row
		hacd::HaF32 g = hacd::HaF32 (100.0f) * dgAbsf(mat[0][1]);
		if ((i > 3) && (dgAbsf(d[0]) + g == dgAbsf(d[0])) && (dgAbsf(d[1]) + g == dgAbsf(d[1]))) {
			mat[0][1] = hacd::HaF32 (0.0f);
		} else if (dgAbsf(mat[0][1]) > thresh) {
			hacd::HaF32 h = d[1] - d[0];
			hacd::HaF32 t;
			if (dgAbsf(h) + g == dgAbsf(h)) {
				t = mat[0][1] / h;
			} else {
				hacd::HaF32 theta = hacd::HaF32 (0.5f) * h / mat[0][1];
				t = hacd::HaF32(1.0f) / (dgAbsf(theta) + dgSqrt(hacd::HaF32(1.0f) + theta * theta));
				if (theta < hacd::HaF32 (0.0f)) {
					t = -t;
				}
			}
			hacd::HaF32 c = hacd::HaF32(1.0f) / dgSqrt (hacd::HaF32 (1.0f) + t * t); 
			hacd::HaF32 s = t * c; 
			hacd::HaF32 tau = s / (hacd::HaF32(1.0f) + c); 
			h = t * mat[0][1];
			z[0] -= h; 
			z[1] += h; 
			d[0] -= h; 
			d[1] += h;
			mat[0][1] = hacd::HaF32(0.0f);
			ROT (mat, 0, 2, 1, 2, s, tau); 
			ROT (eigenVectors, 0, 0, 0, 1, s, tau); 
			ROT (eigenVectors, 1, 0, 1, 1, s, tau); 
			ROT (eigenVectors, 2, 0, 2, 1, s, tau); 

			nrot++;
		}


		// second row
		g = hacd::HaF32 (100.0f) * dgAbsf(mat[0][2]);
		if ((i > 3) && (dgAbsf(d[0]) + g == dgAbsf(d[0])) && (dgAbsf(d[2]) + g == dgAbsf(d[2]))) {
			mat[0][2] = hacd::HaF32 (0.0f);
		} else if (dgAbsf(mat[0][2]) > thresh) {
			hacd::HaF32 h = d[2] - d[0];
			hacd::HaF32 t;
			if (dgAbsf(h) + g == dgAbsf(h)) {
				t = (mat[0][2]) / h;
			}	else {
				hacd::HaF32 theta = hacd::HaF32 (0.5f) * h / mat[0][2];
				t = hacd::HaF32(1.0f) / (dgAbsf(theta) + dgSqrt(hacd::HaF32(1.0f) + theta * theta));
				if (theta < hacd::HaF32 (0.0f)) {
					t = -t;
				}
			}
			hacd::HaF32 c = hacd::HaF32(1.0f) / dgSqrt(hacd::HaF32 (1.0f) + t * t); 
			hacd::HaF32 s = t * c; 
			hacd::HaF32 tau = s / (hacd::HaF32(1.0f) + c); 
			h = t * mat[0][2];
			z[0] -= h; 
			z[2] += h; 
			d[0] -= h; 
			d[2] += h;
			mat[0][2]=hacd::HaF32 (0.0f);
			ROT (mat, 0, 1, 1, 2, s, tau); 
			ROT (eigenVectors, 0, 0, 0, 2, s, tau); 
			ROT (eigenVectors, 1, 0, 1, 2, s, tau); 
			ROT (eigenVectors, 2, 0, 2, 2, s, tau); 
		}

		// third row
		g = hacd::HaF32 (100.0f) * dgAbsf(mat[1][2]);
		if ((i > 3) && (dgAbsf(d[1]) + g == dgAbsf(d[1])) && (dgAbsf(d[2]) + g == dgAbsf(d[2]))) {
			mat[1][2] = hacd::HaF32 (0.0f);
		} else if (dgAbsf(mat[1][2]) > thresh) {
			hacd::HaF32 h = d[2] - d[1];
			hacd::HaF32 t;
			if (dgAbsf(h) + g == dgAbsf(h)) {
				t = mat[1][2] / h;
			}	else {
				hacd::HaF32 theta = hacd::HaF32 (0.5f) * h / mat[1][2];
				t = hacd::HaF32(1.0f) / (dgAbsf(theta) + dgSqrt(hacd::HaF32(1.0f) + theta * theta));
				if (theta < hacd::HaF32 (0.0f)) {
					t = -t;
				}
			}
			hacd::HaF32 c = hacd::HaF32(1.0f) / dgSqrt(hacd::HaF32 (1.0f) + t*t); 
			hacd::HaF32 s = t * c; 
			hacd::HaF32 tau = s / (hacd::HaF32(1.0f) + c); 

			h = t * mat[1][2];
			z[1] -= h; 
			z[2] += h; 
			d[1] -= h; 
			d[2] += h;
			mat[1][2] = hacd::HaF32 (0.0f);
			ROT (mat, 0, 1, 0, 2, s, tau); 
			ROT (eigenVectors, 0, 1, 0, 2, s, tau); 
			ROT (eigenVectors, 1, 1, 1, 2, s, tau); 
			ROT (eigenVectors, 2, 1, 2, 2, s, tau); 
			nrot++;
		}

		b[0] += z[0]; d[0] = b[0]; z[0] = hacd::HaF32 (0.0f);
		b[1] += z[1]; d[1] = b[1]; z[1] = hacd::HaF32 (0.0f);
		b[2] += z[2]; d[2] = b[2]; z[2] = hacd::HaF32 (0.0f);
	}

	eigenValues = dgVector (d[0], d[1], d[2], hacd::HaF32 (0.0f));
	*this = dgGetIdentityMatrix();
} 	
*/

void dgMatrix::EigenVectors (dgVector &eigenValues, const dgMatrix& initialGuess)
{
	hacd::HaF32 b[3];
	hacd::HaF32 z[3];
	hacd::HaF32 d[3];

	dgMatrix& mat = *this;
	dgMatrix eigenVectors (initialGuess.Transpose4X4());
	mat = initialGuess * mat * eigenVectors;

	b[0] = mat[0][0]; 
	b[1] = mat[1][1];
	b[2] = mat[2][2];

	d[0] = mat[0][0]; 
	d[1] = mat[1][1]; 
	d[2] = mat[2][2]; 

	z[0] = hacd::HaF32 (0.0f);
	z[1] = hacd::HaF32 (0.0f);
	z[2] = hacd::HaF32 (0.0f);

	for (hacd::HaI32 i = 0; i < 50; i++) {
		hacd::HaF32 sm = dgAbsf(mat[0][1]) + dgAbsf(mat[0][2]) + dgAbsf(mat[1][2]);

		if (sm < hacd::HaF32 (1.0e-6f)) {
			HACD_ASSERT (dgAbsf((eigenVectors.m_front % eigenVectors.m_front) - hacd::HaF32(1.0f)) < dgEPSILON);
			HACD_ASSERT (dgAbsf((eigenVectors.m_up % eigenVectors.m_up) - hacd::HaF32(1.0f)) < dgEPSILON);
			HACD_ASSERT (dgAbsf((eigenVectors.m_right % eigenVectors.m_right) - hacd::HaF32(1.0f)) < dgEPSILON);

			// order the eigenvalue vectors	
			dgVector tmp (eigenVectors.m_front * eigenVectors.m_up);
			if (tmp % eigenVectors.m_right < hacd::HaF32(0.0f)) {
				eigenVectors.m_right = eigenVectors.m_right.Scale (-hacd::HaF32(1.0f));
			}

			eigenValues = dgVector (d[0], d[1], d[2], hacd::HaF32 (0.0f));
			*this = eigenVectors.Inverse();
			return;
		}

		hacd::HaF32 thresh = hacd::HaF32 (0.0f);
		if (i < 3) {
			thresh = (hacd::HaF32)(0.2f / 9.0f) * sm;
		}

		for (hacd::HaI32 ip = 0; ip < 2; ip ++) {
			for (hacd::HaI32 iq = ip + 1; iq < 3; iq ++) {
				hacd::HaF32 g = hacd::HaF32 (100.0f) * dgAbsf(mat[ip][iq]);
				//if ((i > 3) && (dgAbsf(d[0]) + g == dgAbsf(d[ip])) && (dgAbsf(d[1]) + g == dgAbsf(d[1]))) {
				if ((i > 3) && ((dgAbsf(d[ip]) + g) == dgAbsf(d[ip])) && ((dgAbsf(d[iq]) + g) == dgAbsf(d[iq]))) {
					mat[ip][iq] = hacd::HaF32 (0.0f);
				} else if (dgAbsf(mat[ip][iq]) > thresh) {

					hacd::HaF32 t;
					hacd::HaF32 h = d[iq] - d[ip];
					if (dgAbsf(h) + g == dgAbsf(h)) {
						t = mat[ip][iq] / h;
					} else {
						hacd::HaF32 theta = hacd::HaF32 (0.5f) * h / mat[ip][iq];
						t = hacd::HaF32(1.0f) / (dgAbsf(theta) + dgSqrt(hacd::HaF32(1.0f) + theta * theta));
						if (theta < hacd::HaF32 (0.0f)) {
							t = -t;
						}
					}
					hacd::HaF32 c = hacd::HaF32(1.0f) / dgSqrt (hacd::HaF32 (1.0f) + t * t); 
					hacd::HaF32 s = t * c; 
					hacd::HaF32 tau = s / (hacd::HaF32(1.0f) + c); 
					h = t * mat[ip][iq];
					z[ip] -= h; 
					z[iq] += h; 
					d[ip] -= h; 
					d[iq] += h;
					mat[ip][iq] = hacd::HaF32(0.0f);

					for (hacd::HaI32 j = 0; j <= ip - 1; j ++) {
						//ROT (mat, j, ip, j, iq, s, tau); 
						//ROT(dgMatrix &a, hacd::HaI32 i, hacd::HaI32 j, hacd::HaI32 k, hacd::HaI32 l, hacd::HaF32 s, hacd::HaF32 tau) 
						hacd::HaF32 g = mat[j][ip]; 
						hacd::HaF32 h = mat[j][iq]; 
						mat[j][ip] = g - s * (h + g * tau); 
						mat[j][iq] = h + s * (g - h * tau);

					}
					for (hacd::HaI32 j = ip + 1; j <= iq - 1; j ++) {
						//ROT (mat, ip, j, j, iq, s, tau); 
						//ROT(dgMatrix &a, hacd::HaI32 i, hacd::HaI32 j, hacd::HaI32 k, hacd::HaI32 l, hacd::HaF32 s, hacd::HaF32 tau) 
						hacd::HaF32 g = mat[ip][j]; 
						hacd::HaF32 h = mat[j][iq]; 
						mat[ip][j] = g - s * (h + g * tau); 
						mat[j][iq] = h + s * (g - h * tau);
					}
					for (hacd::HaI32 j = iq + 1; j < 3; j ++) {
						//ROT (mat, ip, j, iq, j, s, tau); 
						//ROT(dgMatrix &a, hacd::HaI32 i, hacd::HaI32 j, hacd::HaI32 k, hacd::HaI32 l, hacd::HaF32 s, hacd::HaF32 tau) 
						hacd::HaF32 g = mat[ip][j]; 
						hacd::HaF32 h = mat[iq][j]; 
						mat[ip][j] = g - s * (h + g * tau); 
						mat[iq][j] = h + s * (g - h * tau);
					}

					for (hacd::HaI32 j = 0; j < 3; j ++) {
						//ROT (eigenVectors, j, ip, j, iq, s, tau); 
						//ROT(dgMatrix &a, hacd::HaI32 i, hacd::HaI32 j, hacd::HaI32 k, hacd::HaI32 l, hacd::HaF32 s, hacd::HaF32 tau) 
						hacd::HaF32 g = eigenVectors[j][ip]; 
						hacd::HaF32 h = eigenVectors[j][iq]; 
						eigenVectors[j][ip] = g - s * (h + g * tau); 
						eigenVectors[j][iq] = h + s * (g - h * tau);
					}
				}
			}
		}
		b[0] += z[0]; d[0] = b[0]; z[0] = hacd::HaF32 (0.0f);
		b[1] += z[1]; d[1] = b[1]; z[1] = hacd::HaF32 (0.0f);
		b[2] += z[2]; d[2] = b[2]; z[2] = hacd::HaF32 (0.0f);
	}

	eigenValues = dgVector (d[0], d[1], d[2], hacd::HaF32 (0.0f));
	*this = dgGetIdentityMatrix();
}

