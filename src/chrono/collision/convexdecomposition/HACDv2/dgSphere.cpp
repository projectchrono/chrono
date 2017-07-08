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
#include "dgPlane.h"
#include "dgSphere.h"
#include "dgMatrix.h"

static dgSphere identitySphere;
const dgSphere& GetIdentitySphere()
{
	return identitySphere;
}



namespace InternalSphere
{
	const hacd::HaF32 SPHERE_TOL = 0.002f;	

	static hacd::HaF32 AspectRatio (hacd::HaF32 x, hacd::HaF32 y)
	{
		hacd::HaF32 tmp;
		x = dgAbsf (x);
		y = dgAbsf (y);

		if (y < x) {
			tmp = y;
			y = x;
			x = tmp;
		}
		if (y < 1.0e-12)	{
			y = 1.0e-12f;
		}
		return x / y;
	}
	

	static void BoundingBox (const dgMatrix &matrix, const hacd::HaF32 vertex[], hacd::HaI32 stride, const hacd::HaI32 index[], hacd::HaI32 indexCount, dgVector &min, dgVector &max)
	{
	   hacd::HaF32 xmin = hacd::HaF32 (1.0e10f);
	   hacd::HaF32 ymin = hacd::HaF32 (1.0e10f);
	   hacd::HaF32 zmin = hacd::HaF32 (1.0e10f);

	   hacd::HaF32 xmax = hacd::HaF32 (-1.0e10f);
	   hacd::HaF32 ymax = hacd::HaF32 (-1.0e10f);
	   hacd::HaF32 zmax = hacd::HaF32 (-1.0e10f);

	   const hacd::HaF32*	const ptr = vertex;
	   for (hacd::HaI32 j = 0 ; j < indexCount; j ++ ) {
			hacd::HaI32 i = index[j] * stride;
			dgVector tmp (ptr[i + 0], ptr[i + 1], ptr[i + 2], hacd::HaF32 (0.0f));
			tmp = matrix.UnrotateVector (tmp);
			if (tmp.m_x < xmin) xmin = tmp.m_x;
			if (tmp.m_y < ymin) ymin = tmp.m_y;
			if (tmp.m_z < zmin) zmin = tmp.m_z;
			if (tmp.m_x > xmax) xmax = tmp.m_x;
			if (tmp.m_y > ymax) ymax = tmp.m_y;
			if (tmp.m_z > zmax) zmax = tmp.m_z;
	   }
		
	   min = dgVector (xmin, ymin, zmin, hacd::HaF32 (0.0f));
	   max = dgVector (xmax, ymax, zmax, hacd::HaF32 (0.0f));
	}

	// Compute axis aligned box
	static void BoundingBox (const dgMatrix &Mat, const hacd::HaF32 vertex[], hacd::HaI32 vertexCount, hacd::HaI32 stride, dgVector &min, dgVector &max)
	{
		hacd::HaF32 xmin = hacd::HaF32 (1.0e10f);
		hacd::HaF32 ymin = hacd::HaF32 (1.0e10f);
		hacd::HaF32 zmin = hacd::HaF32 (1.0e10f);

		hacd::HaF32 xmax = hacd::HaF32 (-1.0e10f);
		hacd::HaF32 ymax = hacd::HaF32 (-1.0e10f);
		hacd::HaF32 zmax = hacd::HaF32 (-1.0e10f);

		const hacd::HaF32* ptr = vertex;
		for (hacd::HaI32 i = 0 ; i < vertexCount; i ++ ) {
			dgVector tmp (ptr[0], ptr[1], ptr[2], hacd::HaF32 (0.0f));
			ptr += stride;
			tmp = Mat.UnrotateVector (tmp);
			if (tmp.m_x < xmin) xmin = tmp.m_x;
			if (tmp.m_y < ymin) ymin = tmp.m_y;
			if (tmp.m_z < zmin) zmin = tmp.m_z;
			if (tmp.m_x > xmax) xmax = tmp.m_x;
			if (tmp.m_y > ymax) ymax = tmp.m_y;
			if (tmp.m_z > zmax) zmax = tmp.m_z;
		}

		min = dgVector (xmin, ymin, zmin, hacd::HaF32 (0.0f));
		max = dgVector (xmax, ymax, zmax, hacd::HaF32 (0.0f));
	}


	static void Statistics (dgSphere &sphere, dgVector &eigenValues, dgVector &scaleVector, const hacd::HaF32 vertex[], const hacd::HaI32 faceIndex[], hacd::HaI32 indexCount, hacd::HaI32 stride)
	{
		dgVector var (hacd::HaF32 (0.0f), hacd::HaF32 (0.0f), hacd::HaF32 (0.0f), hacd::HaF32 (0.0f));
		dgVector cov (hacd::HaF32 (0.0f), hacd::HaF32 (0.0f), hacd::HaF32 (0.0f), hacd::HaF32 (0.0f));
		dgVector centre (hacd::HaF32 (0.0f), hacd::HaF32 (0.0f), hacd::HaF32 (0.0f), hacd::HaF32 (0.0f));
		dgVector massCenter (hacd::HaF32 (0.0f), hacd::HaF32 (0.0f), hacd::HaF32 (0.0f), hacd::HaF32 (0.0f));

		hacd::HaF64 totalArea = hacd::HaF32 (0.0f);
		const hacd::HaF32* const ptr = vertex;
		for (hacd::HaI32 i = 0; i < indexCount; i += 3) {
			hacd::HaI32 index = faceIndex[i] * stride;
			dgVector p0 (&ptr[index]);
			p0 = p0.CompProduct (scaleVector);

			index = faceIndex[i + 1] * stride;;
			dgVector p1 (&ptr[index]);
			p1 = p1.CompProduct (scaleVector);

			index = faceIndex[i + 2] * stride;;
			dgVector p2 (&ptr[index]);
			p2 = p2.CompProduct (scaleVector);

			dgVector normal ((p1 - p0) * (p2 - p0));

			hacd::HaF64 area = hacd::HaF32 (0.5f) * sqrt (normal % normal);

			centre = p0 + p1 + p2;
			centre = centre.Scale (hacd::HaF32  (1.0f / 3.0f));

			// Inertia of each point in the triangle
			hacd::HaF64 Ixx = p0.m_x * p0.m_x + p1.m_x * p1.m_x + p2.m_x * p2.m_x;	
			hacd::HaF64 Iyy = p0.m_y * p0.m_y + p1.m_y * p1.m_y + p2.m_y * p2.m_y;	
			hacd::HaF64 Izz = p0.m_z * p0.m_z + p1.m_z * p1.m_z + p2.m_z * p2.m_z;	

			hacd::HaF64 Ixy = p0.m_x * p0.m_y + p1.m_x * p1.m_y + p2.m_x * p2.m_y;	
			hacd::HaF64 Iyz = p0.m_y * p0.m_z + p1.m_y * p1.m_z + p2.m_y * p2.m_z;	
			hacd::HaF64 Ixz = p0.m_x * p0.m_z + p1.m_x * p1.m_z + p2.m_x * p2.m_z;	

			if (area > dgEPSILON * 10.0) {
				hacd::HaF64 K = area / hacd::HaF64 (12.0);
				//Coriolis theorem for Inertia of a triangle in an arbitrary orientation
				Ixx = K * (Ixx + 9.0 * centre.m_x * centre.m_x);
				Iyy = K * (Iyy + 9.0 * centre.m_y * centre.m_y);
				Izz = K * (Izz + 9.0 * centre.m_z * centre.m_z);

				Ixy = K * (Ixy + 9.0 * centre.m_x * centre.m_y);
				Ixz = K * (Ixz + 9.0 * centre.m_x * centre.m_z);
				Iyz = K * (Iyz + 9.0 * centre.m_y * centre.m_z);
				centre = centre.Scale ((hacd::HaF32)area);
			} 

			totalArea += area;
			massCenter += centre;
			var += dgVector ((hacd::HaF32)Ixx, (hacd::HaF32)Iyy, (hacd::HaF32)Izz, hacd::HaF32 (0.0f));
			cov += dgVector ((hacd::HaF32)Ixy, (hacd::HaF32)Ixz, (hacd::HaF32)Iyz, hacd::HaF32 (0.0f));
		}

		if (totalArea > dgEPSILON * 10.0) {
			hacd::HaF64 K = hacd::HaF64 (1.0) / totalArea; 
			var = var.Scale ((hacd::HaF32)K);
			cov = cov.Scale ((hacd::HaF32)K);
			massCenter = massCenter.Scale ((hacd::HaF32)K);
		}

		hacd::HaF64 Ixx = var.m_x - massCenter.m_x * massCenter.m_x;
		hacd::HaF64 Iyy = var.m_y - massCenter.m_y * massCenter.m_y;
		hacd::HaF64 Izz = var.m_z - massCenter.m_z * massCenter.m_z;

		hacd::HaF64 Ixy = cov.m_x - massCenter.m_x * massCenter.m_y;
		hacd::HaF64 Ixz = cov.m_y - massCenter.m_x * massCenter.m_z;
		hacd::HaF64 Iyz = cov.m_z - massCenter.m_y * massCenter.m_z;

		sphere.m_front = dgVector ((hacd::HaF32)Ixx, (hacd::HaF32)Ixy, (hacd::HaF32)Ixz, hacd::HaF32 (0.0f));
		sphere.m_up    = dgVector ((hacd::HaF32)Ixy, (hacd::HaF32)Iyy, (hacd::HaF32)Iyz, hacd::HaF32 (0.0f));
		sphere.m_right = dgVector ((hacd::HaF32)Ixz, (hacd::HaF32)Iyz, (hacd::HaF32)Izz, hacd::HaF32 (0.0f));
		sphere.EigenVectors(eigenValues);
	}

	
	static void Statistics (dgSphere &sphere, dgVector &eigenValues, dgVector &scaleVector, const hacd::HaF32 vertex[], hacd::HaI32 vertexCount, hacd::HaI32 stride)
	{
		dgBigVector var (hacd::HaF32 (0.0f), hacd::HaF32 (0.0f), hacd::HaF32 (0.0f), hacd::HaF32 (0.0f));
		dgBigVector cov (hacd::HaF32 (0.0f), hacd::HaF32 (0.0f), hacd::HaF32 (0.0f), hacd::HaF32 (0.0f));
		dgBigVector massCenter (hacd::HaF32 (0.0f), hacd::HaF32 (0.0f), hacd::HaF32 (0.0f), hacd::HaF32 (0.0f));
	
		const hacd::HaF32* ptr = vertex;
		for (hacd::HaI32 i = 0; i < vertexCount; i ++) {
			hacd::HaF32 x = ptr[0] * scaleVector.m_x;
			hacd::HaF32 y = ptr[1] * scaleVector.m_y;
			hacd::HaF32 z = ptr[2] * scaleVector.m_z;
			ptr += stride;
			massCenter += dgBigVector (x, y, z, hacd::HaF32 (0.0f));
			var += dgBigVector (x * x, y * y, z * z, hacd::HaF32 (0.0f));
			cov += dgBigVector (x * y, x * z, y * z, hacd::HaF32 (0.0f));
		}
	
		hacd::HaF64 k = hacd::HaF64 (1.0) / vertexCount;
		var = var.Scale (k);
		cov = cov.Scale (k);
		massCenter = massCenter.Scale (k);
	
		hacd::HaF64 Ixx = var.m_x - massCenter.m_x * massCenter.m_x;
		hacd::HaF64 Iyy = var.m_y - massCenter.m_y * massCenter.m_y;
		hacd::HaF64 Izz = var.m_z - massCenter.m_z * massCenter.m_z;
	
		hacd::HaF64 Ixy = cov.m_x - massCenter.m_x * massCenter.m_y;
		hacd::HaF64 Ixz = cov.m_y - massCenter.m_x * massCenter.m_z;
		hacd::HaF64 Iyz = cov.m_z - massCenter.m_y * massCenter.m_z;
	
		sphere.m_front = dgVector (hacd::HaF32(Ixx), hacd::HaF32(Ixy), hacd::HaF32(Ixz), hacd::HaF32 (0.0f));
		sphere.m_up    = dgVector (hacd::HaF32(Ixy), hacd::HaF32(Iyy), hacd::HaF32(Iyz), hacd::HaF32 (0.0f));
		sphere.m_right = dgVector (hacd::HaF32(Ixz), hacd::HaF32(Iyz), hacd::HaF32(Izz), hacd::HaF32 (0.0f));
 		sphere.EigenVectors (eigenValues);
	}

/*
	static void Statistics (
		dgSphere &sphere,
		dgVector &eigenValues,
		const dgVector &scaleVector,
		const hacd::HaF32 vertex[],
		hacd::HaI32 stride,
		const dgFace face[],
		hacd::HaI32 faceCount)
	{
		HACD_ASSERT (0);

		hacd::HaI32 i;
		hacd::HaI32 index;
		const hacd::HaF32 *ptr;
		hacd::HaF64 K;
		hacd::HaF64 Ixx;
		hacd::HaF64 Iyy;
		hacd::HaF64 Izz;
		hacd::HaF64 Ixy;
		hacd::HaF64 Ixz;
		hacd::HaF64 Iyz;
		hacd::HaF64 area;
		hacd::HaF64 totalArea;
		const dgFace *Face;
	
		dgVector var (hacd::HaF32 (0.0f), hacd::HaF32 (0.0f), hacd::HaF32 (0.0f));
		dgVector cov (hacd::HaF32 (0.0f), hacd::HaF32 (0.0f), hacd::HaF32 (0.0f));
		dgVector centre (hacd::HaF32 (0.0f), hacd::HaF32 (0.0f), hacd::HaF32 (0.0f));
		dgVector massCenter (hacd::HaF32 (0.0f), hacd::HaF32 (0.0f), hacd::HaF32 (0.0f));
		
		totalArea = 0.0;
		ptr = vertex;
		for (i = 0; i < faceCount; i ++) {
			 Face = &face[i];
		
			 index = Face->m_wireFrame[0] * stride;
			 dgVector p0 (&ptr[index]);
			 p0 = p0.CompProduct (scaleVector);
	
			 index = Face->m_wireFrame[1] * stride;
			 dgVector p1 (&ptr[index]);
			 p1 = p1.CompProduct (scaleVector);
		
			 index = Face->m_wireFrame[2] * stride;
			 dgVector p2 (&ptr[index]);
			 p2 = p2.CompProduct (scaleVector);
		
			 dgVector normal ((p1 - p0) * (p2 - p0));
		
			 area = 0.5 * sqrt (normal % normal);
		
			 centre = p0 + p1 + p2;
			 centre = centre.Scale (1.0f / 3.0f);
		
			 // Inercia of each point in the triangle
			 Ixx = p0.m_x * p0.m_x + p1.m_x * p1.m_x + p2.m_x * p2.m_x;	
			 Iyy = p0.m_y * p0.m_y + p1.m_y * p1.m_y + p2.m_y * p2.m_y;	
			 Izz = p0.m_z * p0.m_z + p1.m_z * p1.m_z + p2.m_z * p2.m_z;	
		
			 Ixy = p0.m_x * p0.m_y + p1.m_x * p1.m_y + p2.m_x * p2.m_y;	
			 Iyz = p0.m_y * p0.m_z + p1.m_y * p1.m_z + p2.m_y * p2.m_z;	
			 Ixz = p0.m_x * p0.m_z + p1.m_x * p1.m_z + p2.m_x * p2.m_z;	
		
			 if (area > dgEPSILON * 10.0) {
			     K = area / 12.0;
			     //Coriollis teorem for Inercia of a triangle in an arbitrary orientation
			     Ixx = K * (Ixx + 9.0 * centre.m_x * centre.m_x);
			     Iyy = K * (Iyy + 9.0 * centre.m_y * centre.m_y);
			     Izz = K * (Izz + 9.0 * centre.m_z * centre.m_z);
		
			     Ixy = K * (Ixy + 9.0 * centre.m_x * centre.m_y);
			     Ixz = K * (Ixz + 9.0 * centre.m_x * centre.m_z);
			     Iyz = K * (Iyz + 9.0 * centre.m_y * centre.m_z);
			     centre = centre.Scale ((hacd::HaF32)area);
			 } 
		
			 totalArea += area;
			 massCenter += centre;
			 var += dgVector ((hacd::HaF32)Ixx, (hacd::HaF32)Iyy, (hacd::HaF32)Izz);
			 cov += dgVector ((hacd::HaF32)Ixy, (hacd::HaF32)Ixz, (hacd::HaF32)Iyz);
		}
		
		if (totalArea > dgEPSILON * 10.0) {
			K = 1.0 / totalArea; 
			var = var.Scale ((hacd::HaF32)K);
			cov = cov.Scale ((hacd::HaF32)K);
			massCenter = massCenter.Scale ((hacd::HaF32)K);
		}
	
		Ixx = var.m_x - massCenter.m_x * massCenter.m_x;
		Iyy = var.m_y - massCenter.m_y * massCenter.m_y;
		Izz = var.m_z - massCenter.m_z * massCenter.m_z;
		
		Ixy = cov.m_x - massCenter.m_x * massCenter.m_y;
		Ixz = cov.m_y - massCenter.m_x * massCenter.m_z;
		Iyz = cov.m_z - massCenter.m_y * massCenter.m_z;
	
		sphere.m_front = dgVector ((hacd::HaF32)Ixx, (hacd::HaF32)Ixy, (hacd::HaF32)Ixz);
		sphere.m_up    = dgVector ((hacd::HaF32)Ixy, (hacd::HaF32)Iyy, (hacd::HaF32)Iyz);
		sphere.m_right = dgVector ((hacd::HaF32)Ixz, (hacd::HaF32)Iyz, (hacd::HaF32)Izz);
		sphere.EigenVectors(eigenValues);
	}
*/
}


dgSphere::dgSphere ()
	:dgMatrix(dgGetIdentityMatrix()), m_size (0, 0, 0, 0)
{
//	HACD_ASSERT (0);
//	planeTest = FrontTest;
}

dgSphere::dgSphere (const dgQuaternion &quat, const dgVector &position, const dgVector& dim)
	:dgMatrix(quat, position)
{
   SetDimensions (dim.m_x, dim.m_y, dim.m_z);
   HACD_ASSERT (0);
//	planeTest = FrontTest;
}

dgSphere::dgSphere(const dgMatrix &matrix, const dgVector& dim)
	:dgMatrix(matrix)
{
   SetDimensions (dim.m_x, dim.m_y, dim.m_z);
//   HACD_ASSERT (0);
//	planeTest = FrontTest;
}



void dgSphere::SetDimensions (const hacd::HaF32 vertex[], hacd::HaI32 strideInBytes, const hacd::HaI32 triangles[], hacd::HaI32 indexCount, const dgMatrix *basis)
{
	dgVector eigen;
	dgVector scaleVector (hacd::HaF32 (1.0f), hacd::HaF32 (1.0f), hacd::HaF32 (1.0f), hacd::HaF32 (0.0f));

	if (indexCount < 3) {
		return;
	}

	hacd::HaI32 stride = hacd::HaI32 (strideInBytes / sizeof (hacd::HaF32));
	if (!basis)	{

		InternalSphere::Statistics (*this, eigen, scaleVector, vertex, triangles, indexCount, stride);

		hacd::HaI32 k = 0;
		for (hacd::HaI32 i = 0; i < 3; i ++) {
			if (k >= 6) {
				 break;
			}
			for (hacd::HaI32 j = i + 1; j < 3; j ++) {
				hacd::HaF32 aspect = InternalSphere::AspectRatio (eigen[i], eigen[j]);
				if (aspect > hacd::HaF32 (0.9f)) {
					scaleVector[i] *= hacd::HaF32 (2.0f); 
					InternalSphere::Statistics (*this, eigen, scaleVector, vertex, triangles, indexCount, stride);
					k ++;
					i = -1;
					break;
				}
			}
		}
	} else {
		*this = *basis;
	}

	dgVector min; 
	dgVector max;
	InternalSphere::BoundingBox (*this, vertex, stride, triangles, indexCount, min, max);

	dgVector massCenter	(max + min);
	massCenter = massCenter.Scale (hacd::HaF32 (0.5f));
	m_posit = TransformVector (massCenter);

	dgVector dim (max - min);
	dim = dim.Scale (hacd::HaF32(0.5f));
	SetDimensions (dim.m_x, dim.m_y, dim.m_z);
}


void dgSphere::SetDimensions (const hacd::HaF32 vertex[], hacd::HaI32 strideInBytes, hacd::HaI32 count, const dgMatrix *basis) 
{
	dgVector eigen;
	dgVector scaleVector (hacd::HaF32(1.0f), hacd::HaF32(1.0f), hacd::HaF32(1.0f), hacd::HaF32 (0.0f));

	hacd::HaI32 stride = hacd::HaI32 (strideInBytes / sizeof (hacd::HaF32));
	if (!basis)	{
		InternalSphere::Statistics (*this, eigen, scaleVector, vertex, count, stride);

		hacd::HaI32 k = 0;
		for (hacd::HaI32 i = 0; i < 3; i ++) {
			if (k >= 6) {
				 break;
			}
			for (hacd::HaI32 j = i + 1; j < 3; j ++) {
				hacd::HaF32 aspect = InternalSphere::AspectRatio (eigen[i], eigen[j]);
				if (aspect > hacd::HaF32 (0.9f)) {
					scaleVector[i] *= hacd::HaF32 (2.0f); 
					InternalSphere::Statistics (*this, eigen, scaleVector, vertex, count, stride);
					k ++;
					i = -1;
					break;
				}
			}
		}
	} else {
		*this = *basis;
	}
		
   dgVector min; 
   dgVector max;
   InternalSphere::BoundingBox (*this, vertex, count, stride, min, max);

	dgVector massCenter (max + min);
	massCenter = massCenter.Scale (0.5);
	m_posit = TransformVector (massCenter);

	dgVector dim (max - min);
	dim = dim.Scale (hacd::HaF32(0.5f));
	SetDimensions (dim.m_x + InternalSphere::SPHERE_TOL, 
				   dim.m_y + InternalSphere::SPHERE_TOL, 
				   dim.m_z + InternalSphere::SPHERE_TOL);

}

/*
void dgSphere::SetDimensions (
	const hacd::HaF32 vertex[],  
	hacd::HaI32 strideInBytes,
	const hacd::HaI32 index[],
	hacd::HaI32 indexCount,
	const dgMatrix *basis)
{
	hacd::HaI32 i;
	hacd::HaI32 j;
	hacd::HaI32 k;
	hacd::HaI32 stride;
	hacd::HaF32 aspect;
	dgVector eigen;
	dgVector scaleVector (hacd::HaF32(1.0f), hacd::HaF32(1.0f), hacd::HaF32(1.0f), hacd::HaF32 (0.0f));

	stride = strideInBytes / sizeof (hacd::HaF32);
	if (!basis)	{
		InternalSphere::Statistics (*this, eigen, scaleVector, vertex, index, indexCount, stride);

		k = 0;
		for (i = 0; i < 3; i ++) {
			if (k >= 6) {
				 break;
			}
			for (j = i + 1; j < 3; j ++) {
				aspect = InternalSphere::AspectRatio (eigen[i], eigen[j]);
				if (aspect > hacd::HaF32 (0.9f)) {
					scaleVector[i] *= hacd::HaF32 (2.0f); 
					InternalSphere::Statistics (*this, eigen, scaleVector, vertex, index, indexCount, stride);
					i = -1;
					k ++;
					break;
				}
			}
		}
	} else {
		*this = *basis;
	}
		
   dgVector min; 
   dgVector max;
   InternalSphere::BoundingBox (*this, vertex, stride, index, indexCount, min, max);

	dgVector massCenter (max + min);
	massCenter = massCenter.Scale (hacd::HaF32(0.5f));
	m_posit = TransformVector (massCenter);

	dgVector dim (max - min);
	dim = dim.Scale (hacd::HaF32(0.5f));
	SetDimensions (dim.m_x + InternalSphere::SPHERE_TOL, 
				   dim.m_y + InternalSphere::SPHERE_TOL, 
				   dim.m_z + InternalSphere::SPHERE_TOL);
}
*/



/*
dgSphere::dgSphere (
	const dgSphere &dgSphere, 
	const dgVector &Dir)
{
	if ((Dir % Dir) < EPSILON * 0.01f) {
		*this = dgSphere;
		return;
	}

	front = Dir;
	front.Fast_Normalize(); 

	if (dgAbsf (front % dgSphere.right) < 0.995) {
		up = front * dgSphere.right;
		up.Fast_Normalize(); 
	} else {
		up	= dgSphere.up;
	}
	right = up * front;

	dgVector Step (Dir.Scale(0.5));
	size.m_x = (hacd::HaF32)(dgSphere.size.m_x * dgAbsf (right % dgSphere.right) +
						 dgSphere.size.m_y * dgAbsf (right % dgSphere.up) +
						 dgSphere.size.m_z * dgAbsf (right % dgSphere.front));

	size.m_y = (hacd::HaF32)(dgSphere.size.m_x * dgAbsf (up % dgSphere.right) +
						  dgSphere.size.m_y * dgAbsf (up % dgSphere.up) +
						  dgSphere.size.m_z * dgAbsf (up % dgSphere.front));

	size.m_z = (hacd::HaF32)(sqrt (Step % Step) +
							dgSphere.size.m_x * dgAbsf (front % dgSphere.right) +
							dgSphere.size.m_y * dgAbsf (front % dgSphere.up) +
							dgSphere.size.m_z * dgAbsf (front % dgSphere.front));
	posit = dgSphere.posit + Step;

}



bool dgSphere::dgSphere_Overlap_Test (const dgSphere &dgSphere)
{
	hacd::HaF64 R;
	dgVector Dir (dgSphere.posit - posit);

  	R = size.m_x * dgAbsf (right % Dir) + dgSphere.size.m_x * dgAbsf (dgSphere.right % Dir) + 
	    size.m_y * dgAbsf (up % Dir) + dgSphere.size.m_y * dgAbsf (dgSphere.up % Dir) + 
		 size.m_z * dgAbsf (front %Dir) + dgSphere.size.m_z * dgAbsf (dgSphere.front % Dir);
	if (R	< (Dir % Dir)) {
		return false;
	}

	R = size.m_x * dgAbsf (right % dgSphere.right) + 
	    size.m_y * dgAbsf (up % dgSphere.right) + 
		 size.m_z * dgAbsf (front % dgSphere.right) + dgSphere.size.m_x;
	if (R	< dgAbsf (Dir % dgSphere.right)) {
		return false;
	}

	R = size.m_x * dgAbsf (right % dgSphere.up) + 
	    size.m_y * dgAbsf (up % dgSphere.up) + 
		 size.m_z * dgAbsf (front % dgSphere.up) + dgSphere.size.m_y;
	if (R	< dgAbsf (Dir % dgSphere.up)) {
		return false;
	}

	R = size.m_x * dgAbsf (right % dgSphere.front) + 
	    size.m_y * dgAbsf (up % dgSphere.front) + 
		 size.m_z * dgAbsf (front % dgSphere.front) + dgSphere.size.m_z; 
	if (R	< dgAbsf (Dir % dgSphere.front)) {
		return false;
	}

	R = dgSphere.size.m_x * dgAbsf (dgSphere.right % right) + 
	    dgSphere.size.m_y * dgAbsf (dgSphere.up % right) + 
		 dgSphere.size.m_z * dgAbsf (dgSphere.front % right) + size.m_x;
	if (R	< dgAbsf (Dir % right)) {
		return false;
	}

	R = dgSphere.size.m_x * dgAbsf (dgSphere.right % up) + 
	    dgSphere.size.m_y * dgAbsf (dgSphere.up % up) + 
		 dgSphere.size.m_z * dgAbsf (dgSphere.front % up) + size.m_y;
	if (R	< dgAbsf (Dir % up)) {
		return false;
	}

	R = dgSphere.size.m_x * dgAbsf (dgSphere.right % front) + 
	    dgSphere.size.m_y * dgAbsf (dgSphere.up % front) + 
		 dgSphere.size.m_z * dgAbsf (dgSphere.front % front) + size.m_z;
	if (R	< dgAbsf (Dir % front)) {
		return false;
	}

	return true;
}


void dgSphere::Swept_Volume (
	dgVector &min, 
	dgVector &max)
{
	hacd::HaF32 w;
	hacd::HaF32 h;
	hacd::HaF32 b;

	w = (hacd::HaF32)(size.m_x * dgAbsf(right.m_x) + size.m_y * dgAbsf(up.m_x) + size.m_z * dgAbsf(front.m_x));  
	h = (hacd::HaF32)(size.m_x * dgAbsf(right.m_y) + size.m_y * dgAbsf(up.m_y) + size.m_z * dgAbsf(front.m_y));  
	b = (hacd::HaF32)(size.m_x * dgAbsf(right.m_z) + size.m_y * dgAbsf(up.m_z) + size.m_z * dgAbsf(front.m_z));  

	min.m_x = posit.m_x - w;
	min.m_y = posit.m_y - h;
	min.m_z = posit.m_z - b;

	max.m_x = posit.m_x + w;
	max.m_y = posit.m_y + h;
	max.m_z = posit.m_z + b;
}
*/


/*
hacd::HaI32 dgSphere::FrontTest (
	const dgMatrix &matrix, 
	const dgPlane* plane) const
{
	hacd::HaF32 R;
	hacd::HaF32 dR;
	InternalSphere::dgFloatSign flag0;
	InternalSphere::dgFloatSign flag1;

	dR = m_size.m_x * dgAbsf (matrix.m_front.m_x) + m_size.m_y * dgAbsf (matrix.m_up.m_x) + m_size.m_z * dgAbsf (matrix.m_right.m_x);
	R = plane[5].m_x * matrix.m_posit.m_x + plane[5].m_w; 

	flag0.f = R + dR;
	flag1.f = R - dR;
	flag0.i = flag0.i >> 30 & 2;
	flag1.i = flag1.i >> 31 & 1;
	return InternalSphere::CodeTbl[flag0.i | flag1.i];
}

hacd::HaI32 dgSphere::RearTest (const dgMatrix &matrix, const dgPlane* plane) const
{
	hacd::HaF32 R;
	hacd::HaF32 dR;
	InternalSphere::dgFloatSign flag0;
	InternalSphere::dgFloatSign flag1;

	dR = m_size.m_x * dgAbsf (matrix.m_front.m_x) + m_size.m_y * dgAbsf (matrix.m_up.m_x) + m_size.m_z * dgAbsf (matrix.m_right.m_x);
	R = plane[4].m_x * matrix.m_posit.m_x + plane[4].m_w; 

	flag0.f = R + dR;
	flag1.f = R - dR;
	flag0.i = flag0.i >> 30 & 2;
	flag1.i = flag1.i >> 31 & 1;
	return InternalSphere::CodeTbl[flag0.i | flag1.i];
}


hacd::HaI32 dgSphere::LeftTest (const dgMatrix &matrix, const dgPlane* plane) const
{
	hacd::HaF32 R;
	hacd::HaF32 dR;
	InternalSphere::dgFloatSign flag0;
	InternalSphere::dgFloatSign flag1;

	dR = m_size.m_x * dgAbsf (matrix.m_front.m_x * plane[0].m_x + matrix.m_front.m_z * plane[0].m_z) + 
		 m_size.m_y * dgAbsf (matrix.m_up.m_x    * plane[0].m_x + matrix.m_up.m_z    * plane[0].m_z) +
		 m_size.m_z * dgAbsf (matrix.m_right.m_x * plane[0].m_x + matrix.m_right.m_z * plane[0].m_z);
	R = plane[0].m_x * matrix.m_posit.m_x + plane[0].m_z * matrix.m_posit.m_z; 

	flag0.f = R + dR;
	flag1.f = R - dR;
	flag0.i = (flag0.i >> 30) & 2;
	flag1.i = (flag1.i >> 31) & 1;
	return InternalSphere::CodeTbl[flag0.i | flag1.i];
}

hacd::HaI32 dgSphere::RightTest (const dgMatrix &matrix, const dgPlane* plane) const
{
	hacd::HaF32 R;
	hacd::HaF32 dR;
	InternalSphere::dgFloatSign flag0;
	InternalSphere::dgFloatSign flag1;

	dR = m_size.m_x * dgAbsf (matrix.m_front.m_x * plane[1].m_x + matrix.m_front.m_z * plane[1].m_z) + 
		 m_size.m_y * dgAbsf (matrix.m_up.m_x    * plane[1].m_x + matrix.m_up.m_z    * plane[1].m_z) +
		 m_size.m_z * dgAbsf (matrix.m_right.m_x * plane[1].m_x + matrix.m_right.m_z * plane[1].m_z);
	R = plane[1].m_x * matrix.m_posit.m_x + plane[1].m_z * matrix.m_posit.m_z; 

	flag0.f = R + dR;
	flag1.f = R - dR;
	flag0.i = (flag0.i >> 30) & 2;
	flag1.i = (flag1.i >> 31) & 1;
	return InternalSphere::CodeTbl[flag0.i | flag1.i];
}

hacd::HaI32 dgSphere::BottomTest (const dgMatrix &matrix, const dgPlane* plane) const
{
	hacd::HaF32 R;
	hacd::HaF32 dR;
	InternalSphere::dgFloatSign flag0;
	InternalSphere::dgFloatSign flag1;

	dR = m_size.m_x * dgAbsf (matrix.m_front.m_x * plane[2].m_x + matrix.m_front.m_y * plane[2].m_y) + 
		 m_size.m_y * dgAbsf (matrix.m_up.m_x    * plane[2].m_x + matrix.m_up.m_y    * plane[2].m_y) +
		 m_size.m_z * dgAbsf (matrix.m_right.m_x * plane[2].m_x + matrix.m_right.m_y * plane[2].m_y);

	R = plane[2].m_x * matrix.m_posit.m_x + plane[2].m_y * matrix.m_posit.m_y; 

	flag0.f = R + dR;
	flag1.f = R - dR;
	flag0.i = (flag0.i >> 30) & 2;
	flag1.i = (flag1.i >> 31) & 1;

	return InternalSphere::CodeTbl[flag0.i | flag1.i];
}

hacd::HaI32 dgSphere::TopTest (const dgMatrix &matrix, const dgPlane* plane) const
{
	hacd::HaF32 R;
	hacd::HaF32 dR;
	InternalSphere::dgFloatSign flag0;
	InternalSphere::dgFloatSign flag1;

	dR = m_size.m_x * dgAbsf (matrix.m_front.m_x * plane[3].m_x + matrix.m_front.m_y * plane[3].m_y) + 
		 m_size.m_y * dgAbsf (matrix.m_up.m_x    * plane[3].m_x + matrix.m_up.m_y    * plane[3].m_y) +
		 m_size.m_z * dgAbsf (matrix.m_right.m_x * plane[3].m_x + matrix.m_right.m_y * plane[3].m_y);

	R = plane[3].m_x * matrix.m_posit.m_x + plane[3].m_y * matrix.m_posit.m_y; 

	flag0.f = R + dR;
	flag1.f = R - dR;
	flag0.i = (flag0.i >> 30) & 2;
	flag1.i = (flag1.i >> 31) & 1;
	return InternalSphere::CodeTbl[flag0.i | flag1.i];
}



hacd::HaI32 dgSphere::VisibilityTestLow (
	const dgCamera* camera,
	const dgMatrix& matrix) const
{
	hacd::HaI32 i;
	hacd::HaI32 code;
	const dgPlane* planes;
	const dgPlane* guardPlanes;

	planes = camera->GetViewVolume();

	code = (this->*planeTest) (matrix, planes);
	if (code != -1) {
		for (i = 0; i < 6; i ++) {
			code |= (this->*planeTestArray[i]) (matrix, planes);
			if (code == -1) {
				planeTest = planeTestArray[i];
				return -1;
			}
		}

		if (code) {
			guardPlanes = camera->GetGuardViewVolume();
			if (guardPlanes) {
				code = 0;
				for (i = 0; i < 6; i ++) {
					code |= (this->*planeTestArray[i]) (matrix, guardPlanes);
					HACD_ASSERT (code >= 0);
					if (code) {
						return code;
					}
				}
			}
		}
	}

   return code;
}


hacd::HaI32 dgSphere::VisibilityTest (const dgCamera* camera) const
{
	dgMatrix viewMatrix (*this * camera->GetViewMatrix());
	return VisibilityTestLow (camera, viewMatrix);
}

hacd::HaI32 dgSphere::VisibilityTest (const dgCamera* camera, const dgMatrix &worldMatrix) const 
{
	dgMatrix viewMatrix (*this * worldMatrix * camera->GetViewMatrix());
	return VisibilityTestLow (camera, viewMatrix);
}

void dgSphere::Render (
	const dgCamera* camera, 
	const dgMatrix &worldMatrix, 
	dgUnsigned32 rgb) const
{
	hacd::HaI32 i;
	struct ColorVertex
	{
		hacd::HaF32 m_x;
		hacd::HaF32 m_y;
		hacd::HaF32 m_z;
		dgColor m_color;
	};

	dgUnsigned32 index [][2] = {
		{0, 4}, {1, 5}, {2, 6}, {3, 7},
		{0, 1}, {4, 5}, {7, 6}, {3, 2},
		{1, 2}, {5, 6}, {4, 7}, {0, 3},
	};

	ColorVertex* ptr;
	ColorVertex box[8];

	box[0].m_x = -size.m_x;
	box[0].m_y = -size.m_y;
	box[0].m_z = -size.m_z;
	box[0].m_color.m_val = rgb;

	box[1].m_x =  size.m_x;
	box[1].m_y = -size.m_y;
	box[1].m_z = -size.m_z;
	box[1].m_color.m_val = rgb;

	box[2].m_x =  size.m_x;
	box[2].m_y = -size.m_y;
	box[2].m_z =  size.m_z;
	box[2].m_color.m_val = rgb;

	box[3].m_x = -size.m_x;
	box[3].m_y = -size.m_y;
	box[3].m_z =  size.m_z;
	box[3].m_color.m_val = rgb;

	box[4].m_x = -size.m_x;
	box[4].m_y =  size.m_y;
	box[4].m_z = -size.m_z;
	box[4].m_color.m_val = rgb;

	box[5].m_x =  size.m_x;
	box[5].m_y =  size.m_y;
	box[5].m_z = -size.m_z;
	box[5].m_color.m_val = rgb;

	box[6].m_x =  size.m_x;
	box[6].m_y =  size.m_y;
	box[6].m_z =  size.m_z;
	box[6].m_color.m_val = rgb;

	box[7].m_x = -size.m_x;
	box[7].m_y =  size.m_y;
	box[7].m_z =  size.m_z;
	box[7].m_color.m_val = rgb;

	dgRenderDescriptorParams param;
	param.m_indexCount = 0;
	param.m_vertexCount = sizeof (index) / sizeof (hacd::HaI32); 
	param.m_descType = dgDynamicVertex;
	param.m_primitiveType = RENDER_LINELIST;
	param.m_vertexFlags = VERTEX_ENABLE_XYZ | COLOR_ENABLE;

	dgRenderDescriptor desc (param);

	dgMatrix tmpMat (*this * worldMatrix);
	camera->SetWorldMatrix (&tmpMat);

	desc.m_material = dgMaterial::UseDebugMaterial();

  	dgVertexRecord vertexRecord (desc.LockVertex());
	ptr = (ColorVertex*) vertexRecord.vertex.ptr;
	for (i = 0; i < (sizeof (index) / (2 * sizeof (dgUnsigned32))); i ++) {
		 ptr[0] = box[index[i][0]]; 
		 ptr[1] = box[index[i][1]]; 
		 ptr += 2;
	}
	desc.UnlockVertex();

	camera->Render (desc);

	desc.m_material->Release();
}
*/

