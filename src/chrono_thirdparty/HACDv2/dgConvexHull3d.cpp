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
#include "dgStack.h"
#include "dgGoogol.h"
#include "dgConvexHull3d.h"
#include "dgSmallDeterminant.h"

#define DG_VERTEX_CLUMP_SIZE_3D		8 
class dgAABBPointTree3d
{
	public:
#ifdef _DEBUG
	dgAABBPointTree3d()
	{
		static hacd::HaI32 id = 0;
		m_id = id;
		id ++;
	}
	hacd::HaI32 m_id;
#endif

	dgBigVector m_box[2];
	dgAABBPointTree3d* m_left;
	dgAABBPointTree3d* m_right;
	dgAABBPointTree3d* m_parent;
};

class dgHullVertex: public dgBigVector
{
	public:
	hacd::HaI32 m_index;
};	

class dgAABBPointTree3dClump: public dgAABBPointTree3d
{
	public:
	hacd::HaI32 m_count;
	hacd::HaI32 m_indices[DG_VERTEX_CLUMP_SIZE_3D];
};


dgConvexHull3DFace::dgConvexHull3DFace()
{
	m_mark = 0; 
	m_twin[0] = NULL;
	m_twin[1] = NULL;
	m_twin[2] = NULL;
}

hacd::HaF64 dgConvexHull3DFace::Evalue (const dgBigVector* const pointArray, const dgBigVector& point) const
{
	const dgBigVector& p0 = pointArray[m_index[0]];
	const dgBigVector& p1 = pointArray[m_index[1]];
	const dgBigVector& p2 = pointArray[m_index[2]];

	hacd::HaF64 matrix[3][3];
	for (hacd::HaI32 i = 0; i < 3; i ++) {
		matrix[0][i] = p2[i] - p0[i];
		matrix[1][i] = p1[i] - p0[i];
		matrix[2][i] = point[i] - p0[i];
	}

	hacd::HaF64 error;
	hacd::HaF64 det = Determinant3x3 (matrix, &error);
	hacd::HaF64 precision  = hacd::HaF64 (1.0f) / hacd::HaF64 (1<<24);
	hacd::HaF64 errbound = error * precision; 
	if (fabs(det) > errbound) {
		return det;
	}

	dgGoogol exactMatrix[3][3];
	for (hacd::HaI32 i = 0; i < 3; i ++) {
		exactMatrix[0][i] = dgGoogol(p2[i]) - dgGoogol(p0[i]);
		exactMatrix[1][i] = dgGoogol(p1[i]) - dgGoogol(p0[i]);
		exactMatrix[2][i] = dgGoogol(point[i]) - dgGoogol(p0[i]);
	}

	dgGoogol exactDet (Determinant3x3(exactMatrix));
	det = exactDet.GetAproximateValue();
	return det;
}

dgBigPlane dgConvexHull3DFace::GetPlaneEquation (const dgBigVector* const pointArray) const
{
	const dgBigVector& p0 = pointArray[m_index[0]];
	const dgBigVector& p1 = pointArray[m_index[1]];
	const dgBigVector& p2 = pointArray[m_index[2]];
	dgBigPlane plane (p0, p1, p2);
	plane = plane.Scale (1.0f / sqrt (plane % plane));
	return plane;
}


dgConvexHull3d::dgConvexHull3d (void)
	:dgList<dgConvexHull3DFace>(), m_count (0), m_diag(), m_points(1024)
{
}


dgConvexHull3d::dgConvexHull3d(const hacd::HaF64* const vertexCloud, hacd::HaI32 strideInBytes, hacd::HaI32 count, hacd::HaF64 distTol, hacd::HaI32 maxVertexCount)
	:dgList<dgConvexHull3DFace>(),  m_count (0), m_diag(), m_points(count) 
{
	BuildHull (vertexCloud, strideInBytes, count, distTol, maxVertexCount);
}

dgConvexHull3d::~dgConvexHull3d(void)
{
}

void dgConvexHull3d::BuildHull (const hacd::HaF64* const vertexCloud, hacd::HaI32 strideInBytes, hacd::HaI32 count, hacd::HaF64 distTol, hacd::HaI32 maxVertexCount)
{
#if (defined (_WIN_32_VER) || defined (_WIN_64_VER))
	hacd::HaU32 controlWorld = dgControlFP (0xffffffff, 0);
	dgControlFP (_PC_53, _MCW_PC);
#endif

	hacd::HaI32 treeCount = count / (DG_VERTEX_CLUMP_SIZE_3D>>1); 
	if (treeCount < 4) {
		treeCount = 4;
	}
	treeCount *= 2;

	dgStack<dgHullVertex> points (count);
	dgStack<dgAABBPointTree3dClump> treePool (treeCount + 256);
	count = InitVertexArray(&points[0], vertexCloud, strideInBytes, count, &treePool[0], treePool.GetSizeInBytes());

	if (m_count >= 4) {
		CalculateConvexHull (&treePool[0], &points[0], count, distTol, maxVertexCount);
	}

#if (defined (_WIN_32_VER) || defined (_WIN_64_VER))
	dgControlFP (controlWorld, _MCW_PC);
#endif
}

hacd::HaI32 dgConvexHull3d::ConvexCompareVertex(const dgHullVertex* const  A, const dgHullVertex* const B, void* const context)
{
	HACD_FORCE_PARAMETER_REFERENCE(context);
	for (hacd::HaI32 i = 0; i < 3; i ++) {
		if ((*A)[i] < (*B)[i]) {
			return -1;
		} else if ((*A)[i] > (*B)[i]) {
			return 1;
		}
	}
	return 0;
}



dgAABBPointTree3d* dgConvexHull3d::BuildTree (dgAABBPointTree3d* const parent, dgHullVertex* const points, hacd::HaI32 count, hacd::HaI32 baseIndex, hacd::HaI8** memoryPool, hacd::HaI32& maxMemSize) const
{
	dgAABBPointTree3d* tree = NULL;

	HACD_ASSERT (count);
	dgBigVector minP ( hacd::HaF32 (1.0e15f),  hacd::HaF32 (1.0e15f),  hacd::HaF32 (1.0e15f), hacd::HaF32 (0.0f)); 
	dgBigVector maxP (-hacd::HaF32 (1.0e15f), -hacd::HaF32 (1.0e15f), -hacd::HaF32 (1.0e15f), hacd::HaF32 (0.0f)); 
	if (count <= DG_VERTEX_CLUMP_SIZE_3D) {

		dgAABBPointTree3dClump* const clump = new (*memoryPool) dgAABBPointTree3dClump;
		*memoryPool += sizeof (dgAABBPointTree3dClump);
		maxMemSize -= sizeof (dgAABBPointTree3dClump);
		HACD_ASSERT (maxMemSize >= 0);


		clump->m_count = count;
		for (hacd::HaI32 i = 0; i < count; i ++) {
			clump->m_indices[i] = i + baseIndex;

			const dgBigVector& p = points[i];
			minP.m_x = GetMin (p.m_x, minP.m_x); 
			minP.m_y = GetMin (p.m_y, minP.m_y); 
			minP.m_z = GetMin (p.m_z, minP.m_z); 

			maxP.m_x = GetMax (p.m_x, maxP.m_x); 
			maxP.m_y = GetMax (p.m_y, maxP.m_y); 
			maxP.m_z = GetMax (p.m_z, maxP.m_z); 
		}

		clump->m_left = NULL;
		clump->m_right = NULL;
		tree = clump;

	} else {
		dgBigVector median (hacd::HaF32 (0.0f), hacd::HaF32 (0.0f), hacd::HaF32 (0.0f), hacd::HaF32 (0.0f));
		dgBigVector varian (hacd::HaF32 (0.0f), hacd::HaF32 (0.0f), hacd::HaF32 (0.0f), hacd::HaF32 (0.0f));
		for (hacd::HaI32 i = 0; i < count; i ++) {

			const dgBigVector& p = points[i];
			minP.m_x = GetMin (p.m_x, minP.m_x); 
			minP.m_y = GetMin (p.m_y, minP.m_y); 
			minP.m_z = GetMin (p.m_z, minP.m_z); 

			maxP.m_x = GetMax (p.m_x, maxP.m_x); 
			maxP.m_y = GetMax (p.m_y, maxP.m_y); 
			maxP.m_z = GetMax (p.m_z, maxP.m_z); 

			median += p;
			varian += p.CompProduct (p);
		}

		varian = varian.Scale (hacd::HaF32 (count)) - median.CompProduct(median);
		hacd::HaI32 index = 0;
		hacd::HaF64 maxVarian = hacd::HaF64 (-1.0e10f);
		for (hacd::HaI32 i = 0; i < 3; i ++) {
			if (varian[i] > maxVarian) {
				index = i;
				maxVarian = varian[i];
			}
		}
		dgBigVector center = median.Scale (hacd::HaF64 (1.0f) / hacd::HaF64 (count));

		hacd::HaF64 test = center[index];

		hacd::HaI32 i0 = 0;
		hacd::HaI32 i1 = count - 1;
		do {    
			for (; i0 <= i1; i0 ++) {
				hacd::HaF64 val = points[i0][index];
				if (val > test) {
					break;
				}
			}

			for (; i1 >= i0; i1 --) {
				hacd::HaF64 val = points[i1][index];
				if (val < test) {
					break;
				}
			}

			if (i0 < i1)	{
				Swap(points[i0], points[i1]);
				i0++; 
				i1--;
			}
		} while (i0 <= i1);

		if (i0 == 0){
			i0 = count / 2;
		}
		if (i0 == (count - 1)){
			i0 = count / 2;
		}

		tree = new (*memoryPool) dgAABBPointTree3d;
		*memoryPool += sizeof (dgAABBPointTree3d);
		maxMemSize -= sizeof (dgAABBPointTree3d);
		HACD_ASSERT (maxMemSize >= 0);

		HACD_ASSERT (i0);
		HACD_ASSERT (count - i0);

		tree->m_left = BuildTree (tree, points, i0, baseIndex, memoryPool, maxMemSize);
		tree->m_right = BuildTree (tree, &points[i0], count - i0, i0 + baseIndex, memoryPool, maxMemSize);
	}

	HACD_ASSERT (tree);
	tree->m_parent = parent;
	tree->m_box[0] = minP - dgBigVector (hacd::HaF64 (1.0e-3f), hacd::HaF64 (1.0e-3f), hacd::HaF64 (1.0e-3f), hacd::HaF64 (1.0f));
	tree->m_box[1] = maxP + dgBigVector (hacd::HaF64 (1.0e-3f), hacd::HaF64 (1.0e-3f), hacd::HaF64 (1.0e-3f), hacd::HaF64 (1.0f));
	return tree;
}





hacd::HaI32 dgConvexHull3d::InitVertexArray(dgHullVertex* const points, const hacd::HaF64* const vertexCloud, hacd::HaI32 strideInBytes, hacd::HaI32 count, void* const memoryPool, hacd::HaI32 maxMemSize)
{
	hacd::HaI32 stride = hacd::HaI32 (strideInBytes / sizeof (hacd::HaF64));
	if (stride >= 4) {
		for (hacd::HaI32 i = 0; i < count; i ++) {
			hacd::HaI32 index = i * stride;
			dgBigVector& vertex = points[i];
			vertex = dgBigVector (vertexCloud[index], vertexCloud[index + 1], vertexCloud[index + 2], vertexCloud[index + 3]);
			HACD_ASSERT (dgCheckVector(vertex));
			points[i].m_index = 0;
		}
	} else {
		for (hacd::HaI32 i = 0; i < count; i ++) {
			hacd::HaI32 index = i * stride;
			dgBigVector& vertex = points[i];
			vertex = dgBigVector (vertexCloud[index], vertexCloud[index + 1], vertexCloud[index + 2], hacd::HaF64 (0.0f));
			HACD_ASSERT (dgCheckVector(vertex));
			points[i].m_index = 0;
		}
	}

	dgSort(points, count, ConvexCompareVertex);

	hacd::HaI32 indexCount = 0;
	for (int i = 1; i < count; i ++) {
		for (; i < count; i ++) {
			if (ConvexCompareVertex (&points[indexCount], &points[i], NULL)) {
				indexCount ++;
				points[indexCount] = points[i];
				break;
			}
		}
	}
	count = indexCount + 1;
	if (count < 4) {
		m_count = 0;
		return count;
	}

	dgAABBPointTree3d* tree = BuildTree (NULL, points, count, 0, (hacd::HaI8**) &memoryPool, maxMemSize);

	dgBigVector boxSize (tree->m_box[1] - tree->m_box[0]);	
	m_diag = hacd::HaF32 (sqrt (boxSize % boxSize));

	dgStack<dgBigVector> normalArrayPool (256);
	dgBigVector* const normalArray = &normalArrayPool[0];
	hacd::HaI32 normalCount = BuildNormalList (&normalArray[0]);

	hacd::HaI32 index = SupportVertex (&tree, points, normalArray[0]);
	m_points[0] = points[index];
	points[index].m_index = 1;

	bool validTetrahedrum = false;
	dgBigVector e1 (hacd::HaF64 (0.0f), hacd::HaF64 (0.0f), hacd::HaF64 (0.0f), hacd::HaF64 (0.0f)) ;
	for (hacd::HaI32 i = 1; i < normalCount; i ++) {
		hacd::HaI32 index = SupportVertex (&tree, points, normalArray[i]);
		HACD_ASSERT (index >= 0);

		e1 = points[index] - m_points[0];
		hacd::HaF64 error2 = e1 % e1;
		if (error2 > (hacd::HaF32 (1.0e-4f) * m_diag * m_diag)) {
			m_points[1] = points[index];
			points[index].m_index = 1;
			validTetrahedrum = true;
			break;
		}
	}
	if (!validTetrahedrum) {
		m_count = 0;
		HACD_ASSERT (0);
		return count;
	}

	validTetrahedrum = false;
	dgBigVector e2(hacd::HaF32 (0.0f), hacd::HaF32 (0.0f), hacd::HaF32 (0.0f), hacd::HaF32 (0.0f));;
	dgBigVector normal (hacd::HaF32 (0.0f), hacd::HaF32 (0.0f), hacd::HaF32 (0.0f), hacd::HaF32 (0.0f));
	for (hacd::HaI32 i = 2; i < normalCount; i ++) {
		hacd::HaI32 index = SupportVertex (&tree, points, normalArray[i]);
		HACD_ASSERT (index >= 0);
		e2 = points[index] - m_points[0];
		normal = e1 * e2;
		hacd::HaF64 error2 = sqrt (normal % normal);
		if (error2 > (hacd::HaF32 (1.0e-4f) * m_diag * m_diag)) {
			m_points[2] = points[index];
			points[index].m_index = 1;
			validTetrahedrum = true;
			break;
		}
	}

	if (!validTetrahedrum) {
		m_count = 0;
		HACD_ASSERT (0);
		return count;
	}

	// find the largest possible tetrahedron
	validTetrahedrum = false;
	dgBigVector e3(hacd::HaF32 (0.0f), hacd::HaF32 (0.0f), hacd::HaF32 (0.0f), hacd::HaF32 (0.0f));

	index = SupportVertex (&tree, points, normal);
	e3 = points[index] - m_points[0];
	hacd::HaF64 error2 = normal % e3;
	if (fabs (error2) > (hacd::HaF64 (1.0e-6f) * m_diag * m_diag)) {
		// we found a valid tetrahedra, about and start build the hull by adding the rest of the points
		m_points[3] = points[index];
		points[index].m_index = 1;
		validTetrahedrum = true;
	}
	if (!validTetrahedrum) {
		dgVector n (normal.Scale(hacd::HaF64 (-1.0f)));
		hacd::HaI32 index = SupportVertex (&tree, points, n);
		e3 = points[index] - m_points[0];
		hacd::HaF64 error2 = normal % e3;
		if (fabs (error2) > (hacd::HaF64 (1.0e-6f) * m_diag * m_diag)) {
			// we found a valid tetrahedra, about and start build the hull by adding the rest of the points
			m_points[3] = points[index];
			points[index].m_index = 1;
			validTetrahedrum = true;
		}
	}
	if (!validTetrahedrum) {
	for (hacd::HaI32 i = 3; i < normalCount; i ++) {
		hacd::HaI32 index = SupportVertex (&tree, points, normalArray[i]);
		HACD_ASSERT (index >= 0);

		//make sure the volume of the fist tetrahedral is no negative
		e3 = points[index] - m_points[0];
		hacd::HaF64 error2 = normal % e3;
		if (fabs (error2) > (hacd::HaF64 (1.0e-6f) * m_diag * m_diag)) {
			// we found a valid tetrahedra, about and start build the hull by adding the rest of the points
			m_points[3] = points[index];
			points[index].m_index = 1;
			validTetrahedrum = true;
			break;
		}
	}
	}
	if (!validTetrahedrum) {
		// the points do not form a convex hull
		m_count = 0;
		//HACD_ASSERT (0);
		return count;
	}

	m_count = 4;
	hacd::HaF64 volume = TetrahedrumVolume (m_points[0], m_points[1], m_points[2], m_points[3]);
	if (volume > hacd::HaF64 (0.0f)) {
		Swap(m_points[2], m_points[3]);
	}
	HACD_ASSERT (TetrahedrumVolume(m_points[0], m_points[1], m_points[2], m_points[3]) < hacd::HaF64(0.0f));

	return count;
}

hacd::HaF64 dgConvexHull3d::TetrahedrumVolume (const dgBigVector& p0, const dgBigVector& p1, const dgBigVector& p2, const dgBigVector& p3) const
{
	dgBigVector p1p0 (p1 - p0);
	dgBigVector p2p0 (p2 - p0);
	dgBigVector p3p0 (p3 - p0);
	return (p1p0 * p2p0) % p3p0;
}


void dgConvexHull3d::TessellateTriangle (hacd::HaI32 level, const dgVector& p0, const dgVector& p1, const dgVector& p2, hacd::HaI32& count, dgBigVector* const ouput, hacd::HaI32& start) const
{
	if (level) {
		HACD_ASSERT (dgAbsf (p0 % p0 - hacd::HaF32 (1.0f)) < hacd::HaF32 (1.0e-4f));
		HACD_ASSERT (dgAbsf (p1 % p1 - hacd::HaF32 (1.0f)) < hacd::HaF32 (1.0e-4f));
		HACD_ASSERT (dgAbsf (p2 % p2 - hacd::HaF32 (1.0f)) < hacd::HaF32 (1.0e-4f));
		dgVector p01 (p0 + p1);
		dgVector p12 (p1 + p2);
		dgVector p20 (p2 + p0);

		p01 = p01.Scale (hacd::HaF32 (1.0f) / dgSqrt(p01 % p01));
		p12 = p12.Scale (hacd::HaF32 (1.0f) / dgSqrt(p12 % p12));
		p20 = p20.Scale (hacd::HaF32 (1.0f) / dgSqrt(p20 % p20));

		HACD_ASSERT (dgAbsf (p01 % p01 - hacd::HaF32 (1.0f)) < hacd::HaF32 (1.0e-4f));
		HACD_ASSERT (dgAbsf (p12 % p12 - hacd::HaF32 (1.0f)) < hacd::HaF32 (1.0e-4f));
		HACD_ASSERT (dgAbsf (p20 % p20 - hacd::HaF32 (1.0f)) < hacd::HaF32 (1.0e-4f));

		TessellateTriangle  (level - 1, p0,  p01, p20, count, ouput, start);
		TessellateTriangle  (level - 1, p1,  p12, p01, count, ouput, start);
		TessellateTriangle  (level - 1, p2,  p20, p12, count, ouput, start);
		TessellateTriangle  (level - 1, p01, p12, p20, count, ouput, start);

	} else {
		dgBigPlane n (p0, p1, p2);
		n = n.Scale (hacd::HaF64(1.0f) / sqrt (n % n));
		n.m_w = hacd::HaF64(0.0f);
		ouput[start] = n;
		start += 8;
		count ++;
	}
}


hacd::HaI32 dgConvexHull3d::SupportVertex (dgAABBPointTree3d** const treePointer, const dgHullVertex* const points, const dgBigVector& dir) const
{
/*
	hacd::HaF64 dist = hacd::HaF32 (-1.0e10f);
	hacd::HaI32 index = -1;
	for (hacd::HaI32 i = 0; i < count; i ++) {
		//hacd::HaF64 dist1 = dir.DotProduct4 (points[i]);
		hacd::HaF64 dist1 = dir % points[i];
		if (dist1 > dist) {
			dist = dist1;
			index = i;
		}
	}
	HACD_ASSERT (index != -1);
	return index;
*/

	#define DG_STACK_DEPTH_3D 64
	hacd::HaF64 aabbProjection[DG_STACK_DEPTH_3D];
	const dgAABBPointTree3d *stackPool[DG_STACK_DEPTH_3D];

	hacd::HaI32 index = -1;
	hacd::HaI32 stack = 1;
	stackPool[0] = *treePointer;
	aabbProjection[0] = hacd::HaF32 (1.0e20f);
	hacd::HaF64 maxProj = hacd::HaF64 (-1.0e20f); 
	hacd::HaI32 ix = (dir[0] > hacd::HaF64 (0.0f)) ? 1 : 0;
	hacd::HaI32 iy = (dir[1] > hacd::HaF64 (0.0f)) ? 1 : 0;
	hacd::HaI32 iz = (dir[2] > hacd::HaF64 (0.0f)) ? 1 : 0;
	while (stack) {
		stack--;
		hacd::HaF64 boxSupportValue = aabbProjection[stack];
		if (boxSupportValue > maxProj) {
			const dgAABBPointTree3d* const me = stackPool[stack];

			if (me->m_left && me->m_right) {
				dgBigVector leftSupportPoint (me->m_left->m_box[ix].m_x, me->m_left->m_box[iy].m_y, me->m_left->m_box[iz].m_z, hacd::HaF32 (0.0));
				hacd::HaF64 leftSupportDist = leftSupportPoint % dir;

				dgBigVector rightSupportPoint (me->m_right->m_box[ix].m_x, me->m_right->m_box[iy].m_y, me->m_right->m_box[iz].m_z, hacd::HaF32 (0.0));
				hacd::HaF64 rightSupportDist = rightSupportPoint % dir;


				if (rightSupportDist >= leftSupportDist) {
					aabbProjection[stack] = leftSupportDist;
					stackPool[stack] = me->m_left;
					stack++;
					HACD_ASSERT (stack < DG_STACK_DEPTH_3D);
					aabbProjection[stack] = rightSupportDist;
					stackPool[stack] = me->m_right;
					stack++;
					HACD_ASSERT (stack < DG_STACK_DEPTH_3D);
				} else {
					aabbProjection[stack] = rightSupportDist;
					stackPool[stack] = me->m_right;
					stack++;
					HACD_ASSERT (stack < DG_STACK_DEPTH_3D);
					aabbProjection[stack] = leftSupportDist;
					stackPool[stack] = me->m_left;
					stack++;
					HACD_ASSERT (stack < DG_STACK_DEPTH_3D);
				}

			} else {
				dgAABBPointTree3dClump* const clump = (dgAABBPointTree3dClump*) me;
				for (hacd::HaI32 i = 0; i < clump->m_count; i ++) {
					const dgHullVertex& p = points[clump->m_indices[i]];
					HACD_ASSERT (p.m_x >= clump->m_box[0].m_x);
					HACD_ASSERT (p.m_x <= clump->m_box[1].m_x);
					HACD_ASSERT (p.m_y >= clump->m_box[0].m_y);
					HACD_ASSERT (p.m_y <= clump->m_box[1].m_y);
					HACD_ASSERT (p.m_z >= clump->m_box[0].m_z);
					HACD_ASSERT (p.m_z <= clump->m_box[1].m_z);
					if (!p.m_index) {
						hacd::HaF64 dist = p % dir;
						if (dist > maxProj) {
							maxProj = dist;
							index = clump->m_indices[i];
						}
					} else {
						clump->m_indices[i] = clump->m_indices[clump->m_count - 1];
						clump->m_count = clump->m_count - 1;
						i --;
					}
				}

				if (clump->m_count == 0) {
					dgAABBPointTree3d* const parent = clump->m_parent;
					if (parent) {	
						dgAABBPointTree3d* const sibling = (parent->m_left != clump) ? parent->m_left : parent->m_right;
						HACD_ASSERT (sibling != clump);
						dgAABBPointTree3d* const grandParent = parent->m_parent;
						if (grandParent) {
							sibling->m_parent = grandParent;
							if (grandParent->m_right == parent) {
								grandParent->m_right = sibling;
							} else {
								grandParent->m_left = sibling;
							}
						} else {
							sibling->m_parent = NULL;
							*treePointer = sibling;
						}
					}
				}
			}
		}
	}

	HACD_ASSERT (index != -1);
	return index;
}


hacd::HaI32 dgConvexHull3d::BuildNormalList (dgBigVector* const normalArray) const
{
	dgVector p0 ( hacd::HaF32 (1.0f), hacd::HaF32 (0.0f), hacd::HaF32 (0.0f), hacd::HaF32 (0.0f)); 
	dgVector p1 (-hacd::HaF32 (1.0f), hacd::HaF32 (0.0f), hacd::HaF32 (0.0f), hacd::HaF32 (0.0f)); 
	dgVector p2 ( hacd::HaF32 (0.0f), hacd::HaF32 (1.0f), hacd::HaF32 (0.0f), hacd::HaF32 (0.0f)); 
	dgVector p3 ( hacd::HaF32 (0.0f),-hacd::HaF32 (1.0f), hacd::HaF32 (0.0f), hacd::HaF32 (0.0f));
	dgVector p4 ( hacd::HaF32 (0.0f), hacd::HaF32 (0.0f), hacd::HaF32 (1.0f), hacd::HaF32 (0.0f));
	dgVector p5 ( hacd::HaF32 (0.0f), hacd::HaF32 (0.0f),-hacd::HaF32 (1.0f), hacd::HaF32 (0.0f));

	hacd::HaI32 count = 0;
	hacd::HaI32 subdivitions = 1;

	hacd::HaI32 start = 0;
	TessellateTriangle  (subdivitions, p4, p0, p2, count, normalArray, start);
	start = 1;
	TessellateTriangle  (subdivitions, p5, p3, p1, count, normalArray, start);
	start = 2;
	TessellateTriangle  (subdivitions, p5, p1, p2, count, normalArray, start);
	start = 3;
	TessellateTriangle  (subdivitions, p4, p3, p0, count, normalArray, start);
	start = 4;
	TessellateTriangle  (subdivitions, p4, p2, p1, count, normalArray, start);
	start = 5;
	TessellateTriangle  (subdivitions, p5, p0, p3, count, normalArray, start);
	start = 6;
	TessellateTriangle  (subdivitions, p5, p2, p0, count, normalArray, start);
	start = 7;
	TessellateTriangle  (subdivitions, p4, p1, p3, count, normalArray, start);
	return count;
}

dgConvexHull3d::dgListNode* dgConvexHull3d::AddFace (hacd::HaI32 i0, hacd::HaI32 i1, hacd::HaI32 i2)
{
	dgListNode* const node = Append();
	dgConvexHull3DFace& face = node->GetInfo();

	face.m_index[0] = i0; 
	face.m_index[1] = i1; 
	face.m_index[2] = i2; 
	return node;
}

void dgConvexHull3d::DeleteFace (dgListNode* const node) 
{
	Remove (node);
}

bool dgConvexHull3d::Sanity() const
{
/*
	for (dgListNode* node = GetFirst(); node; node = node->GetNext()) {
		dgConvexHull3DFace* const face = &node->GetInfo();		
		for (hacd::HaI32 i = 0; i < 3; i ++) {
			dgListNode* const twinNode = face->m_twin[i];
			if (!twinNode) {
				return false;
			}

			hacd::HaI32 count = 0;
			dgListNode* me = NULL;
			dgConvexHull3DFace* const twinFace = &twinNode->GetInfo();
			for (hacd::HaI32 j = 0; j < 3; j ++) {
				if (twinFace->m_twin[j] == node) {
					count ++;
					me = twinFace->m_twin[j];
				}
			}
			if (count != 1) {
				return false;
			}
			if (me != node) {
				return false;
			}
		}
	}
*/
	return true;
}

void dgConvexHull3d::CalculateConvexHull (dgAABBPointTree3d* vertexTree, dgHullVertex* const points, hacd::HaI32 count, hacd::HaF64 distTol, hacd::HaI32 maxVertexCount)
{
	distTol = fabs (distTol) * m_diag;
	dgListNode* const f0Node = AddFace (0, 1, 2);
	dgListNode* const f1Node = AddFace (0, 2, 3);
	dgListNode* const f2Node = AddFace (2, 1, 3);
	dgListNode* const f3Node = AddFace (1, 0, 3);

	dgConvexHull3DFace* const f0 = &f0Node->GetInfo();
	dgConvexHull3DFace* const f1 = &f1Node->GetInfo();
	dgConvexHull3DFace* const f2 = &f2Node->GetInfo();
	dgConvexHull3DFace* const f3 = &f3Node->GetInfo();

	f0->m_twin[0] = (dgList<dgConvexHull3DFace>::dgListNode*)f3Node; 
	f0->m_twin[1] = (dgList<dgConvexHull3DFace>::dgListNode*)f2Node; 
	f0->m_twin[2] = (dgList<dgConvexHull3DFace>::dgListNode*)f1Node;

	f1->m_twin[0] = (dgList<dgConvexHull3DFace>::dgListNode*)f0Node; 
	f1->m_twin[1] = (dgList<dgConvexHull3DFace>::dgListNode*)f2Node; 
	f1->m_twin[2] = (dgList<dgConvexHull3DFace>::dgListNode*)f3Node;

	f2->m_twin[0] = (dgList<dgConvexHull3DFace>::dgListNode*)f0Node; 
	f2->m_twin[1] = (dgList<dgConvexHull3DFace>::dgListNode*)f3Node; 
	f2->m_twin[2] = (dgList<dgConvexHull3DFace>::dgListNode*)f1Node;

	f3->m_twin[0] = (dgList<dgConvexHull3DFace>::dgListNode*)f0Node; 
	f3->m_twin[1] = (dgList<dgConvexHull3DFace>::dgListNode*)f1Node; 
	f3->m_twin[2] = (dgList<dgConvexHull3DFace>::dgListNode*)f2Node;
	
	dgList<dgListNode*> boundaryFaces;

	boundaryFaces.Append(f0Node);
	boundaryFaces.Append(f1Node);
	boundaryFaces.Append(f2Node);
	boundaryFaces.Append(f3Node);

	dgStack<dgListNode*> stackPool(1024 + m_count);
	dgStack<dgListNode*> coneListPool(1024 + m_count);
	dgStack<dgListNode*> deleteListPool(1024 + m_count);

	dgListNode** const stack = &stackPool[0];
	dgListNode** const coneList = &stackPool[0];
	dgListNode** const deleteList = &deleteListPool[0];

	count -= 4;
	maxVertexCount -= 4;
	hacd::HaI32 currentIndex = 4;

	while (boundaryFaces.GetCount() && count && (maxVertexCount > 0)) {

		dgListNode* const faceNode = boundaryFaces.GetFirst()->GetInfo();
		dgConvexHull3DFace* const face = &faceNode->GetInfo();
		dgBigPlane planeEquation (face->GetPlaneEquation (&m_points[0]));

		hacd::HaI32 index = SupportVertex (&vertexTree, points, planeEquation);
		const dgBigVector& p = points[index];
		hacd::HaF64 dist = planeEquation.Evalue(p);

		if ((dist >= distTol) && (face->Evalue(&m_points[0], p) > hacd::HaF64(0.0f))) {
			HACD_ASSERT (Sanity());
			
			HACD_ASSERT (faceNode);
			stack[0] = faceNode;

			hacd::HaI32 stackIndex = 1;
			hacd::HaI32 deletedCount = 0;

			while (stackIndex) {
				stackIndex --;
				dgListNode* const node = stack[stackIndex];
				dgConvexHull3DFace* const face = &node->GetInfo();

				if (!face->m_mark && (face->Evalue(&m_points[0], p) > hacd::HaF64(0.0f))) { 
					#ifdef _DEBUG
					for (hacd::HaI32 i = 0; i < deletedCount; i ++) {
						HACD_ASSERT (deleteList[i] != node);
					}
					#endif

					deleteList[deletedCount] = node;
					deletedCount ++;
					HACD_ASSERT (deletedCount < hacd::HaI32 (deleteListPool.GetElementsCount()));
					face->m_mark = 1;
					for (hacd::HaI32 i = 0; i < 3; i ++) {
						dgListNode* const twinNode = (dgListNode*)face->m_twin[i];
						HACD_ASSERT (twinNode);
						dgConvexHull3DFace* const twinFace = &twinNode->GetInfo();
						if (!twinFace->m_mark) {
							stack[stackIndex] = twinNode;
							stackIndex ++;
							HACD_ASSERT (stackIndex < hacd::HaI32 (stackPool.GetElementsCount()));
						}
					}
				}
			}

//			Swap (hullVertexArray[index], hullVertexArray[currentIndex]);
			m_points[currentIndex] = points[index];
			points[index].m_index = 1;

			hacd::HaI32 newCount = 0;
			for (hacd::HaI32 i = 0; i < deletedCount; i ++) {
				dgListNode* const node = deleteList[i];
				dgConvexHull3DFace* const face = &node->GetInfo();
				HACD_ASSERT (face->m_mark == 1);
				for (hacd::HaI32 j0 = 0; j0 < 3; j0 ++) {
					dgListNode* const twinNode = face->m_twin[j0];
					dgConvexHull3DFace* const twinFace = &twinNode->GetInfo();
					if (!twinFace->m_mark) {
						hacd::HaI32 j1 = (j0 == 2) ? 0 : j0 + 1;
						dgListNode* const newNode = AddFace (currentIndex, face->m_index[j0], face->m_index[j1]);
						boundaryFaces.Addtop(newNode);

						dgConvexHull3DFace* const newFace = &newNode->GetInfo();
						newFace->m_twin[1] = twinNode;
						for (hacd::HaI32 k = 0; k < 3; k ++) {
							if (twinFace->m_twin[k] == node) {
								twinFace->m_twin[k] = newNode;
							}
						}
						coneList[newCount] = newNode;
						newCount ++;
						HACD_ASSERT (newCount < hacd::HaI32 (coneListPool.GetElementsCount()));
					}
				}
			}
			
			for (hacd::HaI32 i = 0; i < newCount - 1; i ++) {
				dgListNode* const nodeA = coneList[i];
				dgConvexHull3DFace* const faceA = &nodeA->GetInfo();
				HACD_ASSERT (faceA->m_mark == 0);
				for (hacd::HaI32 j = i + 1; j < newCount; j ++) {
					dgListNode* const nodeB = coneList[j];
					dgConvexHull3DFace* const faceB = &nodeB->GetInfo();
					HACD_ASSERT (faceB->m_mark == 0);
					if (faceA->m_index[2] == faceB->m_index[1]) {
						faceA->m_twin[2] = nodeB;
						faceB->m_twin[0] = nodeA;
						break;
					}
				}

				for (hacd::HaI32 j = i + 1; j < newCount; j ++) {
					dgListNode* const nodeB = coneList[j];
					dgConvexHull3DFace* const faceB = &nodeB->GetInfo();
					HACD_ASSERT (faceB->m_mark == 0);
					if (faceA->m_index[1] == faceB->m_index[2]) {
						faceA->m_twin[0] = nodeB;
						faceB->m_twin[2] = nodeA;
						break;
					}
				}
			}

			for (hacd::HaI32 i = 0; i < deletedCount; i ++) {
				dgListNode* const node = deleteList[i];
				boundaryFaces.Remove (node);
				DeleteFace (node); 
			}

			maxVertexCount --;
			currentIndex ++;
			count --;
		} else {
			boundaryFaces.Remove (faceNode);
		}
	}
	m_count = currentIndex;
}



hacd::HaF64 dgConvexHull3d::RayCast (const dgBigVector& localP0, const dgBigVector& localP1) const
{
	hacd::HaF64 interset = hacd::HaF32 (1.2f);

	hacd::HaF64 tE = hacd::HaF64 (0.0f);           //for the maximum entering segment parameter;
	hacd::HaF64 tL = hacd::HaF64 (1.0f);           //for the minimum leaving segment parameter;
	dgBigVector dS (localP1 - localP0); // is the segment direction vector;

	hacd::HaI32 hasHit = 0;
	for (dgListNode* node = GetFirst(); node; node = node->GetNext()) {
		const dgConvexHull3DFace* const face = &node->GetInfo();

		hacd::HaI32 i0 = face->m_index[0];
		hacd::HaI32 i1 = face->m_index[1];
		hacd::HaI32 i2 = face->m_index[2];

		const dgBigVector& p0 = m_points[i0];
		dgBigVector normal ((m_points[i1] - p0) * (m_points[i2] - p0));

		hacd::HaF64 N = -((localP0 - p0) % normal);
		hacd::HaF64 D = dS % normal;

		if (fabs(D) < hacd::HaF64 (1.0e-12f)) { // 
			if (N < hacd::HaF64 (0.0f)) {
				return hacd::HaF64 (1.2f);
			} else {
				continue; 
			}
		}

		hacd::HaF64 t = N / D;
		if (D < hacd::HaF64 (0.0f)) {
			if (t > tE) {
				tE = t;
				hasHit = 1;
//				hitNormal = normal;
			}
			if (tE > tL) {
				return hacd::HaF64 (1.2f);
			}
		} else {
			HACD_ASSERT (D >= hacd::HaF64 (0.0f));
			tL = GetMin (tL, t);
			if (tL < tE) {
				return hacd::HaF64 (1.2f);
			}
		}
	}

	if (hasHit) {
		interset = tE;	
	}

	return interset;
}


