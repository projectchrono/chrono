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

#ifndef __DG_CONVEXHULL_3D__
#define __DG_CONVEXHULL_3D__

#include "dgList.h"
#include "dgArray.h"
#include "dgPlane.h"
#include "dgVector.h"
#include "dgMatrix.h"
#include "dgQuaternion.h"

class dgAABBPointTree3d;

class dgConvexHull3DFace
{
	public:
	dgConvexHull3DFace();
	hacd::HaI32 m_index[3]; 
	
	private:
	hacd::HaF64 Evalue (const dgBigVector* const pointArray, const dgBigVector& point) const;
	dgBigPlane GetPlaneEquation (const dgBigVector* const pointArray) const;

	hacd::HaI32 m_mark;
	dgList<dgConvexHull3DFace>::dgListNode* m_twin[3];
	friend class dgConvexHull3d;
};

class dgHullVertex;

class dgConvexHull3d: public dgList<dgConvexHull3DFace>, public UANS::UserAllocated
{
	public:
	dgConvexHull3d(const hacd::HaF64* const vertexCloud, hacd::HaI32 strideInBytes, hacd::HaI32 count, hacd::HaF64 distTol, hacd::HaI32 maxVertexCount = 0x7fffffff);
	virtual ~dgConvexHull3d();

	hacd::HaI32 GetVertexCount() const;
	const dgBigVector* GetVertexPool() const;
	const dgBigVector& GetVertex(hacd::HaI32 i) const;

	hacd::HaF64 GetDiagonal() const;
	hacd::HaF64 RayCast (const dgBigVector& localP0, const dgBigVector& localP1) const;

	protected:
	
	dgConvexHull3d(void);
	void BuildHull (const hacd::HaF64* const vertexCloud, hacd::HaI32 strideInBytes, hacd::HaI32 count, hacd::HaF64 distTol, hacd::HaI32 maxVertexCount);

	virtual dgListNode* AddFace (hacd::HaI32 i0, hacd::HaI32 i1, hacd::HaI32 i2);
	virtual void DeleteFace (dgListNode* const node) ;
//	virtual hacd::HaI32 InitVertexArray(dgBigVector* const convexPoints, dgBigVector* const points, const hacd::HaF64* const vertexCloud, hacd::HaI32 strideInBytes, hacd::HaI32 count, void* const memoryPool, hacd::HaI32 maxMemSize);
	virtual hacd::HaI32 InitVertexArray(dgHullVertex* const points, const hacd::HaF64* const vertexCloud, hacd::HaI32 strideInBytes, hacd::HaI32 count, void* const memoryPool, hacd::HaI32 maxMemSize);

	void CalculateConvexHull (dgAABBPointTree3d* vertexTree, dgHullVertex* const points, hacd::HaI32 count, hacd::HaF64 distTol, hacd::HaI32 maxVertexCount);
	hacd::HaI32 BuildNormalList (dgBigVector* const normalArray) const;
	hacd::HaI32 SupportVertex (dgAABBPointTree3d** const tree, const dgHullVertex* const points, const dgBigVector& dir) const;
	hacd::HaF64 TetrahedrumVolume (const dgBigVector& p0, const dgBigVector& p1, const dgBigVector& p2, const dgBigVector& p3) const;
	void TessellateTriangle (hacd::HaI32 level, const dgVector& p0, const dgVector& p1, const dgVector& p2, hacd::HaI32& count, dgBigVector* const ouput, hacd::HaI32& start) const;

	dgAABBPointTree3d* BuildTree (dgAABBPointTree3d* const parent, dgHullVertex* const points, hacd::HaI32 count, hacd::HaI32 baseIndex, hacd::HaI8** const memoryPool, hacd::HaI32& maxMemSize) const;
	static hacd::HaI32 ConvexCompareVertex(const dgHullVertex* const  A, const dgHullVertex* const B, void* const context);
	bool Sanity() const;

	hacd::HaI32 m_count;
	hacd::HaF64 m_diag;
	dgArray<dgBigVector> m_points;
};


inline hacd::HaI32 dgConvexHull3d::GetVertexCount() const
{
	return m_count;
}

inline const dgBigVector* dgConvexHull3d::GetVertexPool() const
{
	return &m_points[0];
}

inline const dgBigVector& dgConvexHull3d::GetVertex(hacd::HaI32 index) const
{
	return m_points[index];
}

inline hacd::HaF64 dgConvexHull3d::GetDiagonal() const
{
	return m_diag;
}

#endif
