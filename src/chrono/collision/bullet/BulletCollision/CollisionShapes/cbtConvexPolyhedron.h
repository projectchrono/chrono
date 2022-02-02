/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2011 Advanced Micro Devices, Inc.  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

///This file was written by Erwin Coumans

#ifndef _BT_POLYHEDRAL_FEATURES_H
#define _BT_POLYHEDRAL_FEATURES_H

#include "LinearMath/cbtTransform.h"
#include "LinearMath/cbtAlignedObjectArray.h"

#define TEST_INTERNAL_OBJECTS 1

struct cbtFace
{
	cbtAlignedObjectArray<int> m_indices;
	//	cbtAlignedObjectArray<int>	m_connectedFaces;
	cbtScalar m_plane[4];
};

ATTRIBUTE_ALIGNED16(class)
cbtConvexPolyhedron
{
public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	cbtConvexPolyhedron();
	virtual ~cbtConvexPolyhedron();

	cbtAlignedObjectArray<cbtVector3> m_vertices;
	cbtAlignedObjectArray<cbtFace> m_faces;
	cbtAlignedObjectArray<cbtVector3> m_uniqueEdges;

	cbtVector3 m_localCenter;
	cbtVector3 m_extents;
	cbtScalar m_radius;
	cbtVector3 mC;
	cbtVector3 mE;

	void initialize();
	void initialize2();
	bool testContainment() const;

	void project(const cbtTransform& trans, const cbtVector3& dir, cbtScalar& minProj, cbtScalar& maxProj, cbtVector3& witnesPtMin, cbtVector3& witnesPtMax) const;
};

#endif  //_BT_POLYHEDRAL_FEATURES_H
