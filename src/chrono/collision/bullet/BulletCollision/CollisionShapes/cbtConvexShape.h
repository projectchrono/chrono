/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_CONVEX_SHAPE_INTERFACE1
#define BT_CONVEX_SHAPE_INTERFACE1

#include "cbtCollisionShape.h"

#include "LinearMath/cbtVector3.h"
#include "LinearMath/cbtTransform.h"
#include "LinearMath/cbtMatrix3x3.h"
#include "cbtCollisionMargin.h"
#include "LinearMath/cbtAlignedAllocator.h"

#define MAX_PREFERRED_PENETRATION_DIRECTIONS 10

/// The cbtConvexShape is an abstract shape interface, implemented by all convex shapes such as cbtBoxShape, cbtConvexHullShape etc.
/// It describes general convex shapes using the localGetSupportingVertex interface, used by collision detectors such as cbtGjkPairDetector.
ATTRIBUTE_ALIGNED16(class)
cbtConvexShape : public cbtCollisionShape
{
public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	cbtConvexShape();

	virtual ~cbtConvexShape();

	virtual cbtVector3 localGetSupportingVertex(const cbtVector3& vec) const = 0;

////////
#ifndef __SPU__
	virtual cbtVector3 localGetSupportingVertexWithoutMargin(const cbtVector3& vec) const = 0;
#endif  //#ifndef __SPU__

	cbtVector3 localGetSupportVertexWithoutMarginNonVirtual(const cbtVector3& vec) const;
	cbtVector3 localGetSupportVertexNonVirtual(const cbtVector3& vec) const;
	cbtScalar getMarginNonVirtual() const;
	void getAabbNonVirtual(const cbtTransform& t, cbtVector3& aabbMin, cbtVector3& aabbMax) const;

	virtual void project(const cbtTransform& trans, const cbtVector3& dir, cbtScalar& minProj, cbtScalar& maxProj, cbtVector3& witnesPtMin, cbtVector3& witnesPtMax) const;

	//notice that the vectors should be unit length
	virtual void batchedUnitVectorGetSupportingVertexWithoutMargin(const cbtVector3* vectors, cbtVector3* supportVerticesOut, int numVectors) const = 0;

	///getAabb's default implementation is brute force, expected derived classes to implement a fast dedicated version
	void getAabb(const cbtTransform& t, cbtVector3& aabbMin, cbtVector3& aabbMax) const = 0;

	virtual void getAabbSlow(const cbtTransform& t, cbtVector3& aabbMin, cbtVector3& aabbMax) const = 0;

	virtual void setLocalScaling(const cbtVector3& scaling) = 0;
	virtual const cbtVector3& getLocalScaling() const = 0;

	virtual void setMargin(cbtScalar margin) = 0;

	virtual cbtScalar getMargin() const = 0;

	virtual int getNumPreferredPenetrationDirections() const = 0;

	virtual void getPreferredPenetrationDirection(int index, cbtVector3& penetrationVector) const = 0;
};

#endif  //BT_CONVEX_SHAPE_INTERFACE1
