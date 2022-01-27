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

#ifndef BT_TRIANGLE_MESH_SHAPE_H
#define BT_TRIANGLE_MESH_SHAPE_H

#include "cbtConcaveShape.h"
#include "cbtStridingMeshInterface.h"

///The cbtTriangleMeshShape is an internal concave triangle mesh interface. Don't use this class directly, use cbtBvhTriangleMeshShape instead.
ATTRIBUTE_ALIGNED16(class)
cbtTriangleMeshShape : public cbtConcaveShape
{
protected:
	cbtVector3 m_localAabbMin;
	cbtVector3 m_localAabbMax;
	cbtStridingMeshInterface* m_meshInterface;

	///cbtTriangleMeshShape constructor has been disabled/protected, so that users will not mistakenly use this class.
	///Don't use cbtTriangleMeshShape but use cbtBvhTriangleMeshShape instead!
	cbtTriangleMeshShape(cbtStridingMeshInterface * meshInterface);

public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	virtual ~cbtTriangleMeshShape();

	virtual cbtVector3 localGetSupportingVertex(const cbtVector3& vec) const;

	virtual cbtVector3 localGetSupportingVertexWithoutMargin(const cbtVector3& vec) const
	{
		cbtAssert(0);
		return localGetSupportingVertex(vec);
	}

	void recalcLocalAabb();

	virtual void getAabb(const cbtTransform& t, cbtVector3& aabbMin, cbtVector3& aabbMax) const;

	virtual void processAllTriangles(cbtTriangleCallback * callback, const cbtVector3& aabbMin, const cbtVector3& aabbMax) const;

	virtual void calculateLocalInertia(cbtScalar mass, cbtVector3 & inertia) const;

	virtual void setLocalScaling(const cbtVector3& scaling);
	virtual const cbtVector3& getLocalScaling() const;

	cbtStridingMeshInterface* getMeshInterface()
	{
		return m_meshInterface;
	}

	const cbtStridingMeshInterface* getMeshInterface() const
	{
		return m_meshInterface;
	}

	const cbtVector3& getLocalAabbMin() const
	{
		return m_localAabbMin;
	}
	const cbtVector3& getLocalAabbMax() const
	{
		return m_localAabbMax;
	}

	//debugging
	virtual const char* getName() const { return "TRIANGLEMESH"; }
};

#endif  //BT_TRIANGLE_MESH_SHAPE_H
