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
#ifndef BT_CONVEX_TRIANGLEMESH_SHAPE_H
#define BT_CONVEX_TRIANGLEMESH_SHAPE_H

#include "cbtPolyhedralConvexShape.h"
#include "BulletCollision/BroadphaseCollision/cbtBroadphaseProxy.h"  // for the types

/// The cbtConvexTriangleMeshShape is a convex hull of a triangle mesh, but the performance is not as good as cbtConvexHullShape.
/// A small benefit of this class is that it uses the cbtStridingMeshInterface, so you can avoid the duplication of the triangle mesh data. Nevertheless, most users should use the much better performing cbtConvexHullShape instead.
ATTRIBUTE_ALIGNED16(class)
cbtConvexTriangleMeshShape : public cbtPolyhedralConvexAabbCachingShape
{
	class cbtStridingMeshInterface* m_stridingMesh;

public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	cbtConvexTriangleMeshShape(cbtStridingMeshInterface * meshInterface, bool calcAabb = true);

	class cbtStridingMeshInterface* getMeshInterface()
	{
		return m_stridingMesh;
	}
	const class cbtStridingMeshInterface* getMeshInterface() const
	{
		return m_stridingMesh;
	}

	virtual cbtVector3 localGetSupportingVertex(const cbtVector3& vec) const;
	virtual cbtVector3 localGetSupportingVertexWithoutMargin(const cbtVector3& vec) const;
	virtual void batchedUnitVectorGetSupportingVertexWithoutMargin(const cbtVector3* vectors, cbtVector3* supportVerticesOut, int numVectors) const;

	//debugging
	virtual const char* getName() const { return "ConvexTrimesh"; }

	virtual int getNumVertices() const;
	virtual int getNumEdges() const;
	virtual void getEdge(int i, cbtVector3& pa, cbtVector3& pb) const;
	virtual void getVertex(int i, cbtVector3& vtx) const;
	virtual int getNumPlanes() const;
	virtual void getPlane(cbtVector3 & planeNormal, cbtVector3 & planeSupport, int i) const;
	virtual bool isInside(const cbtVector3& pt, cbtScalar tolerance) const;

	virtual void setLocalScaling(const cbtVector3& scaling);
	virtual const cbtVector3& getLocalScaling() const;

	///computes the exact moment of inertia and the transform from the coordinate system defined by the principal axes of the moment of inertia
	///and the center of mass to the current coordinate system. A mass of 1 is assumed, for other masses just multiply the computed "inertia"
	///by the mass. The resulting transform "principal" has to be applied inversely to the mesh in order for the local coordinate system of the
	///shape to be centered at the center of mass and to coincide with the principal axes. This also necessitates a correction of the world transform
	///of the collision object by the principal transform. This method also computes the volume of the convex mesh.
	void calculatePrincipalAxisTransform(cbtTransform & principal, cbtVector3 & inertia, cbtScalar & volume) const;
};

#endif  //BT_CONVEX_TRIANGLEMESH_SHAPE_H
