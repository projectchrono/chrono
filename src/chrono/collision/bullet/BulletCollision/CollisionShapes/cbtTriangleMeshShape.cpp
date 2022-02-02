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

#include "cbtTriangleMeshShape.h"
#include "LinearMath/cbtVector3.h"
#include "LinearMath/cbtQuaternion.h"
#include "cbtStridingMeshInterface.h"
#include "LinearMath/cbtAabbUtil2.h"
#include "BulletCollision/CollisionShapes/cbtCollisionMargin.h"

cbtTriangleMeshShape::cbtTriangleMeshShape(cbtStridingMeshInterface* meshInterface)
	: cbtConcaveShape(), m_meshInterface(meshInterface)
{
	m_shapeType = TRIANGLE_MESH_SHAPE_PROXYTYPE;
	if (meshInterface->hasPremadeAabb())
	{
		meshInterface->getPremadeAabb(&m_localAabbMin, &m_localAabbMax);
	}
	else
	{
		recalcLocalAabb();
	}
}

cbtTriangleMeshShape::~cbtTriangleMeshShape()
{
}

void cbtTriangleMeshShape::getAabb(const cbtTransform& trans, cbtVector3& aabbMin, cbtVector3& aabbMax) const
{
	cbtVector3 localHalfExtents = cbtScalar(0.5) * (m_localAabbMax - m_localAabbMin);
	localHalfExtents += cbtVector3(getMargin(), getMargin(), getMargin());
	cbtVector3 localCenter = cbtScalar(0.5) * (m_localAabbMax + m_localAabbMin);

	cbtMatrix3x3 abs_b = trans.getBasis().absolute();

	cbtVector3 center = trans(localCenter);

	cbtVector3 extent = localHalfExtents.dot3(abs_b[0], abs_b[1], abs_b[2]);
	aabbMin = center - extent;
	aabbMax = center + extent;
}

void cbtTriangleMeshShape::recalcLocalAabb()
{
	for (int i = 0; i < 3; i++)
	{
		cbtVector3 vec(cbtScalar(0.), cbtScalar(0.), cbtScalar(0.));
		vec[i] = cbtScalar(1.);
		cbtVector3 tmp = localGetSupportingVertex(vec);
		m_localAabbMax[i] = tmp[i] + m_collisionMargin;
		vec[i] = cbtScalar(-1.);
		tmp = localGetSupportingVertex(vec);
		m_localAabbMin[i] = tmp[i] - m_collisionMargin;
	}
}

class SupportVertexCallback : public cbtTriangleCallback
{
	cbtVector3 m_supportVertexLocal;

public:
	cbtTransform m_worldTrans;
	cbtScalar m_maxDot;
	cbtVector3 m_supportVecLocal;

	SupportVertexCallback(const cbtVector3& supportVecWorld, const cbtTransform& trans)
		: m_supportVertexLocal(cbtScalar(0.), cbtScalar(0.), cbtScalar(0.)), m_worldTrans(trans), m_maxDot(cbtScalar(-BT_LARGE_FLOAT))

	{
		m_supportVecLocal = supportVecWorld * m_worldTrans.getBasis();
	}

	virtual void processTriangle(cbtVector3* triangle, int partId, int triangleIndex)
	{
		(void)partId;
		(void)triangleIndex;
		for (int i = 0; i < 3; i++)
		{
			cbtScalar dot = m_supportVecLocal.dot(triangle[i]);
			if (dot > m_maxDot)
			{
				m_maxDot = dot;
				m_supportVertexLocal = triangle[i];
			}
		}
	}

	cbtVector3 GetSupportVertexWorldSpace()
	{
		return m_worldTrans(m_supportVertexLocal);
	}

	cbtVector3 GetSupportVertexLocal()
	{
		return m_supportVertexLocal;
	}
};

void cbtTriangleMeshShape::setLocalScaling(const cbtVector3& scaling)
{
	m_meshInterface->setScaling(scaling);
	recalcLocalAabb();
}

const cbtVector3& cbtTriangleMeshShape::getLocalScaling() const
{
	return m_meshInterface->getScaling();
}

//#define DEBUG_TRIANGLE_MESH

void cbtTriangleMeshShape::processAllTriangles(cbtTriangleCallback* callback, const cbtVector3& aabbMin, const cbtVector3& aabbMax) const
{
	struct FilteredCallback : public cbtInternalTriangleIndexCallback
	{
		cbtTriangleCallback* m_callback;
		cbtVector3 m_aabbMin;
		cbtVector3 m_aabbMax;

		FilteredCallback(cbtTriangleCallback* callback, const cbtVector3& aabbMin, const cbtVector3& aabbMax)
			: m_callback(callback),
			  m_aabbMin(aabbMin),
			  m_aabbMax(aabbMax)
		{
		}

		virtual void internalProcessTriangleIndex(cbtVector3* triangle, int partId, int triangleIndex)
		{
			if (TestTriangleAgainstAabb2(&triangle[0], m_aabbMin, m_aabbMax))
			{
				//check aabb in triangle-space, before doing this
				m_callback->processTriangle(triangle, partId, triangleIndex);
			}
		}
	};

	FilteredCallback filterCallback(callback, aabbMin, aabbMax);

	m_meshInterface->InternalProcessAllTriangles(&filterCallback, aabbMin, aabbMax);
}

void cbtTriangleMeshShape::calculateLocalInertia(cbtScalar mass, cbtVector3& inertia) const
{
	(void)mass;
	//moving concave objects not supported
	cbtAssert(0);
	inertia.setValue(cbtScalar(0.), cbtScalar(0.), cbtScalar(0.));
}

cbtVector3 cbtTriangleMeshShape::localGetSupportingVertex(const cbtVector3& vec) const
{
	cbtVector3 supportVertex;

	cbtTransform ident;
	ident.setIdentity();

	SupportVertexCallback supportCallback(vec, ident);

	cbtVector3 aabbMax(cbtScalar(BT_LARGE_FLOAT), cbtScalar(BT_LARGE_FLOAT), cbtScalar(BT_LARGE_FLOAT));

	processAllTriangles(&supportCallback, -aabbMax, aabbMax);

	supportVertex = supportCallback.GetSupportVertexLocal();

	return supportVertex;
}
