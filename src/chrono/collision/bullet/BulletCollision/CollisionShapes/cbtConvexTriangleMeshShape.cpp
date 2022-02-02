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

#include "cbtConvexTriangleMeshShape.h"
#include "BulletCollision/CollisionShapes/cbtCollisionMargin.h"

#include "LinearMath/cbtQuaternion.h"
#include "BulletCollision/CollisionShapes/cbtStridingMeshInterface.h"

cbtConvexTriangleMeshShape ::cbtConvexTriangleMeshShape(cbtStridingMeshInterface* meshInterface, bool calcAabb)
	: cbtPolyhedralConvexAabbCachingShape(), m_stridingMesh(meshInterface)
{
	m_shapeType = CONVEX_TRIANGLEMESH_SHAPE_PROXYTYPE;
	if (calcAabb)
		recalcLocalAabb();
}

///It's not nice to have all this virtual function overhead, so perhaps we can also gather the points once
///but then we are duplicating
class LocalSupportVertexCallback : public cbtInternalTriangleIndexCallback
{
	cbtVector3 m_supportVertexLocal;

public:
	cbtScalar m_maxDot;
	cbtVector3 m_supportVecLocal;

	LocalSupportVertexCallback(const cbtVector3& supportVecLocal)
		: m_supportVertexLocal(cbtScalar(0.), cbtScalar(0.), cbtScalar(0.)),
		  m_maxDot(cbtScalar(-BT_LARGE_FLOAT)),
		  m_supportVecLocal(supportVecLocal)
	{
	}

	virtual void internalProcessTriangleIndex(cbtVector3* triangle, int partId, int triangleIndex)
	{
		(void)triangleIndex;
		(void)partId;

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

	cbtVector3 GetSupportVertexLocal()
	{
		return m_supportVertexLocal;
	}
};

cbtVector3 cbtConvexTriangleMeshShape::localGetSupportingVertexWithoutMargin(const cbtVector3& vec0) const
{
	cbtVector3 supVec(cbtScalar(0.), cbtScalar(0.), cbtScalar(0.));

	cbtVector3 vec = vec0;
	cbtScalar lenSqr = vec.length2();
	if (lenSqr < cbtScalar(0.0001))
	{
		vec.setValue(1, 0, 0);
	}
	else
	{
		cbtScalar rlen = cbtScalar(1.) / cbtSqrt(lenSqr);
		vec *= rlen;
	}

	LocalSupportVertexCallback supportCallback(vec);
	cbtVector3 aabbMax(cbtScalar(BT_LARGE_FLOAT), cbtScalar(BT_LARGE_FLOAT), cbtScalar(BT_LARGE_FLOAT));
	m_stridingMesh->InternalProcessAllTriangles(&supportCallback, -aabbMax, aabbMax);
	supVec = supportCallback.GetSupportVertexLocal();

	return supVec;
}

void cbtConvexTriangleMeshShape::batchedUnitVectorGetSupportingVertexWithoutMargin(const cbtVector3* vectors, cbtVector3* supportVerticesOut, int numVectors) const
{
	//use 'w' component of supportVerticesOut?
	{
		for (int i = 0; i < numVectors; i++)
		{
			supportVerticesOut[i][3] = cbtScalar(-BT_LARGE_FLOAT);
		}
	}

	///@todo: could do the batch inside the callback!

	for (int j = 0; j < numVectors; j++)
	{
		const cbtVector3& vec = vectors[j];
		LocalSupportVertexCallback supportCallback(vec);
		cbtVector3 aabbMax(cbtScalar(BT_LARGE_FLOAT), cbtScalar(BT_LARGE_FLOAT), cbtScalar(BT_LARGE_FLOAT));
		m_stridingMesh->InternalProcessAllTriangles(&supportCallback, -aabbMax, aabbMax);
		supportVerticesOut[j] = supportCallback.GetSupportVertexLocal();
	}
}

cbtVector3 cbtConvexTriangleMeshShape::localGetSupportingVertex(const cbtVector3& vec) const
{
	cbtVector3 supVertex = localGetSupportingVertexWithoutMargin(vec);

	if (getMargin() != cbtScalar(0.))
	{
		cbtVector3 vecnorm = vec;
		if (vecnorm.length2() < (SIMD_EPSILON * SIMD_EPSILON))
		{
			vecnorm.setValue(cbtScalar(-1.), cbtScalar(-1.), cbtScalar(-1.));
		}
		vecnorm.normalize();
		supVertex += getMargin() * vecnorm;
	}
	return supVertex;
}

//currently just for debugging (drawing), perhaps future support for algebraic continuous collision detection
//Please note that you can debug-draw cbtConvexTriangleMeshShape with the Raytracer Demo
int cbtConvexTriangleMeshShape::getNumVertices() const
{
	//cache this?
	return 0;
}

int cbtConvexTriangleMeshShape::getNumEdges() const
{
	return 0;
}

void cbtConvexTriangleMeshShape::getEdge(int, cbtVector3&, cbtVector3&) const
{
	cbtAssert(0);
}

void cbtConvexTriangleMeshShape::getVertex(int, cbtVector3&) const
{
	cbtAssert(0);
}

int cbtConvexTriangleMeshShape::getNumPlanes() const
{
	return 0;
}

void cbtConvexTriangleMeshShape::getPlane(cbtVector3&, cbtVector3&, int) const
{
	cbtAssert(0);
}

//not yet
bool cbtConvexTriangleMeshShape::isInside(const cbtVector3&, cbtScalar) const
{
	cbtAssert(0);
	return false;
}

void cbtConvexTriangleMeshShape::setLocalScaling(const cbtVector3& scaling)
{
	m_stridingMesh->setScaling(scaling);

	recalcLocalAabb();
}

const cbtVector3& cbtConvexTriangleMeshShape::getLocalScaling() const
{
	return m_stridingMesh->getScaling();
}

void cbtConvexTriangleMeshShape::calculatePrincipalAxisTransform(cbtTransform& principal, cbtVector3& inertia, cbtScalar& volume) const
{
	class CenterCallback : public cbtInternalTriangleIndexCallback
	{
		bool first;
		cbtVector3 ref;
		cbtVector3 sum;
		cbtScalar volume;

	public:
		CenterCallback() : first(true), ref(0, 0, 0), sum(0, 0, 0), volume(0)
		{
		}

		virtual void internalProcessTriangleIndex(cbtVector3* triangle, int partId, int triangleIndex)
		{
			(void)triangleIndex;
			(void)partId;
			if (first)
			{
				ref = triangle[0];
				first = false;
			}
			else
			{
				cbtScalar vol = cbtFabs((triangle[0] - ref).triple(triangle[1] - ref, triangle[2] - ref));
				sum += (cbtScalar(0.25) * vol) * ((triangle[0] + triangle[1] + triangle[2] + ref));
				volume += vol;
			}
		}

		cbtVector3 getCenter()
		{
			return (volume > 0) ? sum / volume : ref;
		}

		cbtScalar getVolume()
		{
			return volume * cbtScalar(1. / 6);
		}
	};

	class InertiaCallback : public cbtInternalTriangleIndexCallback
	{
		cbtMatrix3x3 sum;
		cbtVector3 center;

	public:
		InertiaCallback(cbtVector3& center) : sum(0, 0, 0, 0, 0, 0, 0, 0, 0), center(center)
		{
		}

		virtual void internalProcessTriangleIndex(cbtVector3* triangle, int partId, int triangleIndex)
		{
			(void)triangleIndex;
			(void)partId;
			cbtMatrix3x3 i;
			cbtVector3 a = triangle[0] - center;
			cbtVector3 b = triangle[1] - center;
			cbtVector3 c = triangle[2] - center;
			cbtScalar volNeg = -cbtFabs(a.triple(b, c)) * cbtScalar(1. / 6);
			for (int j = 0; j < 3; j++)
			{
				for (int k = 0; k <= j; k++)
				{
					i[j][k] = i[k][j] = volNeg * (cbtScalar(0.1) * (a[j] * a[k] + b[j] * b[k] + c[j] * c[k]) + cbtScalar(0.05) * (a[j] * b[k] + a[k] * b[j] + a[j] * c[k] + a[k] * c[j] + b[j] * c[k] + b[k] * c[j]));
				}
			}
			cbtScalar i00 = -i[0][0];
			cbtScalar i11 = -i[1][1];
			cbtScalar i22 = -i[2][2];
			i[0][0] = i11 + i22;
			i[1][1] = i22 + i00;
			i[2][2] = i00 + i11;
			sum[0] += i[0];
			sum[1] += i[1];
			sum[2] += i[2];
		}

		cbtMatrix3x3& getInertia()
		{
			return sum;
		}
	};

	CenterCallback centerCallback;
	cbtVector3 aabbMax(cbtScalar(BT_LARGE_FLOAT), cbtScalar(BT_LARGE_FLOAT), cbtScalar(BT_LARGE_FLOAT));
	m_stridingMesh->InternalProcessAllTriangles(&centerCallback, -aabbMax, aabbMax);
	cbtVector3 center = centerCallback.getCenter();
	principal.setOrigin(center);
	volume = centerCallback.getVolume();

	InertiaCallback inertiaCallback(center);
	m_stridingMesh->InternalProcessAllTriangles(&inertiaCallback, -aabbMax, aabbMax);

	cbtMatrix3x3& i = inertiaCallback.getInertia();
	i.diagonalize(principal.getBasis(), cbtScalar(0.00001), 20);
	inertia.setValue(i[0][0], i[1][1], i[2][2]);
	inertia /= volume;
}
