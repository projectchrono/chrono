#ifndef BT_COMPOUND_FROM_GIMPACT
#define BT_COMPOUND_FROM_GIMPACT

#include "BulletCollision/CollisionShapes/cbtCompoundShape.h"
#include "cbtGImpactShape.h"
#include "BulletCollision/NarrowPhaseCollision/cbtRaycastCallback.h"

ATTRIBUTE_ALIGNED16(class)
cbtCompoundFromGimpactShape : public cbtCompoundShape
{
public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	virtual ~cbtCompoundFromGimpactShape()
	{
		/*delete all the cbtBU_Simplex1to4 ChildShapes*/
		for (int i = 0; i < m_children.size(); i++)
		{
			delete m_children[i].m_childShape;
		}
	}
};

struct MyCallback : public cbtTriangleRaycastCallback
{
	int m_ignorePart;
	int m_ignoreTriangleIndex;

	MyCallback(const cbtVector3& from, const cbtVector3& to, int ignorePart, int ignoreTriangleIndex)
		: cbtTriangleRaycastCallback(from, to),
		  m_ignorePart(ignorePart),
		  m_ignoreTriangleIndex(ignoreTriangleIndex)
	{
	}
	virtual cbtScalar reportHit(const cbtVector3& hitNormalLocal, cbtScalar hitFraction, int partId, int triangleIndex)
	{
		if (partId != m_ignorePart || triangleIndex != m_ignoreTriangleIndex)
		{
			if (hitFraction < m_hitFraction)
				return hitFraction;
		}

		return m_hitFraction;
	}
};
struct MyInternalTriangleIndexCallback : public cbtInternalTriangleIndexCallback
{
	const cbtGImpactMeshShape* m_gimpactShape;
	cbtCompoundShape* m_colShape;
	cbtScalar m_depth;

	MyInternalTriangleIndexCallback(cbtCompoundShape* colShape, const cbtGImpactMeshShape* meshShape, cbtScalar depth)
		: m_colShape(colShape),
		  m_gimpactShape(meshShape),
		  m_depth(depth)
	{
	}

	virtual void internalProcessTriangleIndex(cbtVector3* triangle, int partId, int triangleIndex)
	{
		cbtVector3 scale = m_gimpactShape->getLocalScaling();
		cbtVector3 v0 = triangle[0] * scale;
		cbtVector3 v1 = triangle[1] * scale;
		cbtVector3 v2 = triangle[2] * scale;

		cbtVector3 centroid = (v0 + v1 + v2) / 3;
		cbtVector3 normal = (v1 - v0).cross(v2 - v0);
		normal.normalize();
		cbtVector3 rayFrom = centroid;
		cbtVector3 rayTo = centroid - normal * m_depth;

		MyCallback cb(rayFrom, rayTo, partId, triangleIndex);

		m_gimpactShape->processAllTrianglesRay(&cb, rayFrom, rayTo);
		if (cb.m_hitFraction < 1)
		{
			rayTo.setInterpolate3(cb.m_from, cb.m_to, cb.m_hitFraction);
			//rayTo = cb.m_from;
			//rayTo = rayTo.lerp(cb.m_to,cb.m_hitFraction);
			//gDebugDraw.drawLine(tr(centroid),tr(centroid+normal),cbtVector3(1,0,0));
		}

		cbtBU_Simplex1to4* tet = new cbtBU_Simplex1to4(v0, v1, v2, rayTo);
		cbtTransform ident;
		ident.setIdentity();
		m_colShape->addChildShape(ident, tet);
	}
};

cbtCompoundShape* cbtCreateCompoundFromGimpactShape(const cbtGImpactMeshShape* gimpactMesh, cbtScalar depth)
{
	cbtCompoundShape* colShape = new cbtCompoundFromGimpactShape();

	cbtTransform tr;
	tr.setIdentity();

	MyInternalTriangleIndexCallback cb(colShape, gimpactMesh, depth);
	cbtVector3 aabbMin, aabbMax;
	gimpactMesh->getAabb(tr, aabbMin, aabbMax);
	gimpactMesh->getMeshInterface()->InternalProcessAllTriangles(&cb, aabbMin, aabbMax);

	return colShape;
}

#endif  //BT_COMPOUND_FROM_GIMPACT
