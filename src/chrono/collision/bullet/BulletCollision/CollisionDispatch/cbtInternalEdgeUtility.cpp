#include "cbtInternalEdgeUtility.h"

#include "BulletCollision/CollisionShapes/cbtBvhTriangleMeshShape.h"
#include "BulletCollision/CollisionShapes/cbtScaledBvhTriangleMeshShape.h"
#include "BulletCollision/CollisionShapes/cbtTriangleShape.h"
#include "BulletCollision/CollisionDispatch/cbtCollisionObject.h"
#include "BulletCollision/NarrowPhaseCollision/cbtManifoldPoint.h"
#include "LinearMath/cbtIDebugDraw.h"
#include "BulletCollision/CollisionDispatch/cbtCollisionObjectWrapper.h"

//#define DEBUG_INTERNAL_EDGE

#ifdef DEBUG_INTERNAL_EDGE
#include <stdio.h>
#endif  //DEBUG_INTERNAL_EDGE

#ifdef BT_INTERNAL_EDGE_DEBUG_DRAW
static cbtIDebugDraw* gDebugDrawer = 0;

void cbtSetDebugDrawer(cbtIDebugDraw* debugDrawer)
{
	gDebugDrawer = debugDrawer;
}

static void cbtDebugDrawLine(const cbtVector3& from, const cbtVector3& to, const cbtVector3& color)
{
	if (gDebugDrawer)
		gDebugDrawer->drawLine(from, to, color);
}
#endif  //BT_INTERNAL_EDGE_DEBUG_DRAW

static int cbtGetHash(int partId, int triangleIndex)
{
	int hash = (partId << (31 - MAX_NUM_PARTS_IN_BITS)) | triangleIndex;
	return hash;
}

static cbtScalar cbtGetAngle(const cbtVector3& edgeA, const cbtVector3& normalA, const cbtVector3& normalB)
{
	const cbtVector3 refAxis0 = edgeA;
	const cbtVector3 refAxis1 = normalA;
	const cbtVector3 swingAxis = normalB;
	cbtScalar angle = cbtAtan2(swingAxis.dot(refAxis0), swingAxis.dot(refAxis1));
	return angle;
}

struct cbtConnectivityProcessor : public cbtTriangleCallback
{
	int m_partIdA;
	int m_triangleIndexA;
	cbtVector3* m_triangleVerticesA;
	cbtTriangleInfoMap* m_triangleInfoMap;

	virtual void processTriangle(cbtVector3* triangle, int partId, int triangleIndex)
	{
		//skip self-collisions
		if ((m_partIdA == partId) && (m_triangleIndexA == triangleIndex))
			return;

		//skip duplicates (disabled for now)
		//if ((m_partIdA <= partId) && (m_triangleIndexA <= triangleIndex))
		//	return;

		//search for shared vertices and edges
		int numshared = 0;
		int sharedVertsA[3] = {-1, -1, -1};
		int sharedVertsB[3] = {-1, -1, -1};

		///skip degenerate triangles
		cbtScalar crossBSqr = ((triangle[1] - triangle[0]).cross(triangle[2] - triangle[0])).length2();
		if (crossBSqr < m_triangleInfoMap->m_equalVertexThreshold)
			return;

		cbtScalar crossASqr = ((m_triangleVerticesA[1] - m_triangleVerticesA[0]).cross(m_triangleVerticesA[2] - m_triangleVerticesA[0])).length2();
		///skip degenerate triangles
		if (crossASqr < m_triangleInfoMap->m_equalVertexThreshold)
			return;

#if 0
		printf("triangle A[0]	=	(%f,%f,%f)\ntriangle A[1]	=	(%f,%f,%f)\ntriangle A[2]	=	(%f,%f,%f)\n",
			m_triangleVerticesA[0].getX(),m_triangleVerticesA[0].getY(),m_triangleVerticesA[0].getZ(),
			m_triangleVerticesA[1].getX(),m_triangleVerticesA[1].getY(),m_triangleVerticesA[1].getZ(),
			m_triangleVerticesA[2].getX(),m_triangleVerticesA[2].getY(),m_triangleVerticesA[2].getZ());

		printf("partId=%d, triangleIndex=%d\n",partId,triangleIndex);
		printf("triangle B[0]	=	(%f,%f,%f)\ntriangle B[1]	=	(%f,%f,%f)\ntriangle B[2]	=	(%f,%f,%f)\n",
			triangle[0].getX(),triangle[0].getY(),triangle[0].getZ(),
			triangle[1].getX(),triangle[1].getY(),triangle[1].getZ(),
			triangle[2].getX(),triangle[2].getY(),triangle[2].getZ());
#endif

		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				if ((m_triangleVerticesA[i] - triangle[j]).length2() < m_triangleInfoMap->m_equalVertexThreshold)
				{
					sharedVertsA[numshared] = i;
					sharedVertsB[numshared] = j;
					numshared++;
					///degenerate case
					if (numshared >= 3)
						return;
				}
			}
			///degenerate case
			if (numshared >= 3)
				return;
		}
		switch (numshared)
		{
			case 0:
			{
				break;
			}
			case 1:
			{
				//shared vertex
				break;
			}
			case 2:
			{
				//shared edge
				//we need to make sure the edge is in the order V2V0 and not V0V2 so that the signs are correct
				if (sharedVertsA[0] == 0 && sharedVertsA[1] == 2)
				{
					sharedVertsA[0] = 2;
					sharedVertsA[1] = 0;
					int tmp = sharedVertsB[1];
					sharedVertsB[1] = sharedVertsB[0];
					sharedVertsB[0] = tmp;
				}

				int hash = cbtGetHash(m_partIdA, m_triangleIndexA);

				cbtTriangleInfo* info = m_triangleInfoMap->find(hash);
				if (!info)
				{
					cbtTriangleInfo tmp;
					m_triangleInfoMap->insert(hash, tmp);
					info = m_triangleInfoMap->find(hash);
				}

				int sumvertsA = sharedVertsA[0] + sharedVertsA[1];
				int otherIndexA = 3 - sumvertsA;

				cbtVector3 edge(m_triangleVerticesA[sharedVertsA[1]] - m_triangleVerticesA[sharedVertsA[0]]);

				cbtTriangleShape tA(m_triangleVerticesA[0], m_triangleVerticesA[1], m_triangleVerticesA[2]);
				int otherIndexB = 3 - (sharedVertsB[0] + sharedVertsB[1]);

				cbtTriangleShape tB(triangle[sharedVertsB[1]], triangle[sharedVertsB[0]], triangle[otherIndexB]);
				//cbtTriangleShape tB(triangle[0],triangle[1],triangle[2]);

				cbtVector3 normalA;
				cbtVector3 normalB;
				tA.calcNormal(normalA);
				tB.calcNormal(normalB);
				edge.normalize();
				cbtVector3 edgeCrossA = edge.cross(normalA).normalize();

				{
					cbtVector3 tmp = m_triangleVerticesA[otherIndexA] - m_triangleVerticesA[sharedVertsA[0]];
					if (edgeCrossA.dot(tmp) < 0)
					{
						edgeCrossA *= -1;
					}
				}

				cbtVector3 edgeCrossB = edge.cross(normalB).normalize();

				{
					cbtVector3 tmp = triangle[otherIndexB] - triangle[sharedVertsB[0]];
					if (edgeCrossB.dot(tmp) < 0)
					{
						edgeCrossB *= -1;
					}
				}

				cbtScalar angle2 = 0;
				cbtScalar ang4 = 0.f;

				cbtVector3 calculatedEdge = edgeCrossA.cross(edgeCrossB);
				cbtScalar len2 = calculatedEdge.length2();

				cbtScalar correctedAngle(0);
				//cbtVector3 calculatedNormalB = normalA;
				bool isConvex = false;

				if (len2 < m_triangleInfoMap->m_planarEpsilon)
				{
					angle2 = 0.f;
					ang4 = 0.f;
				}
				else
				{
					calculatedEdge.normalize();
					cbtVector3 calculatedNormalA = calculatedEdge.cross(edgeCrossA);
					calculatedNormalA.normalize();
					angle2 = cbtGetAngle(calculatedNormalA, edgeCrossA, edgeCrossB);
					ang4 = SIMD_PI - angle2;
					cbtScalar dotA = normalA.dot(edgeCrossB);
					///@todo: check if we need some epsilon, due to floating point imprecision
					isConvex = (dotA < 0.);

					correctedAngle = isConvex ? ang4 : -ang4;
				}

				//alternatively use
				//cbtVector3 calculatedNormalB2 = quatRotate(orn,normalA);

				switch (sumvertsA)
				{
					case 1:
					{
						cbtVector3 edge = m_triangleVerticesA[0] - m_triangleVerticesA[1];
						cbtQuaternion orn(edge, -correctedAngle);
						cbtVector3 computedNormalB = quatRotate(orn, normalA);
						cbtScalar bla = computedNormalB.dot(normalB);
						if (bla < 0)
						{
							computedNormalB *= -1;
							info->m_flags |= TRI_INFO_V0V1_SWAP_NORMALB;
						}
#ifdef DEBUG_INTERNAL_EDGE
						if ((computedNormalB - normalB).length() > 0.0001)
						{
							printf("warning: normals not identical\n");
						}
#endif  //DEBUG_INTERNAL_EDGE

						info->m_edgeV0V1Angle = -correctedAngle;

						if (isConvex)
							info->m_flags |= TRI_INFO_V0V1_CONVEX;
						break;
					}
					case 2:
					{
						cbtVector3 edge = m_triangleVerticesA[2] - m_triangleVerticesA[0];
						cbtQuaternion orn(edge, -correctedAngle);
						cbtVector3 computedNormalB = quatRotate(orn, normalA);
						if (computedNormalB.dot(normalB) < 0)
						{
							computedNormalB *= -1;
							info->m_flags |= TRI_INFO_V2V0_SWAP_NORMALB;
						}

#ifdef DEBUG_INTERNAL_EDGE
						if ((computedNormalB - normalB).length() > 0.0001)
						{
							printf("warning: normals not identical\n");
						}
#endif  //DEBUG_INTERNAL_EDGE
						info->m_edgeV2V0Angle = -correctedAngle;
						if (isConvex)
							info->m_flags |= TRI_INFO_V2V0_CONVEX;
						break;
					}
					case 3:
					{
						cbtVector3 edge = m_triangleVerticesA[1] - m_triangleVerticesA[2];
						cbtQuaternion orn(edge, -correctedAngle);
						cbtVector3 computedNormalB = quatRotate(orn, normalA);
						if (computedNormalB.dot(normalB) < 0)
						{
							info->m_flags |= TRI_INFO_V1V2_SWAP_NORMALB;
							computedNormalB *= -1;
						}
#ifdef DEBUG_INTERNAL_EDGE
						if ((computedNormalB - normalB).length() > 0.0001)
						{
							printf("warning: normals not identical\n");
						}
#endif  //DEBUG_INTERNAL_EDGE
						info->m_edgeV1V2Angle = -correctedAngle;

						if (isConvex)
							info->m_flags |= TRI_INFO_V1V2_CONVEX;
						break;
					}
				}

				break;
			}
			default:
			{
				//				printf("warning: duplicate triangle\n");
			}
		}
	}
};
/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////

void cbtGenerateInternalEdgeInfo(cbtBvhTriangleMeshShape* trimeshShape, cbtTriangleInfoMap* triangleInfoMap)
{
	//the user pointer shouldn't already be used for other purposes, we intend to store connectivity info there!
	if (trimeshShape->getTriangleInfoMap())
		return;

	trimeshShape->setTriangleInfoMap(triangleInfoMap);

	cbtStridingMeshInterface* meshInterface = trimeshShape->getMeshInterface();
	const cbtVector3& meshScaling = meshInterface->getScaling();

	for (int partId = 0; partId < meshInterface->getNumSubParts(); partId++)
	{
		const unsigned char* vertexbase = 0;
		int numverts = 0;
		PHY_ScalarType type = PHY_INTEGER;
		int stride = 0;
		const unsigned char* indexbase = 0;
		int indexstride = 0;
		int numfaces = 0;
		PHY_ScalarType indicestype = PHY_INTEGER;
		//PHY_ScalarType indexType=0;

		cbtVector3 triangleVerts[3];
		meshInterface->getLockedReadOnlyVertexIndexBase(&vertexbase, numverts, type, stride, &indexbase, indexstride, numfaces, indicestype, partId);
		cbtVector3 aabbMin, aabbMax;

		for (int triangleIndex = 0; triangleIndex < numfaces; triangleIndex++)
		{
			unsigned int* gfxbase = (unsigned int*)(indexbase + triangleIndex * indexstride);

			for (int j = 2; j >= 0; j--)
			{
				int graphicsindex = indicestype == PHY_SHORT ? ((unsigned short*)gfxbase)[j] : gfxbase[j];
				if (type == PHY_FLOAT)
				{
					float* graphicsbase = (float*)(vertexbase + graphicsindex * stride);
					triangleVerts[j] = cbtVector3(
						graphicsbase[0] * meshScaling.getX(),
						graphicsbase[1] * meshScaling.getY(),
						graphicsbase[2] * meshScaling.getZ());
				}
				else
				{
					double* graphicsbase = (double*)(vertexbase + graphicsindex * stride);
					triangleVerts[j] = cbtVector3(cbtScalar(graphicsbase[0] * meshScaling.getX()), cbtScalar(graphicsbase[1] * meshScaling.getY()), cbtScalar(graphicsbase[2] * meshScaling.getZ()));
				}
			}
			aabbMin.setValue(cbtScalar(BT_LARGE_FLOAT), cbtScalar(BT_LARGE_FLOAT), cbtScalar(BT_LARGE_FLOAT));
			aabbMax.setValue(cbtScalar(-BT_LARGE_FLOAT), cbtScalar(-BT_LARGE_FLOAT), cbtScalar(-BT_LARGE_FLOAT));
			aabbMin.setMin(triangleVerts[0]);
			aabbMax.setMax(triangleVerts[0]);
			aabbMin.setMin(triangleVerts[1]);
			aabbMax.setMax(triangleVerts[1]);
			aabbMin.setMin(triangleVerts[2]);
			aabbMax.setMax(triangleVerts[2]);

			cbtConnectivityProcessor connectivityProcessor;
			connectivityProcessor.m_partIdA = partId;
			connectivityProcessor.m_triangleIndexA = triangleIndex;
			connectivityProcessor.m_triangleVerticesA = &triangleVerts[0];
			connectivityProcessor.m_triangleInfoMap = triangleInfoMap;

			trimeshShape->processAllTriangles(&connectivityProcessor, aabbMin, aabbMax);
		}
	}
}

// Given a point and a line segment (defined by two points), compute the closest point
// in the line.  Cap the point at the endpoints of the line segment.
void cbtNearestPointInLineSegment(const cbtVector3& point, const cbtVector3& line0, const cbtVector3& line1, cbtVector3& nearestPoint)
{
	cbtVector3 lineDelta = line1 - line0;

	// Handle degenerate lines
	if (lineDelta.fuzzyZero())
	{
		nearestPoint = line0;
	}
	else
	{
		cbtScalar delta = (point - line0).dot(lineDelta) / (lineDelta).dot(lineDelta);

		// Clamp the point to conform to the segment's endpoints
		if (delta < 0)
			delta = 0;
		else if (delta > 1)
			delta = 1;

		nearestPoint = line0 + lineDelta * delta;
	}
}

bool cbtClampNormal(const cbtVector3& edge, const cbtVector3& tri_normal_org, const cbtVector3& localContactNormalOnB, cbtScalar correctedEdgeAngle, cbtVector3& clampedLocalNormal)
{
	cbtVector3 tri_normal = tri_normal_org;
	//we only have a local triangle normal, not a local contact normal -> only normal in world space...
	//either compute the current angle all in local space, or all in world space

	cbtVector3 edgeCross = edge.cross(tri_normal).normalize();
	cbtScalar curAngle = cbtGetAngle(edgeCross, tri_normal, localContactNormalOnB);

	if (correctedEdgeAngle < 0)
	{
		if (curAngle < correctedEdgeAngle)
		{
			cbtScalar diffAngle = correctedEdgeAngle - curAngle;
			cbtQuaternion rotation(edge, diffAngle);
			clampedLocalNormal = cbtMatrix3x3(rotation) * localContactNormalOnB;
			return true;
		}
	}

	if (correctedEdgeAngle >= 0)
	{
		if (curAngle > correctedEdgeAngle)
		{
			cbtScalar diffAngle = correctedEdgeAngle - curAngle;
			cbtQuaternion rotation(edge, diffAngle);
			clampedLocalNormal = cbtMatrix3x3(rotation) * localContactNormalOnB;
			return true;
		}
	}
	return false;
}

/// Changes a cbtManifoldPoint collision normal to the normal from the mesh.
void cbtAdjustInternalEdgeContacts(cbtManifoldPoint& cp, const cbtCollisionObjectWrapper* colObj0Wrap, const cbtCollisionObjectWrapper* colObj1Wrap, int partId0, int index0, int normalAdjustFlags)
{
	//cbtAssert(colObj0->getCollisionShape()->getShapeType() == TRIANGLE_SHAPE_PROXYTYPE);
	if (colObj0Wrap->getCollisionShape()->getShapeType() != TRIANGLE_SHAPE_PROXYTYPE)
		return;

	cbtBvhTriangleMeshShape* trimesh = 0;

	if (colObj0Wrap->getCollisionObject()->getCollisionShape()->getShapeType() == SCALED_TRIANGLE_MESH_SHAPE_PROXYTYPE)
	{
		trimesh = ((cbtScaledBvhTriangleMeshShape*)colObj0Wrap->getCollisionObject()->getCollisionShape())->getChildShape();
	}
	else
	{
		if (colObj0Wrap->getCollisionObject()->getCollisionShape()->getShapeType() == TRIANGLE_MESH_SHAPE_PROXYTYPE)
		{
			trimesh = (cbtBvhTriangleMeshShape*)colObj0Wrap->getCollisionObject()->getCollisionShape();
		}
	}
	if (trimesh == 0)
		return;

	cbtTriangleInfoMap* triangleInfoMapPtr = (cbtTriangleInfoMap*)trimesh->getTriangleInfoMap();
	if (!triangleInfoMapPtr)
		return;

	int hash = cbtGetHash(partId0, index0);

	cbtTriangleInfo* info = triangleInfoMapPtr->find(hash);
	if (!info)
		return;

	cbtScalar frontFacing = (normalAdjustFlags & BT_TRIANGLE_CONVEX_BACKFACE_MODE) == 0 ? 1.f : -1.f;

	const cbtTriangleShape* tri_shape = static_cast<const cbtTriangleShape*>(colObj0Wrap->getCollisionShape());
	cbtVector3 v0, v1, v2;
	tri_shape->getVertex(0, v0);
	tri_shape->getVertex(1, v1);
	tri_shape->getVertex(2, v2);

	//cbtVector3 center = (v0+v1+v2)*cbtScalar(1./3.);

	cbtVector3 red(1, 0, 0), green(0, 1, 0), blue(0, 0, 1), white(1, 1, 1), black(0, 0, 0);
	cbtVector3 tri_normal;
	tri_shape->calcNormal(tri_normal);

	//cbtScalar dot = tri_normal.dot(cp.m_normalWorldOnB);
	cbtVector3 nearest;
	cbtNearestPointInLineSegment(cp.m_localPointB, v0, v1, nearest);

	cbtVector3 contact = cp.m_localPointB;
#ifdef BT_INTERNAL_EDGE_DEBUG_DRAW
	const cbtTransform& tr = colObj0->getWorldTransform();
	cbtDebugDrawLine(tr * nearest, tr * cp.m_localPointB, red);
#endif  //BT_INTERNAL_EDGE_DEBUG_DRAW

	bool isNearEdge = false;

	int numConcaveEdgeHits = 0;
	int numConvexEdgeHits = 0;

	cbtVector3 localContactNormalOnB = colObj0Wrap->getWorldTransform().getBasis().transpose() * cp.m_normalWorldOnB;
	localContactNormalOnB.normalize();  //is this necessary?

	// Get closest edge
	int bestedge = -1;
	cbtScalar disttobestedge = BT_LARGE_FLOAT;
	//
	// Edge 0 -> 1
	if (cbtFabs(info->m_edgeV0V1Angle) < triangleInfoMapPtr->m_maxEdgeAngleThreshold)
	{
		cbtVector3 nearest;
		cbtNearestPointInLineSegment(cp.m_localPointB, v0, v1, nearest);
		cbtScalar len = (contact - nearest).length();
		//
		if (len < disttobestedge)
		{
			bestedge = 0;
			disttobestedge = len;
		}
	}
	// Edge 1 -> 2
	if (cbtFabs(info->m_edgeV1V2Angle) < triangleInfoMapPtr->m_maxEdgeAngleThreshold)
	{
		cbtVector3 nearest;
		cbtNearestPointInLineSegment(cp.m_localPointB, v1, v2, nearest);
		cbtScalar len = (contact - nearest).length();
		//
		if (len < disttobestedge)
		{
			bestedge = 1;
			disttobestedge = len;
		}
	}
	// Edge 2 -> 0
	if (cbtFabs(info->m_edgeV2V0Angle) < triangleInfoMapPtr->m_maxEdgeAngleThreshold)
	{
		cbtVector3 nearest;
		cbtNearestPointInLineSegment(cp.m_localPointB, v2, v0, nearest);
		cbtScalar len = (contact - nearest).length();
		//
		if (len < disttobestedge)
		{
			bestedge = 2;
			disttobestedge = len;
		}
	}

#ifdef BT_INTERNAL_EDGE_DEBUG_DRAW
	cbtVector3 upfix = tri_normal * cbtVector3(0.1f, 0.1f, 0.1f);
	cbtDebugDrawLine(tr * v0 + upfix, tr * v1 + upfix, red);
#endif
	if (cbtFabs(info->m_edgeV0V1Angle) < triangleInfoMapPtr->m_maxEdgeAngleThreshold)
	{
#ifdef BT_INTERNAL_EDGE_DEBUG_DRAW
		cbtDebugDrawLine(tr * contact, tr * (contact + cp.m_normalWorldOnB * 10), black);
#endif
		cbtScalar len = (contact - nearest).length();
		if (len < triangleInfoMapPtr->m_edgeDistanceThreshold)
			if (bestedge == 0)
			{
				cbtVector3 edge(v0 - v1);
				isNearEdge = true;

				if (info->m_edgeV0V1Angle == cbtScalar(0))
				{
					numConcaveEdgeHits++;
				}
				else
				{
					bool isEdgeConvex = (info->m_flags & TRI_INFO_V0V1_CONVEX);
					cbtScalar swapFactor = isEdgeConvex ? cbtScalar(1) : cbtScalar(-1);
#ifdef BT_INTERNAL_EDGE_DEBUG_DRAW
					cbtDebugDrawLine(tr * nearest, tr * (nearest + swapFactor * tri_normal * 10), white);
#endif  //BT_INTERNAL_EDGE_DEBUG_DRAW

					cbtVector3 nA = swapFactor * tri_normal;

					cbtQuaternion orn(edge, info->m_edgeV0V1Angle);
					cbtVector3 computedNormalB = quatRotate(orn, tri_normal);
					if (info->m_flags & TRI_INFO_V0V1_SWAP_NORMALB)
						computedNormalB *= -1;
					cbtVector3 nB = swapFactor * computedNormalB;

					cbtScalar NdotA = localContactNormalOnB.dot(nA);
					cbtScalar NdotB = localContactNormalOnB.dot(nB);
					bool backFacingNormal = (NdotA < triangleInfoMapPtr->m_convexEpsilon) && (NdotB < triangleInfoMapPtr->m_convexEpsilon);

#ifdef DEBUG_INTERNAL_EDGE
					{
						cbtDebugDrawLine(cp.getPositionWorldOnB(), cp.getPositionWorldOnB() + tr.getBasis() * (nB * 20), red);
					}
#endif  //DEBUG_INTERNAL_EDGE

					if (backFacingNormal)
					{
						numConcaveEdgeHits++;
					}
					else
					{
						numConvexEdgeHits++;
						cbtVector3 clampedLocalNormal;
						bool isClamped = cbtClampNormal(edge, swapFactor * tri_normal, localContactNormalOnB, info->m_edgeV0V1Angle, clampedLocalNormal);
						if (isClamped)
						{
							if (((normalAdjustFlags & BT_TRIANGLE_CONVEX_DOUBLE_SIDED) != 0) || (clampedLocalNormal.dot(frontFacing * tri_normal) > 0))
							{
								cbtVector3 newNormal = colObj0Wrap->getWorldTransform().getBasis() * clampedLocalNormal;
								//					cp.m_distance1 = cp.m_distance1 * newNormal.dot(cp.m_normalWorldOnB);
								cp.m_normalWorldOnB = newNormal;
								// Reproject collision point along normal. (what about cp.m_distance1?)
								cp.m_positionWorldOnB = cp.m_positionWorldOnA - cp.m_normalWorldOnB * cp.m_distance1;
								cp.m_localPointB = colObj0Wrap->getWorldTransform().invXform(cp.m_positionWorldOnB);
							}
						}
					}
				}
			}
	}

	cbtNearestPointInLineSegment(contact, v1, v2, nearest);
#ifdef BT_INTERNAL_EDGE_DEBUG_DRAW
	cbtDebugDrawLine(tr * nearest, tr * cp.m_localPointB, green);
#endif  //BT_INTERNAL_EDGE_DEBUG_DRAW

#ifdef BT_INTERNAL_EDGE_DEBUG_DRAW
	cbtDebugDrawLine(tr * v1 + upfix, tr * v2 + upfix, green);
#endif

	if (cbtFabs(info->m_edgeV1V2Angle) < triangleInfoMapPtr->m_maxEdgeAngleThreshold)
	{
#ifdef BT_INTERNAL_EDGE_DEBUG_DRAW
		cbtDebugDrawLine(tr * contact, tr * (contact + cp.m_normalWorldOnB * 10), black);
#endif  //BT_INTERNAL_EDGE_DEBUG_DRAW

		cbtScalar len = (contact - nearest).length();
		if (len < triangleInfoMapPtr->m_edgeDistanceThreshold)
			if (bestedge == 1)
			{
				isNearEdge = true;
#ifdef BT_INTERNAL_EDGE_DEBUG_DRAW
				cbtDebugDrawLine(tr * nearest, tr * (nearest + tri_normal * 10), white);
#endif  //BT_INTERNAL_EDGE_DEBUG_DRAW

				cbtVector3 edge(v1 - v2);

				isNearEdge = true;

				if (info->m_edgeV1V2Angle == cbtScalar(0))
				{
					numConcaveEdgeHits++;
				}
				else
				{
					bool isEdgeConvex = (info->m_flags & TRI_INFO_V1V2_CONVEX) != 0;
					cbtScalar swapFactor = isEdgeConvex ? cbtScalar(1) : cbtScalar(-1);
#ifdef BT_INTERNAL_EDGE_DEBUG_DRAW
					cbtDebugDrawLine(tr * nearest, tr * (nearest + swapFactor * tri_normal * 10), white);
#endif  //BT_INTERNAL_EDGE_DEBUG_DRAW

					cbtVector3 nA = swapFactor * tri_normal;

					cbtQuaternion orn(edge, info->m_edgeV1V2Angle);
					cbtVector3 computedNormalB = quatRotate(orn, tri_normal);
					if (info->m_flags & TRI_INFO_V1V2_SWAP_NORMALB)
						computedNormalB *= -1;
					cbtVector3 nB = swapFactor * computedNormalB;

#ifdef DEBUG_INTERNAL_EDGE
					{
						cbtDebugDrawLine(cp.getPositionWorldOnB(), cp.getPositionWorldOnB() + tr.getBasis() * (nB * 20), red);
					}
#endif  //DEBUG_INTERNAL_EDGE

					cbtScalar NdotA = localContactNormalOnB.dot(nA);
					cbtScalar NdotB = localContactNormalOnB.dot(nB);
					bool backFacingNormal = (NdotA < triangleInfoMapPtr->m_convexEpsilon) && (NdotB < triangleInfoMapPtr->m_convexEpsilon);

					if (backFacingNormal)
					{
						numConcaveEdgeHits++;
					}
					else
					{
						numConvexEdgeHits++;
						cbtVector3 localContactNormalOnB = colObj0Wrap->getWorldTransform().getBasis().transpose() * cp.m_normalWorldOnB;
						cbtVector3 clampedLocalNormal;
						bool isClamped = cbtClampNormal(edge, swapFactor * tri_normal, localContactNormalOnB, info->m_edgeV1V2Angle, clampedLocalNormal);
						if (isClamped)
						{
							if (((normalAdjustFlags & BT_TRIANGLE_CONVEX_DOUBLE_SIDED) != 0) || (clampedLocalNormal.dot(frontFacing * tri_normal) > 0))
							{
								cbtVector3 newNormal = colObj0Wrap->getWorldTransform().getBasis() * clampedLocalNormal;
								//					cp.m_distance1 = cp.m_distance1 * newNormal.dot(cp.m_normalWorldOnB);
								cp.m_normalWorldOnB = newNormal;
								// Reproject collision point along normal.
								cp.m_positionWorldOnB = cp.m_positionWorldOnA - cp.m_normalWorldOnB * cp.m_distance1;
								cp.m_localPointB = colObj0Wrap->getWorldTransform().invXform(cp.m_positionWorldOnB);
							}
						}
					}
				}
			}
	}

	cbtNearestPointInLineSegment(contact, v2, v0, nearest);
#ifdef BT_INTERNAL_EDGE_DEBUG_DRAW
	cbtDebugDrawLine(tr * nearest, tr * cp.m_localPointB, blue);
#endif  //BT_INTERNAL_EDGE_DEBUG_DRAW
#ifdef BT_INTERNAL_EDGE_DEBUG_DRAW
	cbtDebugDrawLine(tr * v2 + upfix, tr * v0 + upfix, blue);
#endif

	if (cbtFabs(info->m_edgeV2V0Angle) < triangleInfoMapPtr->m_maxEdgeAngleThreshold)
	{
#ifdef BT_INTERNAL_EDGE_DEBUG_DRAW
		cbtDebugDrawLine(tr * contact, tr * (contact + cp.m_normalWorldOnB * 10), black);
#endif  //BT_INTERNAL_EDGE_DEBUG_DRAW

		cbtScalar len = (contact - nearest).length();
		if (len < triangleInfoMapPtr->m_edgeDistanceThreshold)
			if (bestedge == 2)
			{
				isNearEdge = true;
#ifdef BT_INTERNAL_EDGE_DEBUG_DRAW
				cbtDebugDrawLine(tr * nearest, tr * (nearest + tri_normal * 10), white);
#endif  //BT_INTERNAL_EDGE_DEBUG_DRAW

				cbtVector3 edge(v2 - v0);

				if (info->m_edgeV2V0Angle == cbtScalar(0))
				{
					numConcaveEdgeHits++;
				}
				else
				{
					bool isEdgeConvex = (info->m_flags & TRI_INFO_V2V0_CONVEX) != 0;
					cbtScalar swapFactor = isEdgeConvex ? cbtScalar(1) : cbtScalar(-1);
#ifdef BT_INTERNAL_EDGE_DEBUG_DRAW
					cbtDebugDrawLine(tr * nearest, tr * (nearest + swapFactor * tri_normal * 10), white);
#endif  //BT_INTERNAL_EDGE_DEBUG_DRAW

					cbtVector3 nA = swapFactor * tri_normal;
					cbtQuaternion orn(edge, info->m_edgeV2V0Angle);
					cbtVector3 computedNormalB = quatRotate(orn, tri_normal);
					if (info->m_flags & TRI_INFO_V2V0_SWAP_NORMALB)
						computedNormalB *= -1;
					cbtVector3 nB = swapFactor * computedNormalB;

#ifdef DEBUG_INTERNAL_EDGE
					{
						cbtDebugDrawLine(cp.getPositionWorldOnB(), cp.getPositionWorldOnB() + tr.getBasis() * (nB * 20), red);
					}
#endif  //DEBUG_INTERNAL_EDGE

					cbtScalar NdotA = localContactNormalOnB.dot(nA);
					cbtScalar NdotB = localContactNormalOnB.dot(nB);
					bool backFacingNormal = (NdotA < triangleInfoMapPtr->m_convexEpsilon) && (NdotB < triangleInfoMapPtr->m_convexEpsilon);

					if (backFacingNormal)
					{
						numConcaveEdgeHits++;
					}
					else
					{
						numConvexEdgeHits++;
						//				printf("hitting convex edge\n");

						cbtVector3 localContactNormalOnB = colObj0Wrap->getWorldTransform().getBasis().transpose() * cp.m_normalWorldOnB;
						cbtVector3 clampedLocalNormal;
						bool isClamped = cbtClampNormal(edge, swapFactor * tri_normal, localContactNormalOnB, info->m_edgeV2V0Angle, clampedLocalNormal);
						if (isClamped)
						{
							if (((normalAdjustFlags & BT_TRIANGLE_CONVEX_DOUBLE_SIDED) != 0) || (clampedLocalNormal.dot(frontFacing * tri_normal) > 0))
							{
								cbtVector3 newNormal = colObj0Wrap->getWorldTransform().getBasis() * clampedLocalNormal;
								//					cp.m_distance1 = cp.m_distance1 * newNormal.dot(cp.m_normalWorldOnB);
								cp.m_normalWorldOnB = newNormal;
								// Reproject collision point along normal.
								cp.m_positionWorldOnB = cp.m_positionWorldOnA - cp.m_normalWorldOnB * cp.m_distance1;
								cp.m_localPointB = colObj0Wrap->getWorldTransform().invXform(cp.m_positionWorldOnB);
							}
						}
					}
				}
			}
	}

#ifdef DEBUG_INTERNAL_EDGE
	{
		cbtVector3 color(0, 1, 1);
		cbtDebugDrawLine(cp.getPositionWorldOnB(), cp.getPositionWorldOnB() + cp.m_normalWorldOnB * 10, color);
	}
#endif  //DEBUG_INTERNAL_EDGE

	if (isNearEdge)
	{
		if (numConcaveEdgeHits > 0)
		{
			if ((normalAdjustFlags & BT_TRIANGLE_CONCAVE_DOUBLE_SIDED) != 0)
			{
				//fix tri_normal so it pointing the same direction as the current local contact normal
				if (tri_normal.dot(localContactNormalOnB) < 0)
				{
					tri_normal *= -1;
				}
				cp.m_normalWorldOnB = colObj0Wrap->getWorldTransform().getBasis() * tri_normal;
			}
			else
			{
				cbtVector3 newNormal = tri_normal * frontFacing;
				//if the tri_normal is pointing opposite direction as the current local contact normal, skip it
				cbtScalar d = newNormal.dot(localContactNormalOnB);
				if (d < 0)
				{
					return;
				}
				//modify the normal to be the triangle normal (or backfacing normal)
				cp.m_normalWorldOnB = colObj0Wrap->getWorldTransform().getBasis() * newNormal;
			}

			// Reproject collision point along normal.
			cp.m_positionWorldOnB = cp.m_positionWorldOnA - cp.m_normalWorldOnB * cp.m_distance1;
			cp.m_localPointB = colObj0Wrap->getWorldTransform().invXform(cp.m_positionWorldOnB);
		}
	}
}
