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

#if defined(_WIN32) || defined(__i386__)
#define BT_USE_SSE_IN_API
#endif

#include "cbtConvexHullShape.h"
#include "BulletCollision/CollisionShapes/cbtCollisionMargin.h"

#include "LinearMath/cbtQuaternion.h"
#include "LinearMath/cbtSerializer.h"
#include "cbtConvexPolyhedron.h"
#include "LinearMath/cbtConvexHullComputer.h"

cbtConvexHullShape ::cbtConvexHullShape(const cbtScalar* points, int numPoints, int stride) : cbtPolyhedralConvexAabbCachingShape()
{
	m_shapeType = CONVEX_HULL_SHAPE_PROXYTYPE;
	m_unscaledPoints.resize(numPoints);

	unsigned char* pointsAddress = (unsigned char*)points;

	for (int i = 0; i < numPoints; i++)
	{
		cbtScalar* point = (cbtScalar*)pointsAddress;
		m_unscaledPoints[i] = cbtVector3(point[0], point[1], point[2]);
		pointsAddress += stride;
	}

	recalcLocalAabb();
}

void cbtConvexHullShape::setLocalScaling(const cbtVector3& scaling)
{
	m_localScaling = scaling;
	recalcLocalAabb();
}

void cbtConvexHullShape::addPoint(const cbtVector3& point, bool recalculateLocalAabb)
{
	m_unscaledPoints.push_back(point);
	if (recalculateLocalAabb)
		recalcLocalAabb();
}

cbtVector3 cbtConvexHullShape::localGetSupportingVertexWithoutMargin(const cbtVector3& vec) const
{
	cbtVector3 supVec(cbtScalar(0.), cbtScalar(0.), cbtScalar(0.));
	cbtScalar maxDot = cbtScalar(-BT_LARGE_FLOAT);

	// Here we take advantage of dot(a, b*c) = dot(a*b, c).  Note: This is true mathematically, but not numerically.
	if (0 < m_unscaledPoints.size())
	{
		cbtVector3 scaled = vec * m_localScaling;
		int index = (int)scaled.maxDot(&m_unscaledPoints[0], m_unscaledPoints.size(), maxDot);  // FIXME: may violate encapsulation of m_unscaledPoints
		return m_unscaledPoints[index] * m_localScaling;
	}

	return supVec;
}

void cbtConvexHullShape::batchedUnitVectorGetSupportingVertexWithoutMargin(const cbtVector3* vectors, cbtVector3* supportVerticesOut, int numVectors) const
{
	cbtScalar newDot;
	//use 'w' component of supportVerticesOut?
	{
		for (int i = 0; i < numVectors; i++)
		{
			supportVerticesOut[i][3] = cbtScalar(-BT_LARGE_FLOAT);
		}
	}

	for (int j = 0; j < numVectors; j++)
	{
		cbtVector3 vec = vectors[j] * m_localScaling;  // dot(a*b,c) = dot(a,b*c)
		if (0 < m_unscaledPoints.size())
		{
			int i = (int)vec.maxDot(&m_unscaledPoints[0], m_unscaledPoints.size(), newDot);
			supportVerticesOut[j] = getScaledPoint(i);
			supportVerticesOut[j][3] = newDot;
		}
		else
			supportVerticesOut[j][3] = -BT_LARGE_FLOAT;
	}
}

cbtVector3 cbtConvexHullShape::localGetSupportingVertex(const cbtVector3& vec) const
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

void cbtConvexHullShape::optimizeConvexHull()
{
	cbtConvexHullComputer conv;
	conv.compute(&m_unscaledPoints[0].getX(), sizeof(cbtVector3), m_unscaledPoints.size(), 0.f, 0.f);
	int numVerts = conv.vertices.size();
	m_unscaledPoints.resize(0);
	for (int i = 0; i < numVerts; i++)
	{
		m_unscaledPoints.push_back(conv.vertices[i]);
	}
}

//currently just for debugging (drawing), perhaps future support for algebraic continuous collision detection
//Please note that you can debug-draw cbtConvexHullShape with the Raytracer Demo
int cbtConvexHullShape::getNumVertices() const
{
	return m_unscaledPoints.size();
}

int cbtConvexHullShape::getNumEdges() const
{
	return m_unscaledPoints.size();
}

void cbtConvexHullShape::getEdge(int i, cbtVector3& pa, cbtVector3& pb) const
{
	int index0 = i % m_unscaledPoints.size();
	int index1 = (i + 1) % m_unscaledPoints.size();
	pa = getScaledPoint(index0);
	pb = getScaledPoint(index1);
}

void cbtConvexHullShape::getVertex(int i, cbtVector3& vtx) const
{
	vtx = getScaledPoint(i);
}

int cbtConvexHullShape::getNumPlanes() const
{
	return 0;
}

void cbtConvexHullShape::getPlane(cbtVector3&, cbtVector3&, int) const
{
	cbtAssert(0);
}

//not yet
bool cbtConvexHullShape::isInside(const cbtVector3&, cbtScalar) const
{
	cbtAssert(0);
	return false;
}

///fills the dataBuffer and returns the struct name (and 0 on failure)
const char* cbtConvexHullShape::serialize(void* dataBuffer, cbtSerializer* serializer) const
{
	//int szc = sizeof(cbtConvexHullShapeData);
	cbtConvexHullShapeData* shapeData = (cbtConvexHullShapeData*)dataBuffer;
	cbtConvexInternalShape::serialize(&shapeData->m_convexInternalShapeData, serializer);

	int numElem = m_unscaledPoints.size();
	shapeData->m_numUnscaledPoints = numElem;
#ifdef BT_USE_DOUBLE_PRECISION
	shapeData->m_unscaledPointsFloatPtr = 0;
	shapeData->m_unscaledPointsDoublePtr = numElem ? (cbtVector3Data*)serializer->getUniquePointer((void*)&m_unscaledPoints[0]) : 0;
#else
	shapeData->m_unscaledPointsFloatPtr = numElem ? (cbtVector3Data*)serializer->getUniquePointer((void*)&m_unscaledPoints[0]) : 0;
	shapeData->m_unscaledPointsDoublePtr = 0;
#endif

	if (numElem)
	{
		int sz = sizeof(cbtVector3Data);
		//	int sz2 = sizeof(cbtVector3DoubleData);
		//	int sz3 = sizeof(cbtVector3FloatData);
		cbtChunk* chunk = serializer->allocate(sz, numElem);
		cbtVector3Data* memPtr = (cbtVector3Data*)chunk->m_oldPtr;
		for (int i = 0; i < numElem; i++, memPtr++)
		{
			m_unscaledPoints[i].serialize(*memPtr);
		}
		serializer->finalizeChunk(chunk, cbtVector3DataName, BT_ARRAY_CODE, (void*)&m_unscaledPoints[0]);
	}

	// Fill padding with zeros to appease msan.
	memset(shapeData->m_padding3, 0, sizeof(shapeData->m_padding3));

	return "cbtConvexHullShapeData";
}

void cbtConvexHullShape::project(const cbtTransform& trans, const cbtVector3& dir, cbtScalar& minProj, cbtScalar& maxProj, cbtVector3& witnesPtMin, cbtVector3& witnesPtMax) const
{
#if 1
	minProj = FLT_MAX;
	maxProj = -FLT_MAX;

	int numVerts = m_unscaledPoints.size();
	for (int i = 0; i < numVerts; i++)
	{
		cbtVector3 vtx = m_unscaledPoints[i] * m_localScaling;
		cbtVector3 pt = trans * vtx;
		cbtScalar dp = pt.dot(dir);
		if (dp < minProj)
		{
			minProj = dp;
			witnesPtMin = pt;
		}
		if (dp > maxProj)
		{
			maxProj = dp;
			witnesPtMax = pt;
		}
	}
#else
	cbtVector3 localAxis = dir * trans.getBasis();
	witnesPtMin = trans(localGetSupportingVertex(localAxis));
	witnesPtMax = trans(localGetSupportingVertex(-localAxis));

	minProj = witnesPtMin.dot(dir);
	maxProj = witnesPtMax.dot(dir);
#endif

	if (minProj > maxProj)
	{
		cbtSwap(minProj, maxProj);
		cbtSwap(witnesPtMin, witnesPtMax);
	}
}
