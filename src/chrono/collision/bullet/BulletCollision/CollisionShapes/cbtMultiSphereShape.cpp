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

#include "cbtMultiSphereShape.h"
#include "BulletCollision/CollisionShapes/cbtCollisionMargin.h"
#include "LinearMath/cbtQuaternion.h"
#include "LinearMath/cbtSerializer.h"

cbtMultiSphereShape::cbtMultiSphereShape(const cbtVector3* positions, const cbtScalar* radi, int numSpheres)
	: cbtConvexInternalAabbCachingShape()
{
	m_shapeType = MULTI_SPHERE_SHAPE_PROXYTYPE;
	//cbtScalar startMargin = cbtScalar(BT_LARGE_FLOAT);

	m_localPositionArray.resize(numSpheres);
	m_radiArray.resize(numSpheres);
	for (int i = 0; i < numSpheres; i++)
	{
		m_localPositionArray[i] = positions[i];
		m_radiArray[i] = radi[i];
	}

	recalcLocalAabb();
}

#ifndef MIN
#define MIN(_a, _b) ((_a) < (_b) ? (_a) : (_b))
#endif
cbtVector3 cbtMultiSphereShape::localGetSupportingVertexWithoutMargin(const cbtVector3& vec0) const
{
	cbtVector3 supVec(0, 0, 0);

	cbtScalar maxDot(cbtScalar(-BT_LARGE_FLOAT));

	cbtVector3 vec = vec0;
	cbtScalar lenSqr = vec.length2();
	if (lenSqr < (SIMD_EPSILON * SIMD_EPSILON))
	{
		vec.setValue(1, 0, 0);
	}
	else
	{
		cbtScalar rlen = cbtScalar(1.) / cbtSqrt(lenSqr);
		vec *= rlen;
	}

	cbtVector3 vtx;
	cbtScalar newDot;

	const cbtVector3* pos = &m_localPositionArray[0];
	const cbtScalar* rad = &m_radiArray[0];
	int numSpheres = m_localPositionArray.size();

	for (int k = 0; k < numSpheres; k += 128)
	{
		cbtVector3 temp[128];
		int inner_count = MIN(numSpheres - k, 128);
		for (long i = 0; i < inner_count; i++)
		{
			temp[i] = (*pos) * m_localScaling + vec * m_localScaling * (*rad) - vec * getMargin();
			pos++;
			rad++;
		}
		long i = vec.maxDot(temp, inner_count, newDot);
		if (newDot > maxDot)
		{
			maxDot = newDot;
			supVec = temp[i];
		}
	}

	return supVec;
}

void cbtMultiSphereShape::batchedUnitVectorGetSupportingVertexWithoutMargin(const cbtVector3* vectors, cbtVector3* supportVerticesOut, int numVectors) const
{
	for (int j = 0; j < numVectors; j++)
	{
		cbtScalar maxDot(cbtScalar(-BT_LARGE_FLOAT));

		const cbtVector3& vec = vectors[j];

		cbtVector3 vtx;
		cbtScalar newDot;

		const cbtVector3* pos = &m_localPositionArray[0];
		const cbtScalar* rad = &m_radiArray[0];
		int numSpheres = m_localPositionArray.size();

		for (int k = 0; k < numSpheres; k += 128)
		{
			cbtVector3 temp[128];
			int inner_count = MIN(numSpheres - k, 128);
			for (long i = 0; i < inner_count; i++)
			{
				temp[i] = (*pos) * m_localScaling + vec * m_localScaling * (*rad) - vec * getMargin();
				pos++;
				rad++;
			}
			long i = vec.maxDot(temp, inner_count, newDot);
			if (newDot > maxDot)
			{
				maxDot = newDot;
				supportVerticesOut[j] = temp[i];
			}
		}
	}
}

void cbtMultiSphereShape::calculateLocalInertia(cbtScalar mass, cbtVector3& inertia) const
{
	//as an approximation, take the inertia of the box that bounds the spheres

	cbtVector3 localAabbMin, localAabbMax;
	getCachedLocalAabb(localAabbMin, localAabbMax);
	cbtVector3 halfExtents = (localAabbMax - localAabbMin) * cbtScalar(0.5);

	cbtScalar lx = cbtScalar(2.) * (halfExtents.x());
	cbtScalar ly = cbtScalar(2.) * (halfExtents.y());
	cbtScalar lz = cbtScalar(2.) * (halfExtents.z());

	inertia.setValue(mass / (cbtScalar(12.0)) * (ly * ly + lz * lz),
					 mass / (cbtScalar(12.0)) * (lx * lx + lz * lz),
					 mass / (cbtScalar(12.0)) * (lx * lx + ly * ly));
}

///fills the dataBuffer and returns the struct name (and 0 on failure)
const char* cbtMultiSphereShape::serialize(void* dataBuffer, cbtSerializer* serializer) const
{
	cbtMultiSphereShapeData* shapeData = (cbtMultiSphereShapeData*)dataBuffer;
	cbtConvexInternalShape::serialize(&shapeData->m_convexInternalShapeData, serializer);

	int numElem = m_localPositionArray.size();
	shapeData->m_localPositionArrayPtr = numElem ? (cbtPositionAndRadius*)serializer->getUniquePointer((void*)&m_localPositionArray[0]) : 0;

	shapeData->m_localPositionArraySize = numElem;
	if (numElem)
	{
		cbtChunk* chunk = serializer->allocate(sizeof(cbtPositionAndRadius), numElem);
		cbtPositionAndRadius* memPtr = (cbtPositionAndRadius*)chunk->m_oldPtr;
		for (int i = 0; i < numElem; i++, memPtr++)
		{
			m_localPositionArray[i].serializeFloat(memPtr->m_pos);
			memPtr->m_radius = float(m_radiArray[i]);
		}
		serializer->finalizeChunk(chunk, "cbtPositionAndRadius", BT_ARRAY_CODE, (void*)&m_localPositionArray[0]);
	}

	// Fill padding with zeros to appease msan.
	memset(shapeData->m_padding, 0, sizeof(shapeData->m_padding));

	return "cbtMultiSphereShapeData";
}
