/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2010 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef _BT_TRIANGLE_INFO_MAP_H
#define _BT_TRIANGLE_INFO_MAP_H

#include "LinearMath/cbtHashMap.h"
#include "LinearMath/cbtSerializer.h"

///for cbtTriangleInfo m_flags
#define TRI_INFO_V0V1_CONVEX 1
#define TRI_INFO_V1V2_CONVEX 2
#define TRI_INFO_V2V0_CONVEX 4

#define TRI_INFO_V0V1_SWAP_NORMALB 8
#define TRI_INFO_V1V2_SWAP_NORMALB 16
#define TRI_INFO_V2V0_SWAP_NORMALB 32

///The cbtTriangleInfo structure stores information to adjust collision normals to avoid collisions against internal edges
///it can be generated using
struct cbtTriangleInfo
{
	cbtTriangleInfo()
	{
		m_edgeV0V1Angle = SIMD_2_PI;
		m_edgeV1V2Angle = SIMD_2_PI;
		m_edgeV2V0Angle = SIMD_2_PI;
		m_flags = 0;
	}

	int m_flags;

	cbtScalar m_edgeV0V1Angle;
	cbtScalar m_edgeV1V2Angle;
	cbtScalar m_edgeV2V0Angle;
};

typedef cbtHashMap<cbtHashInt, cbtTriangleInfo> cbtInternalTriangleInfoMap;

///The cbtTriangleInfoMap stores edge angle information for some triangles. You can compute this information yourself or using cbtGenerateInternalEdgeInfo.
struct cbtTriangleInfoMap : public cbtInternalTriangleInfoMap
{
	cbtScalar m_convexEpsilon;          ///used to determine if an edge or contact normal is convex, using the dot product
	cbtScalar m_planarEpsilon;          ///used to determine if a triangle edge is planar with zero angle
	cbtScalar m_equalVertexThreshold;   ///used to compute connectivity: if the distance between two vertices is smaller than m_equalVertexThreshold, they are considered to be 'shared'
	cbtScalar m_edgeDistanceThreshold;  ///used to determine edge contacts: if the closest distance between a contact point and an edge is smaller than this distance threshold it is considered to "hit the edge"
	cbtScalar m_maxEdgeAngleThreshold;  //ignore edges that connect triangles at an angle larger than this m_maxEdgeAngleThreshold
	cbtScalar m_zeroAreaThreshold;      ///used to determine if a triangle is degenerate (length squared of cross product of 2 triangle edges < threshold)

	cbtTriangleInfoMap()
	{
		m_convexEpsilon = 0.00f;
		m_planarEpsilon = 0.0001f;
		m_equalVertexThreshold = cbtScalar(0.0001) * cbtScalar(0.0001);
		m_edgeDistanceThreshold = cbtScalar(0.1);
		m_zeroAreaThreshold = cbtScalar(0.0001) * cbtScalar(0.0001);
		m_maxEdgeAngleThreshold = SIMD_2_PI;
	}
	virtual ~cbtTriangleInfoMap() {}

	virtual int calculateSerializeBufferSize() const;

	///fills the dataBuffer and returns the struct name (and 0 on failure)
	virtual const char* serialize(void* dataBuffer, cbtSerializer* serializer) const;

	void deSerialize(struct cbtTriangleInfoMapData& data);
};

// clang-format off

///those fields have to be float and not cbtScalar for the serialization to work properly
struct	cbtTriangleInfoData
{
	int			m_flags;
	float	m_edgeV0V1Angle;
	float	m_edgeV1V2Angle;
	float	m_edgeV2V0Angle;
};

struct	cbtTriangleInfoMapData
{
	int					*m_hashTablePtr;
	int					*m_nextPtr;
	cbtTriangleInfoData	*m_valueArrayPtr;
	int					*m_keyArrayPtr;

	float	m_convexEpsilon;
	float	m_planarEpsilon;
	float	m_equalVertexThreshold; 
	float	m_edgeDistanceThreshold;
	float	m_zeroAreaThreshold;

	int		m_nextSize;
	int		m_hashTableSize;
	int		m_numValues;
	int		m_numKeys;
	char	m_padding[4];
};

// clang-format on

SIMD_FORCE_INLINE int cbtTriangleInfoMap::calculateSerializeBufferSize() const
{
	return sizeof(cbtTriangleInfoMapData);
}

///fills the dataBuffer and returns the struct name (and 0 on failure)
SIMD_FORCE_INLINE const char* cbtTriangleInfoMap::serialize(void* dataBuffer, cbtSerializer* serializer) const
{
	cbtTriangleInfoMapData* tmapData = (cbtTriangleInfoMapData*)dataBuffer;
	tmapData->m_convexEpsilon = (float)m_convexEpsilon;
	tmapData->m_planarEpsilon = (float)m_planarEpsilon;
	tmapData->m_equalVertexThreshold = (float)m_equalVertexThreshold;
	tmapData->m_edgeDistanceThreshold = (float)m_edgeDistanceThreshold;
	tmapData->m_zeroAreaThreshold = (float)m_zeroAreaThreshold;

	tmapData->m_hashTableSize = m_hashTable.size();

	tmapData->m_hashTablePtr = tmapData->m_hashTableSize ? (int*)serializer->getUniquePointer((void*)&m_hashTable[0]) : 0;
	if (tmapData->m_hashTablePtr)
	{
		//serialize an int buffer
		int sz = sizeof(int);
		int numElem = tmapData->m_hashTableSize;
		cbtChunk* chunk = serializer->allocate(sz, numElem);
		int* memPtr = (int*)chunk->m_oldPtr;
		for (int i = 0; i < numElem; i++, memPtr++)
		{
			*memPtr = m_hashTable[i];
		}
		serializer->finalizeChunk(chunk, "int", BT_ARRAY_CODE, (void*)&m_hashTable[0]);
	}

	tmapData->m_nextSize = m_next.size();
	tmapData->m_nextPtr = tmapData->m_nextSize ? (int*)serializer->getUniquePointer((void*)&m_next[0]) : 0;
	if (tmapData->m_nextPtr)
	{
		int sz = sizeof(int);
		int numElem = tmapData->m_nextSize;
		cbtChunk* chunk = serializer->allocate(sz, numElem);
		int* memPtr = (int*)chunk->m_oldPtr;
		for (int i = 0; i < numElem; i++, memPtr++)
		{
			*memPtr = m_next[i];
		}
		serializer->finalizeChunk(chunk, "int", BT_ARRAY_CODE, (void*)&m_next[0]);
	}

	tmapData->m_numValues = m_valueArray.size();
	tmapData->m_valueArrayPtr = tmapData->m_numValues ? (cbtTriangleInfoData*)serializer->getUniquePointer((void*)&m_valueArray[0]) : 0;
	if (tmapData->m_valueArrayPtr)
	{
		int sz = sizeof(cbtTriangleInfoData);
		int numElem = tmapData->m_numValues;
		cbtChunk* chunk = serializer->allocate(sz, numElem);
		cbtTriangleInfoData* memPtr = (cbtTriangleInfoData*)chunk->m_oldPtr;
		for (int i = 0; i < numElem; i++, memPtr++)
		{
			memPtr->m_edgeV0V1Angle = (float)m_valueArray[i].m_edgeV0V1Angle;
			memPtr->m_edgeV1V2Angle = (float)m_valueArray[i].m_edgeV1V2Angle;
			memPtr->m_edgeV2V0Angle = (float)m_valueArray[i].m_edgeV2V0Angle;
			memPtr->m_flags = m_valueArray[i].m_flags;
		}
		serializer->finalizeChunk(chunk, "cbtTriangleInfoData", BT_ARRAY_CODE, (void*)&m_valueArray[0]);
	}

	tmapData->m_numKeys = m_keyArray.size();
	tmapData->m_keyArrayPtr = tmapData->m_numKeys ? (int*)serializer->getUniquePointer((void*)&m_keyArray[0]) : 0;
	if (tmapData->m_keyArrayPtr)
	{
		int sz = sizeof(int);
		int numElem = tmapData->m_numValues;
		cbtChunk* chunk = serializer->allocate(sz, numElem);
		int* memPtr = (int*)chunk->m_oldPtr;
		for (int i = 0; i < numElem; i++, memPtr++)
		{
			*memPtr = m_keyArray[i].getUid1();
		}
		serializer->finalizeChunk(chunk, "int", BT_ARRAY_CODE, (void*)&m_keyArray[0]);
	}

	// Fill padding with zeros to appease msan.
	tmapData->m_padding[0] = 0;
	tmapData->m_padding[1] = 0;
	tmapData->m_padding[2] = 0;
	tmapData->m_padding[3] = 0;

	return "cbtTriangleInfoMapData";
}

///fills the dataBuffer and returns the struct name (and 0 on failure)
SIMD_FORCE_INLINE void cbtTriangleInfoMap::deSerialize(cbtTriangleInfoMapData& tmapData)
{
	m_convexEpsilon = tmapData.m_convexEpsilon;
	m_planarEpsilon = tmapData.m_planarEpsilon;
	m_equalVertexThreshold = tmapData.m_equalVertexThreshold;
	m_edgeDistanceThreshold = tmapData.m_edgeDistanceThreshold;
	m_zeroAreaThreshold = tmapData.m_zeroAreaThreshold;
	m_hashTable.resize(tmapData.m_hashTableSize);
	int i = 0;
	for (i = 0; i < tmapData.m_hashTableSize; i++)
	{
		m_hashTable[i] = tmapData.m_hashTablePtr[i];
	}
	m_next.resize(tmapData.m_nextSize);
	for (i = 0; i < tmapData.m_nextSize; i++)
	{
		m_next[i] = tmapData.m_nextPtr[i];
	}
	m_valueArray.resize(tmapData.m_numValues);
	for (i = 0; i < tmapData.m_numValues; i++)
	{
		m_valueArray[i].m_edgeV0V1Angle = tmapData.m_valueArrayPtr[i].m_edgeV0V1Angle;
		m_valueArray[i].m_edgeV1V2Angle = tmapData.m_valueArrayPtr[i].m_edgeV1V2Angle;
		m_valueArray[i].m_edgeV2V0Angle = tmapData.m_valueArrayPtr[i].m_edgeV2V0Angle;
		m_valueArray[i].m_flags = tmapData.m_valueArrayPtr[i].m_flags;
	}

	m_keyArray.resize(tmapData.m_numKeys, cbtHashInt(0));
	for (i = 0; i < tmapData.m_numKeys; i++)
	{
		m_keyArray[i].setUid1(tmapData.m_keyArrayPtr[i]);
	}
}

#endif  //_BT_TRIANGLE_INFO_MAP_H
