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

//#define DISABLE_BVH

#include "BulletCollision/CollisionShapes/cbtBvhTriangleMeshShape.h"
#include "BulletCollision/CollisionShapes/cbtOptimizedBvh.h"
#include "LinearMath/cbtSerializer.h"

///Bvh Concave triangle mesh is a static-triangle mesh shape with Bounding Volume Hierarchy optimization.
///Uses an interface to access the triangles to allow for sharing graphics/physics triangles.
cbtBvhTriangleMeshShape::cbtBvhTriangleMeshShape(cbtStridingMeshInterface* meshInterface, bool useQuantizedAabbCompression, bool buildBvh)
	: cbtTriangleMeshShape(meshInterface),
	  m_bvh(0),
	  m_triangleInfoMap(0),
	  m_useQuantizedAabbCompression(useQuantizedAabbCompression),
	  m_ownsBvh(false)
{
	m_shapeType = TRIANGLE_MESH_SHAPE_PROXYTYPE;
	//construct bvh from meshInterface
#ifndef DISABLE_BVH

	if (buildBvh)
	{
		buildOptimizedBvh();
	}

#endif  //DISABLE_BVH
}

cbtBvhTriangleMeshShape::cbtBvhTriangleMeshShape(cbtStridingMeshInterface* meshInterface, bool useQuantizedAabbCompression, const cbtVector3& bvhAabbMin, const cbtVector3& bvhAabbMax, bool buildBvh)
	: cbtTriangleMeshShape(meshInterface),
	  m_bvh(0),
	  m_triangleInfoMap(0),
	  m_useQuantizedAabbCompression(useQuantizedAabbCompression),
	  m_ownsBvh(false)
{
	m_shapeType = TRIANGLE_MESH_SHAPE_PROXYTYPE;
	//construct bvh from meshInterface
#ifndef DISABLE_BVH

	if (buildBvh)
	{
		void* mem = cbtAlignedAlloc(sizeof(cbtOptimizedBvh), 16);
		m_bvh = new (mem) cbtOptimizedBvh();

		m_bvh->build(meshInterface, m_useQuantizedAabbCompression, bvhAabbMin, bvhAabbMax);
		m_ownsBvh = true;
	}

#endif  //DISABLE_BVH
}

void cbtBvhTriangleMeshShape::partialRefitTree(const cbtVector3& aabbMin, const cbtVector3& aabbMax)
{
	m_bvh->refitPartial(m_meshInterface, aabbMin, aabbMax);

	m_localAabbMin.setMin(aabbMin);
	m_localAabbMax.setMax(aabbMax);
}

void cbtBvhTriangleMeshShape::refitTree(const cbtVector3& aabbMin, const cbtVector3& aabbMax)
{
	m_bvh->refit(m_meshInterface, aabbMin, aabbMax);

	recalcLocalAabb();
}

cbtBvhTriangleMeshShape::~cbtBvhTriangleMeshShape()
{
	if (m_ownsBvh)
	{
		m_bvh->~cbtOptimizedBvh();
		cbtAlignedFree(m_bvh);
	}
}

void cbtBvhTriangleMeshShape::performRaycast(cbtTriangleCallback* callback, const cbtVector3& raySource, const cbtVector3& rayTarget)
{
	struct MyNodeOverlapCallback : public cbtNodeOverlapCallback
	{
		cbtStridingMeshInterface* m_meshInterface;
		cbtTriangleCallback* m_callback;

		MyNodeOverlapCallback(cbtTriangleCallback* callback, cbtStridingMeshInterface* meshInterface)
			: m_meshInterface(meshInterface),
			  m_callback(callback)
		{
		}

		virtual void processNode(int nodeSubPart, int nodeTriangleIndex)
		{
			cbtVector3 m_triangle[3];
			const unsigned char* vertexbase;
			int numverts;
			PHY_ScalarType type;
			int stride;
			const unsigned char* indexbase;
			int indexstride;
			int numfaces;
			PHY_ScalarType indicestype;

			m_meshInterface->getLockedReadOnlyVertexIndexBase(
				&vertexbase,
				numverts,
				type,
				stride,
				&indexbase,
				indexstride,
				numfaces,
				indicestype,
				nodeSubPart);

			unsigned int* gfxbase = (unsigned int*)(indexbase + nodeTriangleIndex * indexstride);
			cbtAssert(indicestype == PHY_INTEGER || indicestype == PHY_SHORT);

			const cbtVector3& meshScaling = m_meshInterface->getScaling();
			for (int j = 2; j >= 0; j--)
			{
				int graphicsindex = indicestype == PHY_SHORT ? ((unsigned short*)gfxbase)[j] : gfxbase[j];

				if (type == PHY_FLOAT)
				{
					float* graphicsbase = (float*)(vertexbase + graphicsindex * stride);

					m_triangle[j] = cbtVector3(graphicsbase[0] * meshScaling.getX(), graphicsbase[1] * meshScaling.getY(), graphicsbase[2] * meshScaling.getZ());
				}
				else
				{
					double* graphicsbase = (double*)(vertexbase + graphicsindex * stride);

					m_triangle[j] = cbtVector3(cbtScalar(graphicsbase[0]) * meshScaling.getX(), cbtScalar(graphicsbase[1]) * meshScaling.getY(), cbtScalar(graphicsbase[2]) * meshScaling.getZ());
				}
			}

			/* Perform ray vs. triangle collision here */
			m_callback->processTriangle(m_triangle, nodeSubPart, nodeTriangleIndex);
			m_meshInterface->unLockReadOnlyVertexBase(nodeSubPart);
		}
	};

	MyNodeOverlapCallback myNodeCallback(callback, m_meshInterface);

	m_bvh->reportRayOverlappingNodex(&myNodeCallback, raySource, rayTarget);
}

void cbtBvhTriangleMeshShape::performConvexcast(cbtTriangleCallback* callback, const cbtVector3& raySource, const cbtVector3& rayTarget, const cbtVector3& aabbMin, const cbtVector3& aabbMax)
{
	struct MyNodeOverlapCallback : public cbtNodeOverlapCallback
	{
		cbtStridingMeshInterface* m_meshInterface;
		cbtTriangleCallback* m_callback;

		MyNodeOverlapCallback(cbtTriangleCallback* callback, cbtStridingMeshInterface* meshInterface)
			: m_meshInterface(meshInterface),
			  m_callback(callback)
		{
		}

		virtual void processNode(int nodeSubPart, int nodeTriangleIndex)
		{
			cbtVector3 m_triangle[3];
			const unsigned char* vertexbase;
			int numverts;
			PHY_ScalarType type;
			int stride;
			const unsigned char* indexbase;
			int indexstride;
			int numfaces;
			PHY_ScalarType indicestype;

			m_meshInterface->getLockedReadOnlyVertexIndexBase(
				&vertexbase,
				numverts,
				type,
				stride,
				&indexbase,
				indexstride,
				numfaces,
				indicestype,
				nodeSubPart);

			unsigned int* gfxbase = (unsigned int*)(indexbase + nodeTriangleIndex * indexstride);
			cbtAssert(indicestype == PHY_INTEGER || indicestype == PHY_SHORT);

			const cbtVector3& meshScaling = m_meshInterface->getScaling();
			for (int j = 2; j >= 0; j--)
			{
				int graphicsindex = indicestype == PHY_SHORT ? ((unsigned short*)gfxbase)[j] : gfxbase[j];

				if (type == PHY_FLOAT)
				{
					float* graphicsbase = (float*)(vertexbase + graphicsindex * stride);

					m_triangle[j] = cbtVector3(graphicsbase[0] * meshScaling.getX(), graphicsbase[1] * meshScaling.getY(), graphicsbase[2] * meshScaling.getZ());
				}
				else
				{
					double* graphicsbase = (double*)(vertexbase + graphicsindex * stride);

					m_triangle[j] = cbtVector3(cbtScalar(graphicsbase[0]) * meshScaling.getX(), cbtScalar(graphicsbase[1]) * meshScaling.getY(), cbtScalar(graphicsbase[2]) * meshScaling.getZ());
				}
			}

			/* Perform ray vs. triangle collision here */
			m_callback->processTriangle(m_triangle, nodeSubPart, nodeTriangleIndex);
			m_meshInterface->unLockReadOnlyVertexBase(nodeSubPart);
		}
	};

	MyNodeOverlapCallback myNodeCallback(callback, m_meshInterface);

	m_bvh->reportBoxCastOverlappingNodex(&myNodeCallback, raySource, rayTarget, aabbMin, aabbMax);
}

//perform bvh tree traversal and report overlapping triangles to 'callback'
void cbtBvhTriangleMeshShape::processAllTriangles(cbtTriangleCallback* callback, const cbtVector3& aabbMin, const cbtVector3& aabbMax) const
{
#ifdef DISABLE_BVH
	//brute force traverse all triangles
	cbtTriangleMeshShape::processAllTriangles(callback, aabbMin, aabbMax);
#else

	//first get all the nodes

	struct MyNodeOverlapCallback : public cbtNodeOverlapCallback
	{
		cbtStridingMeshInterface* m_meshInterface;
		cbtTriangleCallback* m_callback;
		cbtVector3 m_triangle[3];
		int m_numOverlap;

		MyNodeOverlapCallback(cbtTriangleCallback* callback, cbtStridingMeshInterface* meshInterface)
			: m_meshInterface(meshInterface),
			  m_callback(callback),
			  m_numOverlap(0)
		{
		}

		virtual void processNode(int nodeSubPart, int nodeTriangleIndex)
		{
			m_numOverlap++;
			const unsigned char* vertexbase;
			int numverts;
			PHY_ScalarType type;
			int stride;
			const unsigned char* indexbase;
			int indexstride;
			int numfaces;
			PHY_ScalarType indicestype;

			m_meshInterface->getLockedReadOnlyVertexIndexBase(
				&vertexbase,
				numverts,
				type,
				stride,
				&indexbase,
				indexstride,
				numfaces,
				indicestype,
				nodeSubPart);

			unsigned int* gfxbase = (unsigned int*)(indexbase + nodeTriangleIndex * indexstride);
			cbtAssert(indicestype == PHY_INTEGER || indicestype == PHY_SHORT || indicestype == PHY_UCHAR);

			const cbtVector3& meshScaling = m_meshInterface->getScaling();
			for (int j = 2; j >= 0; j--)
			{
				int graphicsindex = indicestype == PHY_SHORT ? ((unsigned short*)gfxbase)[j] : indicestype == PHY_INTEGER ? gfxbase[j] : ((unsigned char*)gfxbase)[j];

#ifdef DEBUG_TRIANGLE_MESH
				printf("%d ,", graphicsindex);
#endif  //DEBUG_TRIANGLE_MESH
				if (type == PHY_FLOAT)
				{
					float* graphicsbase = (float*)(vertexbase + graphicsindex * stride);

					m_triangle[j] = cbtVector3(
						graphicsbase[0] * meshScaling.getX(),
						graphicsbase[1] * meshScaling.getY(),
						graphicsbase[2] * meshScaling.getZ());
				}
				else
				{
					double* graphicsbase = (double*)(vertexbase + graphicsindex * stride);

					m_triangle[j] = cbtVector3(
						cbtScalar(graphicsbase[0]) * meshScaling.getX(),
						cbtScalar(graphicsbase[1]) * meshScaling.getY(),
						cbtScalar(graphicsbase[2]) * meshScaling.getZ());
				}
#ifdef DEBUG_TRIANGLE_MESH
				printf("triangle vertices:%f,%f,%f\n", triangle[j].x(), triangle[j].y(), triangle[j].z());
#endif  //DEBUG_TRIANGLE_MESH
			}

			m_callback->processTriangle(m_triangle, nodeSubPart, nodeTriangleIndex);
			m_meshInterface->unLockReadOnlyVertexBase(nodeSubPart);
		}
	};

	MyNodeOverlapCallback myNodeCallback(callback, m_meshInterface);

	m_bvh->reportAabbOverlappingNodex(&myNodeCallback, aabbMin, aabbMax);

#endif  //DISABLE_BVH
}

void cbtBvhTriangleMeshShape::setLocalScaling(const cbtVector3& scaling)
{
	if ((getLocalScaling() - scaling).length2() > SIMD_EPSILON)
	{
		cbtTriangleMeshShape::setLocalScaling(scaling);
		buildOptimizedBvh();
	}
}

void cbtBvhTriangleMeshShape::buildOptimizedBvh()
{
	if (m_ownsBvh)
	{
		m_bvh->~cbtOptimizedBvh();
		cbtAlignedFree(m_bvh);
	}
	///m_localAabbMin/m_localAabbMax is already re-calculated in cbtTriangleMeshShape. We could just scale aabb, but this needs some more work
	void* mem = cbtAlignedAlloc(sizeof(cbtOptimizedBvh), 16);
	m_bvh = new (mem) cbtOptimizedBvh();
	//rebuild the bvh...
	m_bvh->build(m_meshInterface, m_useQuantizedAabbCompression, m_localAabbMin, m_localAabbMax);
	m_ownsBvh = true;
}

void cbtBvhTriangleMeshShape::setOptimizedBvh(cbtOptimizedBvh* bvh, const cbtVector3& scaling)
{
	cbtAssert(!m_bvh);
	cbtAssert(!m_ownsBvh);

	m_bvh = bvh;
	m_ownsBvh = false;
	// update the scaling without rebuilding the bvh
	if ((getLocalScaling() - scaling).length2() > SIMD_EPSILON)
	{
		cbtTriangleMeshShape::setLocalScaling(scaling);
	}
}

///fills the dataBuffer and returns the struct name (and 0 on failure)
const char* cbtBvhTriangleMeshShape::serialize(void* dataBuffer, cbtSerializer* serializer) const
{
	cbtTriangleMeshShapeData* trimeshData = (cbtTriangleMeshShapeData*)dataBuffer;

	cbtCollisionShape::serialize(&trimeshData->m_collisionShapeData, serializer);

	m_meshInterface->serialize(&trimeshData->m_meshInterface, serializer);

	trimeshData->m_collisionMargin = float(m_collisionMargin);

	if (m_bvh && !(serializer->getSerializationFlags() & BT_SERIALIZE_NO_BVH))
	{
		void* chunk = serializer->findPointer(m_bvh);
		if (chunk)
		{
#ifdef BT_USE_DOUBLE_PRECISION
			trimeshData->m_quantizedDoubleBvh = (cbtQuantizedBvhData*)chunk;
			trimeshData->m_quantizedFloatBvh = 0;
#else
			trimeshData->m_quantizedFloatBvh = (cbtQuantizedBvhData*)chunk;
			trimeshData->m_quantizedDoubleBvh = 0;
#endif  //BT_USE_DOUBLE_PRECISION
		}
		else
		{
#ifdef BT_USE_DOUBLE_PRECISION
			trimeshData->m_quantizedDoubleBvh = (cbtQuantizedBvhData*)serializer->getUniquePointer(m_bvh);
			trimeshData->m_quantizedFloatBvh = 0;
#else
			trimeshData->m_quantizedFloatBvh = (cbtQuantizedBvhData*)serializer->getUniquePointer(m_bvh);
			trimeshData->m_quantizedDoubleBvh = 0;
#endif  //BT_USE_DOUBLE_PRECISION

			int sz = m_bvh->calculateSerializeBufferSizeNew();
			cbtChunk* chunk = serializer->allocate(sz, 1);
			const char* structType = m_bvh->serialize(chunk->m_oldPtr, serializer);
			serializer->finalizeChunk(chunk, structType, BT_QUANTIZED_BVH_CODE, m_bvh);
		}
	}
	else
	{
		trimeshData->m_quantizedFloatBvh = 0;
		trimeshData->m_quantizedDoubleBvh = 0;
	}

	if (m_triangleInfoMap && !(serializer->getSerializationFlags() & BT_SERIALIZE_NO_TRIANGLEINFOMAP))
	{
		void* chunk = serializer->findPointer(m_triangleInfoMap);
		if (chunk)
		{
			trimeshData->m_triangleInfoMap = (cbtTriangleInfoMapData*)chunk;
		}
		else
		{
			trimeshData->m_triangleInfoMap = (cbtTriangleInfoMapData*)serializer->getUniquePointer(m_triangleInfoMap);
			int sz = m_triangleInfoMap->calculateSerializeBufferSize();
			cbtChunk* chunk = serializer->allocate(sz, 1);
			const char* structType = m_triangleInfoMap->serialize(chunk->m_oldPtr, serializer);
			serializer->finalizeChunk(chunk, structType, BT_TRIANLGE_INFO_MAP, m_triangleInfoMap);
		}
	}
	else
	{
		trimeshData->m_triangleInfoMap = 0;
	}

	// Fill padding with zeros to appease msan.
	memset(trimeshData->m_pad3, 0, sizeof(trimeshData->m_pad3));

	return "cbtTriangleMeshShapeData";
}

void cbtBvhTriangleMeshShape::serializeSingleBvh(cbtSerializer* serializer) const
{
	if (m_bvh)
	{
		int len = m_bvh->calculateSerializeBufferSizeNew();  //make sure not to use calculateSerializeBufferSize because it is used for in-place
		cbtChunk* chunk = serializer->allocate(len, 1);
		const char* structType = m_bvh->serialize(chunk->m_oldPtr, serializer);
		serializer->finalizeChunk(chunk, structType, BT_QUANTIZED_BVH_CODE, (void*)m_bvh);
	}
}

void cbtBvhTriangleMeshShape::serializeSingleTriangleInfoMap(cbtSerializer* serializer) const
{
	if (m_triangleInfoMap)
	{
		int len = m_triangleInfoMap->calculateSerializeBufferSize();
		cbtChunk* chunk = serializer->allocate(len, 1);
		const char* structType = m_triangleInfoMap->serialize(chunk->m_oldPtr, serializer);
		serializer->finalizeChunk(chunk, structType, BT_TRIANLGE_INFO_MAP, (void*)m_triangleInfoMap);
	}
}
