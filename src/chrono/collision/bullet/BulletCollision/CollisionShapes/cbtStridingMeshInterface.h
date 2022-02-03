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

#ifndef BT_STRIDING_MESHINTERFACE_H
#define BT_STRIDING_MESHINTERFACE_H

#include "LinearMath/cbtVector3.h"
#include "cbtTriangleCallback.h"
#include "cbtConcaveShape.h"

///	The cbtStridingMeshInterface is the interface class for high performance generic access to triangle meshes, used in combination with cbtBvhTriangleMeshShape and some other collision shapes.
/// Using index striding of 3*sizeof(integer) it can use triangle arrays, using index striding of 1*sizeof(integer) it can handle triangle strips.
/// It allows for sharing graphics and collision meshes. Also it provides locking/unlocking of graphics meshes that are in gpu memory.
ATTRIBUTE_ALIGNED16(class)
cbtStridingMeshInterface
{
protected:
	cbtVector3 m_scaling;

public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	cbtStridingMeshInterface() : m_scaling(cbtScalar(1.), cbtScalar(1.), cbtScalar(1.))
	{
	}

	virtual ~cbtStridingMeshInterface();

	virtual void InternalProcessAllTriangles(cbtInternalTriangleIndexCallback * callback, const cbtVector3& aabbMin, const cbtVector3& aabbMax) const;

	///brute force method to calculate aabb
	void calculateAabbBruteForce(cbtVector3 & aabbMin, cbtVector3 & aabbMax);

	/// get read and write access to a subpart of a triangle mesh
	/// this subpart has a continuous array of vertices and indices
	/// in this way the mesh can be handled as chunks of memory with striding
	/// very similar to OpenGL vertexarray support
	/// make a call to unLockVertexBase when the read and write access is finished
	virtual void getLockedVertexIndexBase(unsigned char** vertexbase, int& numverts, PHY_ScalarType& type, int& stride, unsigned char** indexbase, int& indexstride, int& numfaces, PHY_ScalarType& indicestype, int subpart = 0) = 0;

	virtual void getLockedReadOnlyVertexIndexBase(const unsigned char** vertexbase, int& numverts, PHY_ScalarType& type, int& stride, const unsigned char** indexbase, int& indexstride, int& numfaces, PHY_ScalarType& indicestype, int subpart = 0) const = 0;

	/// unLockVertexBase finishes the access to a subpart of the triangle mesh
	/// make a call to unLockVertexBase when the read and write access (using getLockedVertexIndexBase) is finished
	virtual void unLockVertexBase(int subpart) = 0;

	virtual void unLockReadOnlyVertexBase(int subpart) const = 0;

	/// getNumSubParts returns the number of seperate subparts
	/// each subpart has a continuous array of vertices and indices
	virtual int getNumSubParts() const = 0;

	virtual void preallocateVertices(int numverts) = 0;
	virtual void preallocateIndices(int numindices) = 0;

	virtual bool hasPremadeAabb() const { return false; }
	virtual void setPremadeAabb(const cbtVector3& aabbMin, const cbtVector3& aabbMax) const
	{
		(void)aabbMin;
		(void)aabbMax;
	}
	virtual void getPremadeAabb(cbtVector3 * aabbMin, cbtVector3 * aabbMax) const
	{
		(void)aabbMin;
		(void)aabbMax;
	}

	const cbtVector3& getScaling() const
	{
		return m_scaling;
	}
	void setScaling(const cbtVector3& scaling)
	{
		m_scaling = scaling;
	}

	virtual int calculateSerializeBufferSize() const;

	///fills the dataBuffer and returns the struct name (and 0 on failure)
	virtual const char* serialize(void* dataBuffer, cbtSerializer* serializer) const;
};

struct cbtIntIndexData
{
	int m_value;
};

struct cbtShortIntIndexData
{
	short m_value;
	char m_pad[2];
};

struct cbtShortIntIndexTripletData
{
	short m_values[3];
	char m_pad[2];
};

struct cbtCharIndexTripletData
{
	unsigned char m_values[3];
	char m_pad;
};

// clang-format off

///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct	cbtMeshPartData
{
	cbtVector3FloatData			*m_vertices3f;
	cbtVector3DoubleData			*m_vertices3d;

	cbtIntIndexData				*m_indices32;
	cbtShortIntIndexTripletData	*m_3indices16;
	cbtCharIndexTripletData		*m_3indices8;

	cbtShortIntIndexData			*m_indices16;//backwards compatibility

	int                     m_numTriangles;//length of m_indices = m_numTriangles
	int                     m_numVertices;
};


///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct	cbtStridingMeshInterfaceData
{
	cbtMeshPartData	*m_meshPartsPtr;
	cbtVector3FloatData	m_scaling;
	int	m_numMeshParts;
	char m_padding[4];
};

// clang-format on

SIMD_FORCE_INLINE int cbtStridingMeshInterface::calculateSerializeBufferSize() const
{
	return sizeof(cbtStridingMeshInterfaceData);
}

#endif  //BT_STRIDING_MESHINTERFACE_H
