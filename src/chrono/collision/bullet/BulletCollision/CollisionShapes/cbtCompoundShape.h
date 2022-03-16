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

#ifndef BT_COMPOUND_SHAPE_H
#define BT_COMPOUND_SHAPE_H

#include "cbtCollisionShape.h"

#include "LinearMath/cbtVector3.h"
#include "LinearMath/cbtTransform.h"
#include "LinearMath/cbtMatrix3x3.h"
#include "cbtCollisionMargin.h"
#include "LinearMath/cbtAlignedObjectArray.h"

//class cbtOptimizedBvh;
struct cbtDbvt;

ATTRIBUTE_ALIGNED16(struct)
cbtCompoundShapeChild
{
	BT_DECLARE_ALIGNED_ALLOCATOR();

	cbtTransform m_transform;
	cbtCollisionShape* m_childShape;
	int m_childShapeType;
	cbtScalar m_childMargin;
	struct cbtDbvtNode* m_node;
};

SIMD_FORCE_INLINE bool operator==(const cbtCompoundShapeChild& c1, const cbtCompoundShapeChild& c2)
{
	return (c1.m_transform == c2.m_transform &&
			c1.m_childShape == c2.m_childShape &&
			c1.m_childShapeType == c2.m_childShapeType &&
			c1.m_childMargin == c2.m_childMargin);
}

/// The cbtCompoundShape allows to store multiple other cbtCollisionShapes
/// This allows for moving concave collision objects. This is more general then the static concave cbtBvhTriangleMeshShape.
/// It has an (optional) dynamic aabb tree to accelerate early rejection tests.
/// @todo: This aabb tree can also be use to speed up ray tests on cbtCompoundShape, see http://code.google.com/p/bullet/issues/detail?id=25
/// Currently, removal of child shapes is only supported when disabling the aabb tree (pass 'false' in the constructor of cbtCompoundShape)
ATTRIBUTE_ALIGNED16(class)
cbtCompoundShape : public cbtCollisionShape
{
protected:
	cbtAlignedObjectArray<cbtCompoundShapeChild> m_children;
	cbtVector3 m_localAabbMin;
	cbtVector3 m_localAabbMax;

	cbtDbvt* m_dynamicAabbTree;

	///increment m_updateRevision when adding/removing/replacing child shapes, so that some caches can be updated
	int m_updateRevision;

	cbtScalar m_collisionMargin;

	cbtVector3 m_localScaling;

public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	explicit cbtCompoundShape(bool enableDynamicAabbTree = true, const int initialChildCapacity = 0);

	virtual ~cbtCompoundShape();

	void addChildShape(const cbtTransform& localTransform, cbtCollisionShape* shape);

	/// Remove all children shapes that contain the specified shape
	virtual void removeChildShape(cbtCollisionShape * shape);

	void removeChildShapeByIndex(int childShapeindex);

	int getNumChildShapes() const
	{
		return int(m_children.size());
	}

	cbtCollisionShape* getChildShape(int index)
	{
		return m_children[index].m_childShape;
	}
	const cbtCollisionShape* getChildShape(int index) const
	{
		return m_children[index].m_childShape;
	}

	cbtTransform& getChildTransform(int index)
	{
		return m_children[index].m_transform;
	}
	const cbtTransform& getChildTransform(int index) const
	{
		return m_children[index].m_transform;
	}

	///set a new transform for a child, and update internal data structures (local aabb and dynamic tree)
	void updateChildTransform(int childIndex, const cbtTransform& newChildTransform, bool shouldRecalculateLocalAabb = true);

	cbtCompoundShapeChild* getChildList()
	{
		return &m_children[0];
	}

	///getAabb's default implementation is brute force, expected derived classes to implement a fast dedicated version
	virtual void getAabb(const cbtTransform& t, cbtVector3& aabbMin, cbtVector3& aabbMax) const;

	/** Re-calculate the local Aabb. Is called at the end of removeChildShapes. 
	Use this yourself if you modify the children or their transforms. */
	virtual void recalculateLocalAabb();

	virtual void setLocalScaling(const cbtVector3& scaling);

	virtual const cbtVector3& getLocalScaling() const
	{
		return m_localScaling;
	}

	virtual void calculateLocalInertia(cbtScalar mass, cbtVector3 & inertia) const;

	virtual void setMargin(cbtScalar margin)
	{
		m_collisionMargin = margin;
	}
	virtual cbtScalar getMargin() const
	{
		return m_collisionMargin;
	}
	virtual const char* getName() const
	{
		return "Compound";
	}

	const cbtDbvt* getDynamicAabbTree() const
	{
		return m_dynamicAabbTree;
	}

	cbtDbvt* getDynamicAabbTree()
	{
		return m_dynamicAabbTree;
	}

	void createAabbTreeFromChildren();

	///computes the exact moment of inertia and the transform from the coordinate system defined by the principal axes of the moment of inertia
	///and the center of mass to the current coordinate system. "masses" points to an array of masses of the children. The resulting transform
	///"principal" has to be applied inversely to all children transforms in order for the local coordinate system of the compound
	///shape to be centered at the center of mass and to coincide with the principal axes. This also necessitates a correction of the world transform
	///of the collision object by the principal transform.
	void calculatePrincipalAxisTransform(const cbtScalar* masses, cbtTransform& principal, cbtVector3& inertia) const;

	int getUpdateRevision() const
	{
		return m_updateRevision;
	}

	virtual int calculateSerializeBufferSize() const;

	///fills the dataBuffer and returns the struct name (and 0 on failure)
	virtual const char* serialize(void* dataBuffer, cbtSerializer* serializer) const;
};

// clang-format off

///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct cbtCompoundShapeChildData
{
	cbtTransformFloatData	m_transform;
	cbtCollisionShapeData	*m_childShape;
	int						m_childShapeType;
	float					m_childMargin;
};

///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct	cbtCompoundShapeData
{
	cbtCollisionShapeData		m_collisionShapeData;

	cbtCompoundShapeChildData	*m_childShapePtr;

	int							m_numChildShapes;

	float	m_collisionMargin;

};

// clang-format on

SIMD_FORCE_INLINE int cbtCompoundShape::calculateSerializeBufferSize() const
{
	return sizeof(cbtCompoundShapeData);
}

#endif  //BT_COMPOUND_SHAPE_H
