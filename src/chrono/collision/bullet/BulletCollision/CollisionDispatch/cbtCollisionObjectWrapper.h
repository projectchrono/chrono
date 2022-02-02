#ifndef BT_COLLISION_OBJECT_WRAPPER_H
#define BT_COLLISION_OBJECT_WRAPPER_H

///cbtCollisionObjectWrapperis an internal data structure.
///Most users can ignore this and use cbtCollisionObject and cbtCollisionShape instead
class cbtCollisionShape;
class cbtCollisionObject;
class cbtTransform;
#include "LinearMath/cbtScalar.h"  // for SIMD_FORCE_INLINE definition

#define BT_DECLARE_STACK_ONLY_OBJECT \
private:                             \
	void* operator new(size_t size); \
	void operator delete(void*);

struct cbtCollisionObjectWrapper;
struct cbtCollisionObjectWrapper
{
	BT_DECLARE_STACK_ONLY_OBJECT

private:
	cbtCollisionObjectWrapper(const cbtCollisionObjectWrapper&);  // not implemented. Not allowed.
	cbtCollisionObjectWrapper* operator=(const cbtCollisionObjectWrapper&);

public:
	const cbtCollisionObjectWrapper* m_parent;
	const cbtCollisionShape* m_shape;
	const cbtCollisionObject* m_collisionObject;
	const cbtTransform& m_worldTransform;
	int m_partId;
	int m_index;

	cbtCollisionObjectWrapper(const cbtCollisionObjectWrapper* parent, const cbtCollisionShape* shape, const cbtCollisionObject* collisionObject, const cbtTransform& worldTransform, int partId, int index)
		: m_parent(parent), m_shape(shape), m_collisionObject(collisionObject), m_worldTransform(worldTransform), m_partId(partId), m_index(index)
	{
	}

	SIMD_FORCE_INLINE const cbtTransform& getWorldTransform() const { return m_worldTransform; }
	SIMD_FORCE_INLINE const cbtCollisionObject* getCollisionObject() const { return m_collisionObject; }
	SIMD_FORCE_INLINE const cbtCollisionShape* getCollisionShape() const { return m_shape; }
};

#endif  //BT_COLLISION_OBJECT_WRAPPER_H
