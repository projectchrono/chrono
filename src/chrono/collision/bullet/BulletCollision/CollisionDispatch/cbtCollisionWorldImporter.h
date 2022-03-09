/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2014 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_COLLISION_WORLD_IMPORTER_H
#define BT_COLLISION_WORLD_IMPORTER_H

#include "LinearMath/cbtTransform.h"
#include "LinearMath/cbtVector3.h"
#include "LinearMath/cbtAlignedObjectArray.h"
#include "LinearMath/cbtHashMap.h"

class cbtCollisionShape;
class cbtCollisionObject;
struct cbtBulletSerializedArrays;

struct ConstraintInput;
class cbtCollisionWorld;
struct cbtCollisionShapeData;
class cbtTriangleIndexVertexArray;
class cbtStridingMeshInterface;
struct cbtStridingMeshInterfaceData;
class cbtGImpactMeshShape;
class cbtOptimizedBvh;
struct cbtTriangleInfoMap;
class cbtBvhTriangleMeshShape;
class cbtPoint2PointConstraint;
class cbtHingeConstraint;
class cbtConeTwistConstraint;
class cbtGeneric6DofConstraint;
class cbtGeneric6DofSpringConstraint;
class cbtSliderConstraint;
class cbtGearConstraint;
struct cbtContactSolverInfo;

class cbtCollisionWorldImporter
{
protected:
	cbtCollisionWorld* m_collisionWorld;

	int m_verboseMode;

	cbtAlignedObjectArray<cbtCollisionShape*> m_allocatedCollisionShapes;
	cbtAlignedObjectArray<cbtCollisionObject*> m_allocatedRigidBodies;

	cbtAlignedObjectArray<cbtOptimizedBvh*> m_allocatedBvhs;
	cbtAlignedObjectArray<cbtTriangleInfoMap*> m_allocatedTriangleInfoMaps;
	cbtAlignedObjectArray<cbtTriangleIndexVertexArray*> m_allocatedTriangleIndexArrays;
	cbtAlignedObjectArray<cbtStridingMeshInterfaceData*> m_allocatedcbtStridingMeshInterfaceDatas;
	cbtAlignedObjectArray<cbtCollisionObject*> m_allocatedCollisionObjects;

	cbtAlignedObjectArray<char*> m_allocatedNames;

	cbtAlignedObjectArray<int*> m_indexArrays;
	cbtAlignedObjectArray<short int*> m_shortIndexArrays;
	cbtAlignedObjectArray<unsigned char*> m_charIndexArrays;

	cbtAlignedObjectArray<cbtVector3FloatData*> m_floatVertexArrays;
	cbtAlignedObjectArray<cbtVector3DoubleData*> m_doubleVertexArrays;

	cbtHashMap<cbtHashPtr, cbtOptimizedBvh*> m_bvhMap;
	cbtHashMap<cbtHashPtr, cbtTriangleInfoMap*> m_timMap;

	cbtHashMap<cbtHashString, cbtCollisionShape*> m_nameShapeMap;
	cbtHashMap<cbtHashString, cbtCollisionObject*> m_nameColObjMap;

	cbtHashMap<cbtHashPtr, const char*> m_objectNameMap;

	cbtHashMap<cbtHashPtr, cbtCollisionShape*> m_shapeMap;
	cbtHashMap<cbtHashPtr, cbtCollisionObject*> m_bodyMap;

	//methods

	char* duplicateName(const char* name);

	cbtCollisionShape* convertCollisionShape(cbtCollisionShapeData* shapeData);

public:
	cbtCollisionWorldImporter(cbtCollisionWorld* world);

	virtual ~cbtCollisionWorldImporter();

	bool convertAllObjects(cbtBulletSerializedArrays* arrays);

	///delete all memory collision shapes, rigid bodies, constraints etc. allocated during the load.
	///make sure you don't use the dynamics world containing objects after you call this method
	virtual void deleteAllData();

	void setVerboseMode(int verboseMode)
	{
		m_verboseMode = verboseMode;
	}

	int getVerboseMode() const
	{
		return m_verboseMode;
	}

	// query for data
	int getNumCollisionShapes() const;
	cbtCollisionShape* getCollisionShapeByIndex(int index);
	int getNumRigidBodies() const;
	cbtCollisionObject* getRigidBodyByIndex(int index) const;

	int getNumBvhs() const;
	cbtOptimizedBvh* getBvhByIndex(int index) const;
	int getNumTriangleInfoMaps() const;
	cbtTriangleInfoMap* getTriangleInfoMapByIndex(int index) const;

	// queris involving named objects
	cbtCollisionShape* getCollisionShapeByName(const char* name);
	cbtCollisionObject* getCollisionObjectByName(const char* name);

	const char* getNameForPointer(const void* ptr) const;

	///those virtuals are called by load and can be overridden by the user

	//bodies

	virtual cbtCollisionObject* createCollisionObject(const cbtTransform& startTransform, cbtCollisionShape* shape, const char* bodyName);

	///shapes

	virtual cbtCollisionShape* createPlaneShape(const cbtVector3& planeNormal, cbtScalar planeConstant);
	virtual cbtCollisionShape* createBoxShape(const cbtVector3& halfExtents);
	virtual cbtCollisionShape* createSphereShape(cbtScalar radius);
	virtual cbtCollisionShape* createCapsuleShapeX(cbtScalar radius, cbtScalar height);
	virtual cbtCollisionShape* createCapsuleShapeY(cbtScalar radius, cbtScalar height);
	virtual cbtCollisionShape* createCapsuleShapeZ(cbtScalar radius, cbtScalar height);

	virtual cbtCollisionShape* createCylinderShapeX(cbtScalar radius, cbtScalar height);
	virtual cbtCollisionShape* createCylinderShapeY(cbtScalar radius, cbtScalar height);
	virtual cbtCollisionShape* createCylinderShapeZ(cbtScalar radius, cbtScalar height);
    virtual cbtCollisionShape* createCylindricalShellShape(cbtScalar radius, cbtScalar height);  /* ***CHRONO*** */
    virtual cbtCollisionShape* createConeShapeX(cbtScalar radius, cbtScalar height);
	virtual cbtCollisionShape* createConeShapeY(cbtScalar radius, cbtScalar height);
	virtual cbtCollisionShape* createConeShapeZ(cbtScalar radius, cbtScalar height);
	virtual class cbtTriangleIndexVertexArray* createTriangleMeshContainer();
	virtual cbtBvhTriangleMeshShape* createBvhTriangleMeshShape(cbtStridingMeshInterface* trimesh, cbtOptimizedBvh* bvh);
	virtual cbtCollisionShape* createConvexTriangleMeshShape(cbtStridingMeshInterface* trimesh);
#ifdef SUPPORT_GIMPACT_SHAPE_IMPORT
	virtual cbtGImpactMeshShape* createGimpactShape(cbtStridingMeshInterface* trimesh);
#endif  //SUPPORT_GIMPACT_SHAPE_IMPORT
	virtual cbtStridingMeshInterfaceData* createStridingMeshInterfaceData(cbtStridingMeshInterfaceData* interfaceData);

	virtual class cbtConvexHullShape* createConvexHullShape();
	virtual class cbtCompoundShape* createCompoundShape();
	virtual class cbtScaledBvhTriangleMeshShape* createScaledTrangleMeshShape(cbtBvhTriangleMeshShape* meshShape, const cbtVector3& localScalingcbtBvhTriangleMeshShape);

	virtual class cbtMultiSphereShape* createMultiSphereShape(const cbtVector3* positions, const cbtScalar* radi, int numSpheres);

	virtual cbtTriangleIndexVertexArray* createMeshInterface(cbtStridingMeshInterfaceData& meshData);

	///acceleration and connectivity structures
	virtual cbtOptimizedBvh* createOptimizedBvh();
	virtual cbtTriangleInfoMap* createTriangleInfoMap();
};

#endif  //BT_WORLD_IMPORTER_H
