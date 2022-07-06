
#ifndef BT_INTERNAL_EDGE_UTILITY_H
#define BT_INTERNAL_EDGE_UTILITY_H

#include "LinearMath/cbtHashMap.h"
#include "LinearMath/cbtVector3.h"

#include "BulletCollision/CollisionShapes/cbtTriangleInfoMap.h"

///The cbtInternalEdgeUtility helps to avoid or reduce artifacts due to wrong collision normals caused by internal edges.
///See also http://code.google.com/p/bullet/issues/detail?id=27

class cbtBvhTriangleMeshShape;
class cbtCollisionObject;
struct cbtCollisionObjectWrapper;
class cbtManifoldPoint;
class cbtIDebugDraw;

enum cbtInternalEdgeAdjustFlags
{
	BT_TRIANGLE_CONVEX_BACKFACE_MODE = 1,
	BT_TRIANGLE_CONCAVE_DOUBLE_SIDED = 2,  //double sided options are experimental, single sided is recommended
	BT_TRIANGLE_CONVEX_DOUBLE_SIDED = 4
};

///Call cbtGenerateInternalEdgeInfo to create triangle info, store in the shape 'userInfo'
void cbtGenerateInternalEdgeInfo(cbtBvhTriangleMeshShape* trimeshShape, cbtTriangleInfoMap* triangleInfoMap);

///Call the cbtFixMeshNormal to adjust the collision normal, using the triangle info map (generated using cbtGenerateInternalEdgeInfo)
///If this info map is missing, or the triangle is not store in this map, nothing will be done
void cbtAdjustInternalEdgeContacts(cbtManifoldPoint& cp, const cbtCollisionObjectWrapper* trimeshColObj0Wrap, const cbtCollisionObjectWrapper* otherColObj1Wrap, int partId0, int index0, int normalAdjustFlags = 0);

///Enable the BT_INTERNAL_EDGE_DEBUG_DRAW define and call cbtSetDebugDrawer, to get visual info to see if the internal edge utility works properly.
///If the utility doesn't work properly, you might have to adjust the threshold values in cbtTriangleInfoMap
//#define BT_INTERNAL_EDGE_DEBUG_DRAW

#ifdef BT_INTERNAL_EDGE_DEBUG_DRAW
void cbtSetDebugDrawer(cbtIDebugDraw* debugDrawer);
#endif  //BT_INTERNAL_EDGE_DEBUG_DRAW

#endif  //BT_INTERNAL_EDGE_UTILITY_H
