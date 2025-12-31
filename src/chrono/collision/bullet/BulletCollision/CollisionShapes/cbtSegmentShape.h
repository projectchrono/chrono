/*
***CHRONO***
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If
you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not
required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original
software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_SEGMENT_SHAPE_H
#define BT_SEGMENT_SHAPE_H

#include "BulletCollision/CollisionShapes/cbtConvexInternalShape.h"
#include "BulletCollision/BroadphaseCollision/cbtBroadphaseProxy.h"
#include "LinearMath/cbtVector3.h"

#include "chrono/core/ChVector3.h"

/// Class for segment-like elements.
/// These are equivalent to capsules, but defined via 2 points expressed in the global frame.
class cbtSegmentShape : public cbtConvexInternalShape {
  public:
    cbtSegmentShape(const chrono::ChVector3d* p1,
                    const chrono::ChVector3d* p2,
                    bool owns_vertex_1,
                    bool owns_vertex_2,
                    double radius);

    // cbtCollisionShape interface
    virtual const char* getName() const { return "SegmentShape"; }
    virtual void calculateLocalInertia(cbtScalar mass, cbtVector3& inertia) const;
    virtual void getAabb(const cbtTransform& t, cbtVector3& aabbMin, cbtVector3& aabbMax) const;

    // cbtConvexShape interface
    virtual cbtVector3 localGetSupportingVertexWithoutMargin(const cbtVector3& vec) const;
    virtual void batchedUnitVectorGetSupportingVertexWithoutMargin(const cbtVector3* vectors,
                                                                   cbtVector3* supportVerticesOut,
                                                                   int numVectors) const;
    // access segment vertex points
    const chrono::ChVector3d* get_p1() const { return p1; }
    const chrono::ChVector3d* get_p2() const { return p2; }

    // check if this segment owns the vertices
    bool owns_v1() const { return owns_vertex_1; }
    bool owns_v2() const { return owns_vertex_2; }

    // thickness, for sphere-swept triangles.
    double radius() const { return sradius; }

  private:
    const chrono::ChVector3d* p1;
    const chrono::ChVector3d* p2;
    bool owns_vertex_1;
    bool owns_vertex_2;
    double sradius;
};

#endif
