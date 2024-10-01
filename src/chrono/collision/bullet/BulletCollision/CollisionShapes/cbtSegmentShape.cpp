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

// #define NOMINMAX
#include <algorithm>
#include <stdio.h>

#include "BulletCollision/CollisionShapes/cbtSegmentShape.h"
#include "BulletCollision/CollisionShapes/cbtCollisionMargin.h"
#include "LinearMath/cbtQuaternion.h"

#include "chrono/collision/bullet/ChCollisionModelBullet.h"
#include "chrono/collision/bullet/ChCollisionUtilsBullet.h"

using namespace chrono;

cbtSegmentShape::cbtSegmentShape(const ChVector3d* p1,
                                 const ChVector3d* p2,
                                 bool owns_vertex_1,
                                 bool owns_vertex_2,
                                 double radius)
    : p1(p1), p2(p2), owns_vertex_1(owns_vertex_1), owns_vertex_2(owns_vertex_2), sradius(radius) {
    m_shapeType = SEGMENT_SHAPE_PROXYTYPE;
}

cbtVector3 cbtSegmentShape::localGetSupportingVertexWithoutMargin(const cbtVector3& vec0) const {
    cbtVector3 vec = vec0;
    cbtScalar lenSqr = vec.length2();
    if (lenSqr < cbtScalar(0.0001)) {
        vec.setValue(1, 0, 0);
    } else {
        cbtScalar rlen = cbtScalar(1.) / cbtSqrt(lenSqr);
        vec *= rlen;
    }

    auto v1 = cbtVector3CH(*p1);
    auto v2 = cbtVector3CH(*p2);

    cbtScalar d1 = vec.dot(v1);
    cbtScalar d2 = vec.dot(v2);

    cbtVector3 sup = (d1 > d2) ? v1 : v2;

    return sup;
}

void cbtSegmentShape::batchedUnitVectorGetSupportingVertexWithoutMargin(const cbtVector3* vectors,
                                                                        cbtVector3* supportVerticesOut,
                                                                        int numVectors) const {
    printf("NOT SUPPORTED!! \n");
}

void cbtSegmentShape::calculateLocalInertia(cbtScalar mass, cbtVector3& inertia) const {
    //// TODO
    // as an approximation, take the inertia of an average radius sphere
    cbtScalar r = (cbtScalar)((*p2 - *p1).Length() / 2);
    cbtVector3 h(r, r, r);

    cbtScalar lx = cbtScalar(2.0) * (h[0]);
    cbtScalar ly = cbtScalar(2.0) * (h[1]);
    cbtScalar lz = cbtScalar(2.0) * (h[2]);
    const cbtScalar x2 = lx * lx;
    const cbtScalar y2 = ly * ly;
    const cbtScalar z2 = lz * lz;
    const cbtScalar scaledmass = mass * cbtScalar(0.08333333);

    inertia[0] = scaledmass * (y2 + z2);
    inertia[1] = scaledmass * (x2 + z2);
    inertia[2] = scaledmass * (x2 + y2);
}

void cbtSegmentShape::getAabb(const cbtTransform& t, cbtVector3& aabbMin, cbtVector3& aabbMax) const {
    cbtVector3 p1_w = t.getOrigin() + t.getBasis() * cbtVector3CH(*p1);
    cbtVector3 p2_w = t.getOrigin() + t.getBasis() * cbtVector3CH(*p2);

    ChCollisionModelBullet* triModel = (ChCollisionModelBullet*)getUserPointer();

    cbtVector3 venvelope(triModel->GetEnvelope(), triModel->GetEnvelope(), triModel->GetEnvelope());
    cbtVector3 vsphereswept((cbtScalar)sradius, (cbtScalar)sradius, (cbtScalar)sradius);

    aabbMin = cbtVector3(std::min(p1_w.x(), p2_w.x()), std::min(p1_w.y(), p2_w.y()), std::min(p1_w.z(), p2_w.z())) -
              venvelope - vsphereswept;
    aabbMax = cbtVector3(std::max(p1_w.x(), p2_w.x()), std::max(p1_w.y(), p2_w.y()), std::max(p1_w.z(), p2_w.z())) +
              venvelope + vsphereswept;
}
