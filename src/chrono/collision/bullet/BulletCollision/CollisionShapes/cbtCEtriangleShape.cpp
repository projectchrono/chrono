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
#include "cbtCEtriangleShape.h"
#include "BulletCollision/CollisionShapes/cbtCollisionMargin.h"
#include "LinearMath/cbtQuaternion.h"
#include "chrono/collision/bullet/ChCollisionModelBullet.h"
#include <stdio.h>

using namespace chrono;

cbtCEtriangleShape::cbtCEtriangleShape(ChVector<>* mp1,
                                       ChVector<>* mp2,
                                       ChVector<>* mp3,
                                       ChVector<>* me1,
                                       ChVector<>* me2,
                                       ChVector<>* me3,
                                       bool mowns_vertex_1,
                                       bool mowns_vertex_2,
                                       bool mowns_vertex_3,
                                       bool mowns_edge_1,
                                       bool mowns_edge_2,
                                       bool mowns_edge_3,
                                       double msphereswept_rad) {
    p1 = mp1;
    p2 = mp2;
    p3 = mp3;
    e1 = me1;
    e2 = me2;
    e3 = me3;
    owns_vertex_1 = mowns_vertex_1;
    owns_vertex_2 = mowns_vertex_2;
    owns_vertex_3 = mowns_vertex_3;
    owns_edge_1 = mowns_edge_1;
    owns_edge_2 = mowns_edge_2;
    owns_edge_3 = mowns_edge_3;
    sphereswept_rad = msphereswept_rad;

    m_shapeType = CE_TRIANGLE_SHAPE_PROXYTYPE;
}

cbtVector3 cbtCEtriangleShape::localGetSupportingVertexWithoutMargin(const cbtVector3& vec0) const {
    cbtVector3 supVec(cbtScalar(0.), cbtScalar(0.), cbtScalar(0.));
    cbtScalar newDot, maxDot = cbtScalar(-BT_LARGE_FLOAT);
    cbtVector3 vtx;
    vtx = cbtVector3((cbtScalar)this->p1->x(), (cbtScalar)this->p1->y(), (cbtScalar)this->p1->z());
    newDot = vec0.dot(vtx);
    if (newDot > maxDot) {
        maxDot = newDot;
        supVec = vtx;
    }
    vtx = cbtVector3((cbtScalar)this->p2->x(), (cbtScalar)this->p2->y(), (cbtScalar)this->p2->z());
    newDot = vec0.dot(vtx);
    if (newDot > maxDot) {
        maxDot = newDot;
        supVec = vtx;
    }
    vtx = cbtVector3((cbtScalar)this->p3->x(), (cbtScalar)this->p3->y(), (cbtScalar)this->p3->z());
    newDot = vec0.dot(vtx);
    if (newDot > maxDot) {
        maxDot = newDot;
        supVec = vtx;
    }

    return supVec;  //+ vec0.normalized()*this->sphereswept_rad; //***TODO*** add the sphereswept_rad layer (but gives
                    //seldom jittering.. why?)
}

void cbtCEtriangleShape::batchedUnitVectorGetSupportingVertexWithoutMargin(const cbtVector3* vectors,
                                                                           cbtVector3* supportVerticesOut,
                                                                           int numVectors) const {
    printf("NOT SUPPORTED!! \n");
}

void cbtCEtriangleShape::calculateLocalInertia(cbtScalar mass, cbtVector3& inertia) const {
    //***TO DO***
    // as an approximation, take the inertia of an average radius sphere

    cbtTransform ident;
    ident.setIdentity();

    cbtVector3 halfExtents;
    double radius = ChMax((*p2 - *p1).Length(), (*p3 - *p1).Length());
    halfExtents.setValue((cbtScalar)(radius), (cbtScalar)(radius), (cbtScalar)(radius));

    cbtScalar margin = CONVEX_DISTANCE_MARGIN;

    cbtScalar lx = cbtScalar(2.) * (halfExtents[0] + margin);
    cbtScalar ly = cbtScalar(2.) * (halfExtents[1] + margin);
    cbtScalar lz = cbtScalar(2.) * (halfExtents[2] + margin);
    const cbtScalar x2 = lx * lx;
    const cbtScalar y2 = ly * ly;
    const cbtScalar z2 = lz * lz;
    const cbtScalar scaledmass = mass * cbtScalar(.08333333);

    inertia[0] = scaledmass * (y2 + z2);
    inertia[1] = scaledmass * (x2 + z2);
    inertia[2] = scaledmass * (x2 + y2);
}

void cbtCEtriangleShape::getAabb(const cbtTransform& t, cbtVector3& aabbMin, cbtVector3& aabbMax) const {
    cbtVector3 p1_w = t.getOrigin() + t.getBasis() * cbtVector3((cbtScalar)this->p1->x(), (cbtScalar)this->p1->y(),
                                                                (cbtScalar)this->p1->z());
    cbtVector3 p2_w = t.getOrigin() + t.getBasis() * cbtVector3((cbtScalar)this->p2->x(), (cbtScalar)this->p2->y(),
                                                                (cbtScalar)this->p2->z());
    cbtVector3 p3_w = t.getOrigin() + t.getBasis() * cbtVector3((cbtScalar)this->p3->x(), (cbtScalar)this->p3->y(),
                                                                (cbtScalar)this->p3->z());

    ChCollisionModelBullet* triModel = (ChCollisionModelBullet*)this->getUserPointer();

    cbtVector3 venvelope(triModel->GetEnvelope(), triModel->GetEnvelope(), triModel->GetEnvelope());
    cbtVector3 vsphereswept((cbtScalar)this->sphereswept_rad, (cbtScalar)this->sphereswept_rad,
                            (cbtScalar)this->sphereswept_rad);

    aabbMin = cbtVector3((cbtScalar)ChMin(ChMin(p1_w.x(), p2_w.x()), p3_w.x()),
                         (cbtScalar)ChMin(ChMin(p1_w.y(), p2_w.y()), p3_w.y()),
                         (cbtScalar)ChMin(ChMin(p1_w.z(), p2_w.z()), p3_w.z())) -
              venvelope - vsphereswept;

    aabbMax = cbtVector3((cbtScalar)ChMax(ChMax(p1_w.x(), p2_w.x()), p3_w.x()),
                         (cbtScalar)ChMax(ChMax(p1_w.y(), p2_w.y()), p3_w.y()),
                         (cbtScalar)ChMax(ChMax(p1_w.z(), p2_w.z()), p3_w.z())) +
              venvelope + vsphereswept;
}
