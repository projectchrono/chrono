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
#include <cmath>
#include <stdio.h>

#include "BulletCollision/CollisionShapes/cbtChTriangleShape.h"
#include "BulletCollision/CollisionShapes/cbtCollisionMargin.h"
#include "LinearMath/cbtQuaternion.h"

#include "chrono/collision/bullet/ChCollisionModelBullet.h"
#include "chrono/collision/bullet/ChCollisionUtilsBullet.h"

using namespace chrono;

cbtChTriangleShape::cbtChTriangleShape(const ChVector3d* mp1,
                                       const ChVector3d* mp2,
                                       const ChVector3d* mp3,
                                       const ChVector3d* me1,
                                       const ChVector3d* me2,
                                       const ChVector3d* me3,
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

    m_shapeType = CH_TRIANGLE_SHAPE_PROXYTYPE;
}

cbtVector3 cbtChTriangleShape::localGetSupportingVertexWithoutMargin(const cbtVector3& vec0) const {
    auto v1 = cbtVector3CH(*p1);
    auto v2 = cbtVector3CH(*p2);
    auto v3 = cbtVector3CH(*p3);

    cbtScalar d1 = vec0.dot(v1);
    cbtScalar d2 = vec0.dot(v2);
    cbtScalar d3 = vec0.dot(v3);

    cbtVector3 sup = (d1 > d2) ? (d1 > d3) ? v1 : v3 : (d2 > d3) ? v2 : v3;

    return sup;
    //// TODO  add the sphereswept_rad layer (but sometimes results in jittering.. why?)
    ////return sup + vec0.normalized() * sphereswept_rad;
}

void cbtChTriangleShape::batchedUnitVectorGetSupportingVertexWithoutMargin(const cbtVector3* vectors, cbtVector3* supportVerticesOut, int numVectors) const {
    printf("NOT SUPPORTED!! \n");
}

void cbtChTriangleShape::calculateLocalInertia(cbtScalar mass, cbtVector3& inertia) const {
    //// TODO
    // as an approximation, take the inertia of an average radius sphere

    cbtTransform ident;
    ident.setIdentity();

    cbtScalar radius = (cbtScalar)std::max((*p2 - *p1).Length(), (*p3 - *p1).Length());
    cbtVector3 halfExtents(radius, radius, radius);

    cbtScalar margin = CONVEX_DISTANCE_MARGIN;

    cbtScalar lx = 2 * (halfExtents[0] + margin);
    cbtScalar ly = 2 * (halfExtents[1] + margin);
    cbtScalar lz = 2 * (halfExtents[2] + margin);
    const cbtScalar x2 = lx * lx;
    const cbtScalar y2 = ly * ly;
    const cbtScalar z2 = lz * lz;
    const cbtScalar scaledmass = mass * cbtScalar(0.08333333);

    inertia[0] = scaledmass * (y2 + z2);
    inertia[1] = scaledmass * (x2 + z2);
    inertia[2] = scaledmass * (x2 + y2);
}

void cbtChTriangleShape::getAabb(const cbtTransform& t, cbtVector3& aabbMin, cbtVector3& aabbMax) const {
    cbtVector3 p1_w = t.getOrigin() + t.getBasis() * cbtVector3CH(*p1);
    cbtVector3 p2_w = t.getOrigin() + t.getBasis() * cbtVector3CH(*p2);
    cbtVector3 p3_w = t.getOrigin() + t.getBasis() * cbtVector3CH(*p3);

    auto tri_data = (ChCollisionModelBullet::ShapeData*)getUserPointer();
    ChCollisionModelBullet* bt_model = tri_data->bt_model;

    cbtVector3 venvelope(bt_model->GetEnvelope(), bt_model->GetEnvelope(), bt_model->GetEnvelope());
    cbtVector3 vsphereswept((cbtScalar)sphereswept_rad, (cbtScalar)sphereswept_rad, (cbtScalar)sphereswept_rad);

    aabbMin = cbtVector3(std::min(std::min(p1_w.x(), p2_w.x()), p3_w.x()), std::min(std::min(p1_w.y(), p2_w.y()), p3_w.y()), std::min(std::min(p1_w.z(), p2_w.z()), p3_w.z())) -
              venvelope - vsphereswept;

    aabbMax = cbtVector3(std::max(std::max(p1_w.x(), p2_w.x()), p3_w.x()), std::max(std::max(p1_w.y(), p2_w.y()), p3_w.y()), std::max(std::max(p1_w.z(), p2_w.z()), p3_w.z())) +
              venvelope + vsphereswept;
}

bool cbtChTriangleShape::rayTest(const cbtTransform& t, const cbtVector3& from, const cbtVector3& to, cbtScalar& fraction, cbtVector3& normal) const {
    // World frame, double precision
    const cbtMatrix3x3& R = t.getBasis();
    const cbtVector3& o = t.getOrigin();
    auto world = [&R, &o](const ChVector3d& p) {
        return ChVector3d(R[0].x() * p.x() + R[0].y() * p.y() + R[0].z() * p.z() + o.x(),  //
                          R[1].x() * p.x() + R[1].y() * p.y() + R[1].z() * p.z() + o.y(),  //
                          R[2].x() * p.x() + R[2].y() * p.y() + R[2].z() * p.z() + o.z());
    };
    const ChVector3d V[3] = {world(*p1), world(*p2), world(*p3)};
    const ChVector3d O(from.x(), from.y(), from.z());
    const ChVector3d D = ChVector3d(to.x(), to.y(), to.z()) - O;
    const double r = getMargin();  // Bullet margin: collision envelope + sphere-swept radius of the mesh

    ChVector3d nt = Vcross(V[1] - V[0], V[2] - V[0]);
    if (nt.Length2() == 0)
        return false;  // degenerate triangle
    nt.Normalize();

    // Line through Q along d crosses the triangle if the three signed edge volumes share a sign. A shared edge
    // gives exactly opposite volumes in its two triangles, so a ray through it is never missed by both. For that,
    // each volume is evaluated with the edge end points in a fixed order: (a x b) and -(b x a) can differ in their
    // last bits when the compiler contracts the products into fused multiply-adds.
    auto before = [](const ChVector3d& a, const ChVector3d& b) {
        return a.x() < b.x() || (a.x() == b.x() && (a.y() < b.y() || (a.y() == b.y() && a.z() < b.z())));
    };
    auto volume = [&before](const ChVector3d& A, const ChVector3d& B, const ChVector3d& Q, const ChVector3d& d) {
        return before(B, A) ? -Vdot(Vcross(B - Q, A - Q), d) : Vdot(Vcross(A - Q, B - Q), d);
    };
    auto crosses = [&V, &volume](const ChVector3d& Q, const ChVector3d& d) {
        const double w0 = volume(V[0], V[1], Q, d);
        const double w1 = volume(V[1], V[2], Q, d);
        const double w2 = volume(V[2], V[0], Q, d);
        return (w0 >= 0 && w1 >= 0 && w2 >= 0) || (w0 <= 0 && w1 <= 0 && w2 <= 0);
    };

    double h = Vdot(O - V[0], nt);
    const ChVector3d n = h < 0 ? -nt : nt;
    h = std::abs(h);

    // No hit if the segment starts inside the solid (as with the convex cast)
    if (h <= r && crosses(O, n))
        return false;
    for (int i = 0; i < 3; i++) {
        const ChVector3d E = V[(i + 1) % 3] - V[i];
        const double u = std::max(0.0, std::min(1.0, Vdot(O - V[i], E) / Vdot(E, E)));
        if ((O - V[i] - E * u).Length2() <= r * r)
            return false;
    }

    double best = 2;  // entry fraction, above 1 means no hit
    ChVector3d best_n;

    // Face offset by r
    const double dn = Vdot(D, n);
    if (dn < 0) {
        const double s = (r - h) / dn;
        if (s >= 0 && s <= 1 && crosses(O - n * r, D)) {
            best = s;
            best_n = n;
        }
    }

    // Edge cylinders and vertex spheres
    if (r > 0) {
        for (int i = 0; i < 3; i++) {
            const ChVector3d& P = V[i];
            const ChVector3d E = V[(i + 1) % 3] - P;
            const double ee = Vdot(E, E);

            // |(O + s D - P) x E|^2 = r^2 |E|^2
            const ChVector3d c0 = Vcross(O - P, E);
            const ChVector3d c1 = Vcross(D, E);
            const double a = Vdot(c1, c1);
            const double b = Vdot(c0, c1);
            const double disc = b * b - a * (Vdot(c0, c0) - r * r * ee);
            if (a > 0 && disc >= 0) {
                const double s = (-b - std::sqrt(disc)) / a;
                const ChVector3d X = O + D * s;
                const double u = Vdot(X - P, E) / ee;
                if (s >= 0 && s < best && u >= 0 && u <= 1) {
                    best = s;
                    best_n = X - P - E * u;
                }
            }

            // |O + s D - P|^2 = r^2
            const ChVector3d w = O - P;
            const double a2 = Vdot(D, D);
            const double b2 = Vdot(w, D);
            const double disc2 = b2 * b2 - a2 * (Vdot(w, w) - r * r);
            if (a2 > 0 && disc2 >= 0) {
                const double s = (-b2 - std::sqrt(disc2)) / a2;
                if (s >= 0 && s < best) {
                    best = s;
                    best_n = O + D * s - P;
                }
            }
        }
    }

    if (best > 1)
        return false;

    best_n.Normalize();
    fraction = (cbtScalar)best;
    normal.setValue((cbtScalar)best_n.x(), (cbtScalar)best_n.y(), (cbtScalar)best_n.z());
    return true;
}
