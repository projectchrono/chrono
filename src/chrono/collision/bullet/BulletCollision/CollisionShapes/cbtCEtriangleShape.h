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

#ifndef BT_CE_TRIANGLE_SHAPE_H
#define BT_CE_TRIANGLE_SHAPE_H

#include "cbtConvexInternalShape.h"
#include "BulletCollision/BroadphaseCollision/cbtBroadphaseProxy.h"  // for the types
#include "LinearMath/cbtVector3.h"
#include "chrono/core/ChVector.h"

/// cbtCEtriangleShape represents a triangle that is part of a collision mesh.
/// This because the default Bullet or GImpact triangle mesh system is not flexible enough to
/// handle FEM problems where each triangle may have its collision model, and because
/// of other limitations related to robustness etc.
/// The idea is to use 'representative triangles' with additional info on neighbours as in
/// "Fast Collision Detection for Deformable Models using Representative-Triangles"
/// S.Rasmus Tamstorf, D.Manocha1

class cbtCEtriangleShape : public cbtConvexInternalShape {
  private:
    chrono::ChVector<>* p1;
    chrono::ChVector<>* p2;
    chrono::ChVector<>* p3;
    chrono::ChVector<>* e1;
    chrono::ChVector<>* e2;
    chrono::ChVector<>* e3;
    bool owns_vertex_1;
    bool owns_vertex_2;
    bool owns_vertex_3;
    bool owns_edge_1;
    bool owns_edge_2;
    bool owns_edge_3;
    double sphereswept_rad;

  public:
    cbtCEtriangleShape(chrono::ChVector<>* mp1,
                       chrono::ChVector<>* mp2,
                       chrono::ChVector<>* mp3,
                       chrono::ChVector<>* me1,
                       chrono::ChVector<>* me2,
                       chrono::ChVector<>* me3,
                       bool mowns_vertex_1,
                       bool mowns_vertex_2,
                       bool mowns_vertex_3,
                       bool mowns_edge_1,
                       bool mowns_edge_2,
                       bool mowns_edge_3,
                       double msphereswept_rad = 0);

    /// CollisionShape Interface
    virtual void calculateLocalInertia(cbtScalar mass, cbtVector3& inertia) const;

    /// cbtConvexShape Interface
    virtual cbtVector3 localGetSupportingVertexWithoutMargin(const cbtVector3& vec) const;

    virtual void batchedUnitVectorGetSupportingVertexWithoutMargin(const cbtVector3* vectors,
                                                                   cbtVector3* supportVerticesOut,
                                                                   int numVectors) const;

    virtual const char* getName() const { return "CEtriangleShape"; }

    virtual void getAabb(const cbtTransform& t, cbtVector3& aabbMin, cbtVector3& aabbMax) const;

    /// access vertex points  of triangle
    chrono::ChVector<>* get_p1() const { return p1; }
    chrono::ChVector<>* get_p2() const { return p2; }
    chrono::ChVector<>* get_p3() const { return p3; }

    /// access points of neighbouring triangles at edges, if any (if no neighbour, is null ptr)
    chrono::ChVector<>* get_e1() const { return e1; }
    chrono::ChVector<>* get_e2() const { return e2; }
    chrono::ChVector<>* get_e3() const { return e3; }

    /// tell if the representative triangle owns the vertex
    bool owns_v1() const { return owns_vertex_1; }
    bool owns_v2() const { return owns_vertex_2; }
    bool owns_v3() const { return owns_vertex_3; }

    /// tell if the representative triangle owns the edge
    bool owns_e1() const { return owns_edge_1; }
    bool owns_e2() const { return owns_edge_2; }
    bool owns_e3() const { return owns_edge_3; }

    /// thickness, for sphere-swept triangles.
    double sphereswept_r() const { return sphereswept_rad; }
};

#endif
