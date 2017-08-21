// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2016 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Hammad Mazhar
// =============================================================================
//
// Description: Data structures used by the narrowphase
//
// =============================================================================

#pragma once

#include "chrono_parallel/ChDataManager.h"

namespace chrono {
namespace collision {

/// @addtogroup parallel_collision
/// @{

/// Base class for convex contact shapes.
class ConvexBase {
  public:
    ConvexBase() {}
    virtual ~ConvexBase() {}
    virtual const int Type() const { return 0; }
    virtual const real3 A() const { return real3(0); }
    virtual const quaternion R() const { return quaternion(1, 0, 0, 0); }
    virtual const int Size() const { return 0; }
    virtual const real3* Convex() const { return 0; }
    virtual const real3* Triangles() const { return 0; }
    virtual const real Radius() const { return 0; }
    virtual const real3 Box() const { return real3(0); }
    virtual const real4 Rbox() const { return real4(0); }
    virtual const real2 Capsule() const { return real2(0); }
    virtual const uvec4 TetIndex() const { return _make_uvec4(0, 0, 0, 0); }
    virtual const real3* TetNodes() const { return 0; }
};

/// Convex contact shape.
class ConvexShape : public ConvexBase {
  public:
    ConvexShape() {}
    ConvexShape(int i, shape_container* d) : index(i), data(d) {}
    virtual ~ConvexShape() {}
    virtual const int Type() const { return data->typ_rigid[index]; }
    virtual const real3 A() const { return data->obj_data_A_global[index]; }
    virtual const quaternion R() const { return data->obj_data_R_global[index]; }
    virtual const int Size() const { return data->length_rigid[index]; }
    virtual const real3* Convex() const { return &data->convex_rigid[start()]; }
    virtual const real3* Triangles() const { return &data->triangle_global[start()]; }
    virtual const real Radius() const { return data->sphere_rigid[start()]; }
    virtual const real3 Box() const { return data->box_like_rigid[start()]; }
    virtual const real4 Rbox() const { return data->rbox_like_rigid[start()]; }
    virtual const real2 Capsule() const { return data->capsule_rigid[start()]; }
    int index;
    shape_container* data;  // pointer to convex data;
  private:
    virtual const inline int start() const { return data->start_rigid[index]; }
};

/// Sphere contact shape.
class ConvexShapeSphere : public ConvexBase {
  public:
    ConvexShapeSphere(real3 p, real r) : position(p), radius(r) {}
    virtual ~ConvexShapeSphere() {}
    const inline int Type() const { return SPHERE; }
    const inline real3 A() const { return position; }
    const inline real Radius() const { return radius; }
    real3 position;
    real radius;
};

/// Custom contact shape.
class ConvexShapeCustom : public ConvexBase {
  public:
    ConvexShapeCustom() {}
    ConvexShapeCustom(const int t, const real3& p, const quaternion& rot, const real3& d, const real r = 0)
        : type(t), position(p), rotation(rot), dimensions(d), radius(r) {}
    virtual ~ConvexShapeCustom() {}
    const inline int Type() const { return type; }
    const inline real3 A() const { return position; }
    const inline quaternion R() const { return rotation; }
    const inline real Radius() const { return dimensions.x; }
    const inline real3 Box() const { return dimensions; }
    const inline real4 Rbox() const { return real4(dimensions, radius); }
    const inline real2 Capsule() const { return real2(dimensions.x, dimensions.y); }
    int type;
    real3 position;
    quaternion rotation;
    real3 dimensions;
    real radius;
};

/// Tetrahedron contact shape.
class ConvexShapeTetrahedron : public ConvexBase {
  public:
    ConvexShapeTetrahedron(uvec4 i, real3* n) : indices(i), nodes(n) {}
    virtual ~ConvexShapeTetrahedron() {}
    const inline int Type() const { return TETRAHEDRON; }
    const inline real3 A() const { return real3(0); }
    const uvec4 TetIndex() const { return indices; }
    const real3* TetNodes() const { return nodes; }
    uvec4 indices;
    real3* nodes;
};

/// Triangle contact shape.
class ConvexShapeTriangle : public ConvexBase {
  public:
    ConvexShapeTriangle(real3& t1, real3& t2, real3 t3) {
        tri[0] = t1;
        tri[1] = t2;
        tri[2] = t3;
    }
    virtual ~ConvexShapeTriangle() {}
    const inline int Type() const { return TRIANGLEMESH; }
    const inline real3 A() const { return real3(0); }
    virtual const real3* Triangles() const { return &tri[0]; }
    real3 tri[3];
};

/// @} parallel_colision

} // end namespace collision
} // end namespace chrono
