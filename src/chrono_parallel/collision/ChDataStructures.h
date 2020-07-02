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
    virtual int Type() const { return 0; }
    virtual real3 A() const { return real3(0); }
    virtual quaternion R() const { return quaternion(1, 0, 0, 0); }
    virtual int Size() const { return 0; }
    virtual const real3* Convex() const { return 0; }
    virtual const real3* Triangles() const { return 0; }
    virtual real Radius() const { return 0; }
    virtual real3 Box() const { return real3(0); }
    virtual real4 Rbox() const { return real4(0); }
    virtual real2 Capsule() const { return real2(0); }
    virtual uvec4 TetIndex() const { return _make_uvec4(0, 0, 0, 0); }
    virtual const real3* TetNodes() const { return 0; }
};

/// Convex contact shape.
class ConvexShape : public ConvexBase {
  public:
    ConvexShape() {}
    ConvexShape(int i, shape_container* d) : index(i), data(d) {}
    virtual ~ConvexShape() {}
    virtual int Type() const override { return data->typ_rigid[index]; }
    virtual real3 A() const override { return data->obj_data_A_global[index]; }
    virtual quaternion R() const override { return data->obj_data_R_global[index]; }
    virtual int Size() const override { return data->length_rigid[index]; }
    virtual const real3* Convex() const override { return &data->convex_rigid[start()]; }
    virtual const real3* Triangles() const override { return &data->triangle_global[start()]; }
    virtual real Radius() const override { return data->sphere_rigid[start()]; }
    virtual real3 Box() const override { return data->box_like_rigid[start()]; }
    virtual real4 Rbox() const override { return data->rbox_like_rigid[start()]; }
    virtual real2 Capsule() const override { return data->capsule_rigid[start()]; }
    int index;
    shape_container* data;  // pointer to convex data;
  private:
    virtual inline int start() const { return data->start_rigid[index]; }
};

/// Sphere contact shape.
class ConvexShapeSphere : public ConvexBase {
  public:
    ConvexShapeSphere(real3 p, real r) : position(p), radius(r) {}
    virtual ~ConvexShapeSphere() {}
    inline int Type() const override { return ChCollisionShape::Type::SPHERE; }
    inline real3 A() const override { return position; }
    inline real Radius() const override { return radius; }
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
    inline int Type() const override { return type; }
    inline real3 A() const override { return position; }
    inline quaternion R() const override { return rotation; }
    inline real Radius() const override { return dimensions.x; }
    inline real3 Box() const override { return dimensions; }
    inline real4 Rbox() const override { return real4(dimensions, radius); }
    inline real2 Capsule() const override { return real2(dimensions.x, dimensions.y); }
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
    inline int Type() const override { return ChCollisionShape::Type::TETRAHEDRON; }
    inline real3 A() const override { return real3(0); }
    uvec4 TetIndex() const override { return indices; }
    const real3* TetNodes() const override { return nodes; }
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
    inline int Type() const override { return ChCollisionShape::Type::TRIANGLE; }
    inline real3 A() const override { return real3(0); }
    const real3* Triangles() const override { return &tri[0]; }
    real3 tri[3];
};

/// @} parallel_colision

} // end namespace collision
} // end namespace chrono
