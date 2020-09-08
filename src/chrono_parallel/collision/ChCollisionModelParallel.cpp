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
// Description: class for a parallel collision model
//
// =============================================================================

#include "chrono_parallel/collision/ChCollisionModelParallel.h"

#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChBodyAuxRef.h"
#include "chrono/physics/ChSystem.h"

namespace chrono {
namespace collision {

ChCollisionModelParallel::ChCollisionModelParallel() : aabb_min(C_LARGE_REAL), aabb_max(-C_LARGE_REAL) {
    model_safe_margin = 0;
}

ChCollisionModelParallel::~ChCollisionModelParallel() {
    m_shapes.clear();
}

int ChCollisionModelParallel::ClearModel() {
    if (GetPhysicsItem()->GetSystem() && GetPhysicsItem()->GetCollide()) {
        GetPhysicsItem()->GetSystem()->GetCollisionSystem()->Remove(this);
    }

    local_convex_data.clear();
    m_shapes.clear();
    aabb_min = real3(C_LARGE_REAL);
    aabb_max = real3(-C_LARGE_REAL);
    family_group = 1;
    family_mask = 0x7FFF;

    return 1;
}

int ChCollisionModelParallel::BuildModel() {
    if (GetPhysicsItem()->GetSystem() && GetPhysicsItem()->GetCollide()) {
        GetPhysicsItem()->GetSystem()->GetCollisionSystem()->Add(this);
    }

    return 1;
}

// TransformToCOG
// This utility function converts a given position and orientation, specified
// with respect to a body's reference frame, into a frame defined with respect
// to the body's centroidal frame.  Note that by default, a body's reference
// frame is the centroidal frame. This is not true for a ChBodyAuxRef.
void TransformToCOG(ChBody* body, const ChVector<>& pos, const ChMatrix33<>& rot, ChFrame<>& frame) {
    frame = ChFrame<>(pos, rot);
    if (ChBodyAuxRef* body_ar = dynamic_cast<ChBodyAuxRef*>(body)) {
        frame = frame >> body_ar->GetFrame_REF_to_COG();
    }
}

bool ChCollisionModelParallel::AddSphere(std::shared_ptr<ChMaterialSurface> material,
                                         double radius,
                                         const ChVector<>& pos) {
    ChFrame<> frame;
    TransformToCOG(GetBody(), pos, ChMatrix33<>(1), frame);
    const ChVector<>& position = frame.GetPos();

    auto shape = new ChCollisionShapeParallel(ChCollisionShape::Type::SPHERE, material);
    shape->A = real3(position.x(), position.y(), position.z());
    shape->B = real3(radius, 0, 0);
    shape->C = real3(0, 0, 0);
    shape->R = quaternion(1, 0, 0, 0);
    m_shapes.push_back(std::shared_ptr<ChCollisionShape>(shape));

    return true;
}

bool ChCollisionModelParallel::AddEllipsoid(std::shared_ptr<ChMaterialSurface> material,
                                            double rx,
                                            double ry,
                                            double rz,
                                            const ChVector<>& pos,
                                            const ChMatrix33<>& rot) {
    ChFrame<> frame;
    TransformToCOG(GetBody(), pos, rot, frame);
    const ChVector<>& position = frame.GetPos();
    const ChQuaternion<>& rotation = frame.GetRot();

    auto shape = new ChCollisionShapeParallel(ChCollisionShape::Type::ELLIPSOID, material);
    shape->A = real3(position.x(), position.y(), position.z());
    shape->B = real3(rx, ry, rz);
    shape->C = real3(0, 0, 0);
    shape->R = quaternion(rotation.e0(), rotation.e1(), rotation.e2(), rotation.e3());
    m_shapes.push_back(std::shared_ptr<ChCollisionShape>(shape));

    return true;
}

bool ChCollisionModelParallel::AddBox(std::shared_ptr<ChMaterialSurface> material,
                                      double rx,
                                      double ry,
                                      double rz,
                                      const ChVector<>& pos,
                                      const ChMatrix33<>& rot) {
    ChFrame<> frame;
    TransformToCOG(GetBody(), pos, rot, frame);
    const ChVector<>& position = frame.GetPos();
    const ChQuaternion<>& rotation = frame.GetRot();

    auto shape = new ChCollisionShapeParallel(ChCollisionShape::Type::BOX, material);
    shape->A = real3(position.x(), position.y(), position.z());
    shape->B = real3(rx, ry, rz);
    shape->C = real3(0, 0, 0);
    shape->R = quaternion(rotation.e0(), rotation.e1(), rotation.e2(), rotation.e3());
    m_shapes.push_back(std::shared_ptr<ChCollisionShape>(shape));

    return true;
}

bool ChCollisionModelParallel::AddRoundedBox(std::shared_ptr<ChMaterialSurface> material,
                                             double rx,
                                             double ry,
                                             double rz,
                                             double sphere_r,
                                             const ChVector<>& pos,
                                             const ChMatrix33<>& rot) {
    ChFrame<> frame;
    TransformToCOG(GetBody(), pos, rot, frame);
    const ChVector<>& position = frame.GetPos();
    const ChQuaternion<>& rotation = frame.GetRot();

    auto shape = new ChCollisionShapeParallel(ChCollisionShape::Type::ROUNDEDBOX, material);
    shape->A = real3(position.x(), position.y(), position.z());
    shape->B = real3(rx, ry, rz);
    shape->C = real3(sphere_r, 0, 0);
    shape->R = quaternion(rotation.e0(), rotation.e1(), rotation.e2(), rotation.e3());
    m_shapes.push_back(std::shared_ptr<ChCollisionShape>(shape));

    return true;
}

bool ChCollisionModelParallel::AddTriangle(std::shared_ptr<ChMaterialSurface> material,
                                           ChVector<> A,
                                           ChVector<> B,
                                           ChVector<> C,
                                           const ChVector<>& pos,
                                           const ChMatrix33<>& rot) {
    ChFrame<> frame;
    TransformToCOG(GetBody(), pos, rot, frame);
    const ChVector<>& position = frame.GetPos();
    const ChQuaternion<>& rotation = frame.GetRot();

    auto shape = new ChCollisionShapeParallel(ChCollisionShape::Type::TRIANGLE, material);
    shape->A = real3(A.x() + position.x(), A.y() + position.y(), A.z() + position.z());
    shape->B = real3(B.x() + position.x(), B.y() + position.y(), B.z() + position.z());
    shape->C = real3(C.x() + position.x(), C.y() + position.y(), C.z() + position.z());
    shape->R = quaternion(rotation.e0(), rotation.e1(), rotation.e2(), rotation.e3());
    m_shapes.push_back(std::shared_ptr<ChCollisionShape>(shape));

    return true;
}

bool ChCollisionModelParallel::AddCylinder(std::shared_ptr<ChMaterialSurface> material,
                                           double rx,
                                           double rz,
                                           double hy,
                                           const ChVector<>& pos,
                                           const ChMatrix33<>& rot) {
    ChFrame<> frame;
    TransformToCOG(GetBody(), pos, rot, frame);
    const ChVector<>& position = frame.GetPos();
    const ChQuaternion<>& rotation = frame.GetRot();

    auto shape = new ChCollisionShapeParallel(ChCollisionShape::Type::CYLINDER, material);
    shape->A = real3(position.x(), position.y(), position.z());
    shape->B = real3(rx, hy, rz);
    shape->C = real3(0, 0, 0);
    shape->R = quaternion(rotation.e0(), rotation.e1(), rotation.e2(), rotation.e3());
    m_shapes.push_back(std::shared_ptr<ChCollisionShape>(shape));

    return true;
}

bool ChCollisionModelParallel::AddRoundedCylinder(std::shared_ptr<ChMaterialSurface> material,
                                                  double rx,
                                                  double rz,
                                                  double hy,
                                                  double sphere_r,
                                                  const ChVector<>& pos,
                                                  const ChMatrix33<>& rot) {
    ChFrame<> frame;
    TransformToCOG(GetBody(), pos, rot, frame);
    const ChVector<>& position = frame.GetPos();
    const ChQuaternion<>& rotation = frame.GetRot();

    auto shape = new ChCollisionShapeParallel(ChCollisionShape::Type::ROUNDEDCYL, material);
    shape->A = real3(position.x(), position.y(), position.z());
    shape->B = real3(rx, hy, rz);
    shape->C = real3(sphere_r, 0, 0);
    shape->R = quaternion(rotation.e0(), rotation.e1(), rotation.e2(), rotation.e3());
    m_shapes.push_back(std::shared_ptr<ChCollisionShape>(shape));

    return true;
}

bool ChCollisionModelParallel::AddCone(std::shared_ptr<ChMaterialSurface> material,
                                       double rx,
                                       double rz,
                                       double hy,
                                       const ChVector<>& pos,
                                       const ChMatrix33<>& rot) {
    ChFrame<> frame;
    TransformToCOG(GetBody(), pos, rot, frame);
    const ChVector<>& position = frame.GetPos();
    const ChQuaternion<>& rotation = frame.GetRot();

    auto shape = new ChCollisionShapeParallel(ChCollisionShape::Type::CONE, material);
    shape->A = real3(position.x(), position.y(), position.z());
    shape->B = real3(rx, hy, rz);
    shape->C = real3(0, 0, 0);
    shape->R = quaternion(rotation.e0(), rotation.e1(), rotation.e2(), rotation.e3());
    m_shapes.push_back(std::shared_ptr<ChCollisionShape>(shape));

    return true;
}

bool ChCollisionModelParallel::AddRoundedCone(std::shared_ptr<ChMaterialSurface> material,
                                              double rx,
                                              double rz,
                                              double hy,
                                              double sphere_r,
                                              const ChVector<>& pos,
                                              const ChMatrix33<>& rot) {
    ChFrame<> frame;
    TransformToCOG(GetBody(), pos, rot, frame);
    const ChVector<>& position = frame.GetPos();
    const ChQuaternion<>& rotation = frame.GetRot();

    auto shape = new ChCollisionShapeParallel(ChCollisionShape::Type::ROUNDEDCONE, material);
    shape->A = real3(position.x(), position.y(), position.z());
    shape->B = real3(rx, hy, rz);
    shape->C = real3(sphere_r, 0, 0);
    shape->R = quaternion(rotation.e0(), rotation.e1(), rotation.e2(), rotation.e3());
    m_shapes.push_back(std::shared_ptr<ChCollisionShape>(shape));

    return true;
}

bool ChCollisionModelParallel::AddCapsule(std::shared_ptr<ChMaterialSurface> material,
                                          double radius,
                                          double hlen,
                                          const ChVector<>& pos,
                                          const ChMatrix33<>& rot) {
    ChFrame<> frame;
    TransformToCOG(GetBody(), pos, rot, frame);
    const ChVector<>& position = frame.GetPos();
    const ChQuaternion<>& rotation = frame.GetRot();

    auto shape = new ChCollisionShapeParallel(ChCollisionShape::Type::CAPSULE, material);
    shape->A = real3(position.x(), position.y(), position.z());
    shape->B = real3(radius, hlen, radius);
    shape->C = real3(0, 0, 0);
    shape->R = quaternion(rotation.e0(), rotation.e1(), rotation.e2(), rotation.e3());
    m_shapes.push_back(std::shared_ptr<ChCollisionShape>(shape));

    return true;
}

bool ChCollisionModelParallel::AddConvexHull(std::shared_ptr<ChMaterialSurface> material,
                                             const std::vector<ChVector<double> >& pointlist,
                                             const ChVector<>& pos,
                                             const ChMatrix33<>& rot) {
    ChFrame<> frame;
    TransformToCOG(GetBody(), pos, rot, frame);
    const ChVector<>& position = frame.GetPos();
    const ChQuaternion<>& rotation = frame.GetRot();

    auto shape = new ChCollisionShapeParallel(ChCollisionShape::Type::CONVEX, material);
    shape->A = real3(position.x(), position.y(), position.z());
    shape->B = real3((chrono::real)pointlist.size(), (chrono::real)local_convex_data.size(), 0);
    shape->C = real3(0, 0, 0);
    shape->R = quaternion(rotation.e0(), rotation.e1(), rotation.e2(), rotation.e3());

    for (int i = 0; i < pointlist.size(); i++) {
        local_convex_data.push_back(real3(pointlist[i].x(), pointlist[i].y(), pointlist[i].z()));
    }

    m_shapes.push_back(std::shared_ptr<ChCollisionShape>(shape));

    return true;
}

bool ChCollisionModelParallel::AddBarrel(std::shared_ptr<ChMaterialSurface> material,
                                         double Y_low,
                                         double Y_high,
                                         double R_vert,
                                         double R_hor,
                                         double R_offset,
                                         const ChVector<>& pos,
                                         const ChMatrix33<>& rot) {
    // NOT SUPPORTED
    return false;
}

/// Add a triangle mesh to this model
bool ChCollisionModelParallel::AddTriangleMesh(std::shared_ptr<ChMaterialSurface> material,
                                               std::shared_ptr<geometry::ChTriangleMesh> trimesh,
                                               bool is_static,
                                               bool is_convex,
                                               const ChVector<>& pos,
                                               const ChMatrix33<>& rot,
                                               double sphereswept_thickness) {
    ChFrame<> frame;
    TransformToCOG(GetBody(), pos, rot, frame);
    const ChVector<>& position = frame.GetPos();
    const ChQuaternion<>& rotation = frame.GetRot();

    for (int i = 0; i < trimesh->getNumTriangles(); i++) {
        auto shape = new ChCollisionShapeParallel(ChCollisionShape::Type::TRIANGLE, material);
        geometry::ChTriangle temptri = trimesh->getTriangle(i);
        shape->A = real3(temptri.p1.x() + position.x(), temptri.p1.y() + position.y(), temptri.p1.z() + position.z());
        shape->B = real3(temptri.p2.x() + position.x(), temptri.p2.y() + position.y(), temptri.p2.z() + position.z());
        shape->C = real3(temptri.p3.x() + position.x(), temptri.p3.y() + position.y(), temptri.p3.z() + position.z());
        shape->R = quaternion(rotation.e0(), rotation.e1(), rotation.e2(), rotation.e3());
        m_shapes.push_back(std::shared_ptr<ChCollisionShape>(shape));
    }

    return true;
}

bool ChCollisionModelParallel::AddCopyOfAnotherModel(ChCollisionModel* another) {
    // NOT SUPPORTED
    return false;
}

void ChCollisionModelParallel::GetAABB(ChVector<>& bbmin, ChVector<>& bbmax) const {
    bbmin.x() = aabb_min.x;
    bbmin.y() = aabb_min.y;
    bbmin.z() = aabb_min.z;
    bbmax.x() = aabb_max.x;
    bbmax.y() = aabb_max.y;
    bbmax.z() = aabb_max.z;
}

ChCoordsys<> ChCollisionModelParallel::GetShapePos(int index) const {
    auto shape = std::static_pointer_cast<ChCollisionShapeParallel>(m_shapes[index]);
    const real3& p = shape->A;
    const quaternion& q = shape->R;
    return ChCoordsys<>(ChVector<>((double)p.x, (double)p.y, (double)p.z),
                        ChQuaternion<>((double)q.w, (double)q.x, (double)q.y, (double)q.z));
}

std::vector<double> ChCollisionModelParallel::GetShapeDimensions(int index) const {
    assert(index < GetNumShapes());

    auto shape = std::static_pointer_cast<ChCollisionShapeParallel>(m_shapes[index]);

    std::vector<double> dims;
    switch (m_shapes[index]->GetType()) {
        case ChCollisionShape::Type::SPHERE:
            dims = {shape->B.x};
            break;
        case ChCollisionShape::Type::ELLIPSOID:
            dims = {shape->B.x, shape->B.y, shape->B.z};
            break;
        case ChCollisionShape::Type::BOX:
            dims = {shape->B.x, shape->B.y, shape->B.z};
            break;
        case ChCollisionShape::Type::CYLINDER:
            dims = {shape->B.x, shape->B.z, shape->B.y};
            break;
        case ChCollisionShape::Type::CAPSULE:
            dims = {shape->B.x, shape->B.y};
            break;
        case ChCollisionShape::Type::CONE:
            dims = {shape->B.x, shape->B.z, shape->B.y};
            break;
        case ChCollisionShape::Type::ROUNDEDBOX:
            dims = {shape->B.x, shape->B.y, shape->B.z, shape->C.x};
            break;
        case ChCollisionShape::Type::ROUNDEDCYL:
            dims = {shape->B.x, shape->B.z, shape->B.y, shape->C.x};
            break;
        default:
            break;
    }
    return dims;
}

void ChCollisionModelParallel::SyncPosition() {
    ChBody* bpointer = GetBody();
    assert(bpointer);
    // assert(bpointer->GetSystem());
}

void ChCollisionModelParallel::SetContactable(ChContactable* mc) {
    // Invoke the base class method.
    ChCollisionModel::SetContactable(mc);

    // Currently, a ChCollisionModelParallel can only be a associated with a rigid body.
    mbody = dynamic_cast<ChBody*>(mc);
    assert(mbody);
}

}  // end namespace collision
}  // end namespace chrono
