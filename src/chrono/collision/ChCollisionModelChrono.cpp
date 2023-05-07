// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2021 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Hammad Mazhar
// =============================================================================
// Geometric model for the custom multicore Chrono collision system
//
// TODO: Collision family information must currently be set BEFORE the collision
//       model is added to the collision system!   Can this be relaxed?
//       Note that we can figure out if the model was added by checking whether
//       the associated body has a ChSystem (mbody->GetSystem())
// =============================================================================

#include "chrono/collision/ChCollisionModelChrono.h"

#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChBodyAuxRef.h"
#include "chrono/physics/ChSystem.h"

namespace chrono {
namespace collision {

ChCollisionModelChrono::ChCollisionModelChrono() : aabb_min(C_REAL_MAX), aabb_max(-C_REAL_MAX) {
    model_safe_margin = 0;
}

ChCollisionModelChrono::~ChCollisionModelChrono() {
    m_shapes.clear();
}

int ChCollisionModelChrono::ClearModel() {
    if (GetPhysicsItem()->GetSystem() && GetPhysicsItem()->GetCollide()) {
        GetPhysicsItem()->GetSystem()->GetCollisionSystem()->Remove(this);
    }

    local_convex_data.clear();
    m_shapes.clear();
    aabb_min = ChVector<>(C_REAL_MAX);
    aabb_max = ChVector<>(-C_REAL_MAX);
    family_group = 1;
    family_mask = 0x7FFF;

    return 1;
}

int ChCollisionModelChrono::BuildModel() {
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

bool ChCollisionModelChrono::AddSphere(std::shared_ptr<ChMaterialSurface> material,
                                       double radius,
                                       const ChVector<>& pos) {
    ChFrame<> frame;
    TransformToCOG(GetBody(), pos, ChMatrix33<>(1), frame);
    const ChVector<>& position = frame.GetPos();

    auto shape = new ChCollisionShapeChrono(ChCollisionShape::Type::SPHERE, material);
    shape->A = real3(position.x(), position.y(), position.z());
    shape->B = real3(radius, 0, 0);
    shape->C = real3(0, 0, 0);
    shape->R = quaternion(1, 0, 0, 0);
    m_shapes.push_back(std::shared_ptr<ChCollisionShape>(shape));

    return true;
}

bool ChCollisionModelChrono::AddEllipsoid(std::shared_ptr<ChMaterialSurface> material,
                                          double axis_x,
                                          double axis_y,
                                          double axis_z,
                                          const ChVector<>& pos,
                                          const ChMatrix33<>& rot) {
    ChFrame<> frame;
    TransformToCOG(GetBody(), pos, rot, frame);
    const ChVector<>& position = frame.GetPos();
    const ChQuaternion<>& rotation = frame.GetRot();

    auto shape = new ChCollisionShapeChrono(ChCollisionShape::Type::ELLIPSOID, material);
    shape->A = real3(position.x(), position.y(), position.z());
    shape->B = real3(axis_x / 2, axis_y / 2, axis_z / 2);
    shape->C = real3(0, 0, 0);
    shape->R = quaternion(rotation.e0(), rotation.e1(), rotation.e2(), rotation.e3());
    m_shapes.push_back(std::shared_ptr<ChCollisionShape>(shape));

    return true;
}

bool ChCollisionModelChrono::AddBox(std::shared_ptr<ChMaterialSurface> material,
                                    double size_x,
                                    double size_y,
                                    double size_z,
                                    const ChVector<>& pos,
                                    const ChMatrix33<>& rot) {
    ChFrame<> frame;
    TransformToCOG(GetBody(), pos, rot, frame);
    const ChVector<>& position = frame.GetPos();
    const ChQuaternion<>& rotation = frame.GetRot();

    auto shape = new ChCollisionShapeChrono(ChCollisionShape::Type::BOX, material);
    shape->A = real3(position.x(), position.y(), position.z());
    shape->B = real3(size_x / 2, size_y / 2, size_z / 2);
    shape->C = real3(0, 0, 0);
    shape->R = quaternion(rotation.e0(), rotation.e1(), rotation.e2(), rotation.e3());
    m_shapes.push_back(std::shared_ptr<ChCollisionShape>(shape));

    return true;
}

bool ChCollisionModelChrono::AddRoundedBox(std::shared_ptr<ChMaterialSurface> material,
                                           double size_x,
                                           double size_y,
                                           double size_z,
                                           double sphere_r,
                                           const ChVector<>& pos,
                                           const ChMatrix33<>& rot) {
    ChFrame<> frame;
    TransformToCOG(GetBody(), pos, rot, frame);
    const ChVector<>& position = frame.GetPos();
    const ChQuaternion<>& rotation = frame.GetRot();

    auto shape = new ChCollisionShapeChrono(ChCollisionShape::Type::ROUNDEDBOX, material);
    shape->A = real3(position.x(), position.y(), position.z());
    shape->B = real3(size_x / 2, size_y / 2, size_z / 2);
    shape->C = real3(sphere_r, 0, 0);
    shape->R = quaternion(rotation.e0(), rotation.e1(), rotation.e2(), rotation.e3());
    m_shapes.push_back(std::shared_ptr<ChCollisionShape>(shape));

    return true;
}

bool ChCollisionModelChrono::AddTriangle(std::shared_ptr<ChMaterialSurface> material,
                                         ChVector<> A,
                                         ChVector<> B,
                                         ChVector<> C,
                                         const ChVector<>& pos,
                                         const ChMatrix33<>& rot) {
    ChFrame<> frame;
    TransformToCOG(GetBody(), pos, rot, frame);
    const ChVector<>& position = frame.GetPos();
    const ChQuaternion<>& rotation = frame.GetRot();

    auto shape = new ChCollisionShapeChrono(ChCollisionShape::Type::TRIANGLE, material);
    shape->A = real3(A.x() + position.x(), A.y() + position.y(), A.z() + position.z());
    shape->B = real3(B.x() + position.x(), B.y() + position.y(), B.z() + position.z());
    shape->C = real3(C.x() + position.x(), C.y() + position.y(), C.z() + position.z());
    shape->R = quaternion(rotation.e0(), rotation.e1(), rotation.e2(), rotation.e3());
    m_shapes.push_back(std::shared_ptr<ChCollisionShape>(shape));

    return true;
}

bool ChCollisionModelChrono::AddCylinder(std::shared_ptr<ChMaterialSurface> material,
                                         double radius,
                                         double height,
                                         const ChVector<>& pos,
                                         const ChMatrix33<>& rot) {
    ChFrame<> frame;
    TransformToCOG(GetBody(), pos, rot, frame);
    const ChVector<>& position = frame.GetPos();
    const ChQuaternion<>& rotation = frame.GetRot();

    auto shape = new ChCollisionShapeChrono(ChCollisionShape::Type::CYLINDER, material);
    shape->A = real3(position.x(), position.y(), position.z());
    shape->B = real3(radius, radius, height / 2);
    shape->C = real3(0, 0, 0);
    shape->R = quaternion(rotation.e0(), rotation.e1(), rotation.e2(), rotation.e3());
    m_shapes.push_back(std::shared_ptr<ChCollisionShape>(shape));

    return true;
}

bool ChCollisionModelChrono::AddCylindricalShell(std::shared_ptr<ChMaterialSurface> material,
                                                 double radius,
                                                 double height,
                                                 const ChVector<>& pos,
                                                 const ChMatrix33<>& rot) {
    ChFrame<> frame;
    TransformToCOG(GetBody(), pos, rot, frame);
    const ChVector<>& position = frame.GetPos();
    const ChQuaternion<>& rotation = frame.GetRot();

    auto shape = new ChCollisionShapeChrono(ChCollisionShape::Type::CYLSHELL, material);
    shape->A = real3(position.x(), position.y(), position.z());
    shape->B = real3(radius, radius, height / 2);
    shape->C = real3(0, 0, 0);
    shape->R = quaternion(rotation.e0(), rotation.e1(), rotation.e2(), rotation.e3());
    m_shapes.push_back(std::shared_ptr<ChCollisionShape>(shape));

    return true;
}

bool ChCollisionModelChrono::AddRoundedCylinder(std::shared_ptr<ChMaterialSurface> material,
                                                double radius,
                                                double height,
                                                double sphere_r,
                                                const ChVector<>& pos,
                                                const ChMatrix33<>& rot) {
    ChFrame<> frame;
    TransformToCOG(GetBody(), pos, rot, frame);
    const ChVector<>& position = frame.GetPos();
    const ChQuaternion<>& rotation = frame.GetRot();

    auto shape = new ChCollisionShapeChrono(ChCollisionShape::Type::ROUNDEDCYL, material);
    shape->A = real3(position.x(), position.y(), position.z());
    shape->B = real3(radius, radius, height / 2);
    shape->C = real3(sphere_r, 0, 0);
    shape->R = quaternion(rotation.e0(), rotation.e1(), rotation.e2(), rotation.e3());
    m_shapes.push_back(std::shared_ptr<ChCollisionShape>(shape));

    return true;
}

bool ChCollisionModelChrono::AddCone(std::shared_ptr<ChMaterialSurface> material,
                                     double radius,
                                     double height,
                                     const ChVector<>& pos,
                                     const ChMatrix33<>& rot) {
    ChFrame<> frame;
    TransformToCOG(GetBody(), pos, rot, frame);
    const ChVector<>& position = frame.GetPos();
    const ChQuaternion<>& rotation = frame.GetRot();

    auto shape = new ChCollisionShapeChrono(ChCollisionShape::Type::CONE, material);
    shape->A = real3(position.x(), position.y(), position.z());
    shape->B = real3(radius, radius, height / 2);
    shape->C = real3(0, 0, 0);
    shape->R = quaternion(rotation.e0(), rotation.e1(), rotation.e2(), rotation.e3());
    m_shapes.push_back(std::shared_ptr<ChCollisionShape>(shape));

    return true;
}

bool ChCollisionModelChrono::AddCapsule(std::shared_ptr<ChMaterialSurface> material,
                                        double radius,
                                        double height,
                                        const ChVector<>& pos,
                                        const ChMatrix33<>& rot) {
    ChFrame<> frame;
    TransformToCOG(GetBody(), pos, rot, frame);
    const ChVector<>& position = frame.GetPos();
    const ChQuaternion<>& rotation = frame.GetRot();

    auto shape = new ChCollisionShapeChrono(ChCollisionShape::Type::CAPSULE, material);
    shape->A = real3(position.x(), position.y(), position.z());
    shape->B = real3(radius, radius, height / 2);
    shape->C = real3(0, 0, 0);
    shape->R = quaternion(rotation.e0(), rotation.e1(), rotation.e2(), rotation.e3());
    m_shapes.push_back(std::shared_ptr<ChCollisionShape>(shape));

    return true;
}

bool ChCollisionModelChrono::AddConvexHull(std::shared_ptr<ChMaterialSurface> material,
                                           const std::vector<ChVector<double> >& pointlist,
                                           const ChVector<>& pos,
                                           const ChMatrix33<>& rot) {
    ChFrame<> frame;
    TransformToCOG(GetBody(), pos, rot, frame);
    const ChVector<>& position = frame.GetPos();
    const ChQuaternion<>& rotation = frame.GetRot();

    auto shape = new ChCollisionShapeChrono(ChCollisionShape::Type::CONVEX, material);
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

bool ChCollisionModelChrono::AddBarrel(std::shared_ptr<ChMaterialSurface> material,
                                       double Y_low,
                                       double Y_high,
                                       double axis_vert,
                                       double axis_hor,
                                       double R_offset,
                                       const ChVector<>& pos,
                                       const ChMatrix33<>& rot) {
    // NOT SUPPORTED
    return false;
}

/// Add a triangle mesh to this model
bool ChCollisionModelChrono::AddTriangleMesh(std::shared_ptr<ChMaterialSurface> material,
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
        auto shape = new ChCollisionShapeChrono(ChCollisionShape::Type::TRIANGLE, material);
        geometry::ChTriangle temptri = trimesh->getTriangle(i);
        shape->A = real3(temptri.p1.x() + position.x(), temptri.p1.y() + position.y(), temptri.p1.z() + position.z());
        shape->B = real3(temptri.p2.x() + position.x(), temptri.p2.y() + position.y(), temptri.p2.z() + position.z());
        shape->C = real3(temptri.p3.x() + position.x(), temptri.p3.y() + position.y(), temptri.p3.z() + position.z());
        shape->R = quaternion(rotation.e0(), rotation.e1(), rotation.e2(), rotation.e3());
        m_shapes.push_back(std::shared_ptr<ChCollisionShape>(shape));
    }

    return true;
}

bool ChCollisionModelChrono::AddCopyOfAnotherModel(ChCollisionModel* another) {
    // NOT SUPPORTED
    return false;
}

void ChCollisionModelChrono::GetAABB(ChVector<>& bbmin, ChVector<>& bbmax) const {
    bbmin = aabb_min;
    bbmax = aabb_max;
}

ChCoordsys<> ChCollisionModelChrono::GetShapePos(int index) const {
    auto shape = std::static_pointer_cast<ChCollisionShapeChrono>(m_shapes[index]);
    const real3& p = shape->A;
    const quaternion& q = shape->R;
    return ChCoordsys<>(ChVector<>((double)p.x, (double)p.y, (double)p.z),
                        ChQuaternion<>((double)q.w, (double)q.x, (double)q.y, (double)q.z));
}

std::vector<double> ChCollisionModelChrono::GetShapeDimensions(int index) const {
    assert(index < GetNumShapes());

    auto shape = std::static_pointer_cast<ChCollisionShapeChrono>(m_shapes[index]);

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

void ChCollisionModelChrono::SyncPosition() {
#if !defined(NDEBUG)
    ChBody* bpointer = GetBody();
    assert(bpointer);
    assert(bpointer->GetSystem());
#endif
}

void ChCollisionModelChrono::SetContactable(ChContactable* mc) {
    // Invoke the base class method.
    ChCollisionModel::SetContactable(mc);

    // Currently, a ChCollisionModelChrono can only be a associated with a rigid body.
    mbody = dynamic_cast<ChBody*>(mc);
    assert(mbody);
}

}  // end namespace collision
}  // end namespace chrono
