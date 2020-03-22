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
    mData.clear();
}

int ChCollisionModelParallel::ClearModel() {
    if (GetPhysicsItem()->GetSystem() && GetPhysicsItem()->GetCollide()) {
        GetPhysicsItem()->GetSystem()->GetCollisionSystem()->Remove(this);
    }

    local_convex_data.clear();
    mData.clear();
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

    ConvexModel tData;
    tData.A = real3(position.x(), position.y(), position.z());
    tData.B = real3(radius, 0, 0);
    tData.C = real3(0, 0, 0);
    tData.R = quaternion(1, 0, 0, 0);
    tData.type = ChCollisionShape::Type::SPHERE;
    mData.push_back(tData);

    auto shape = new ChCollisionShape(ChCollisionShape::Type::SPHERE, material);
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

    ConvexModel tData;
    tData.A = real3(position.x(), position.y(), position.z());
    tData.B = real3(rx, ry, rz);
    tData.C = real3(0, 0, 0);
    tData.R = quaternion(rotation.e0(), rotation.e1(), rotation.e2(), rotation.e3());
    tData.type = ChCollisionShape::Type::ELLIPSOID;
    mData.push_back(tData);

    auto shape = new ChCollisionShape(ChCollisionShape::Type::ELLIPSOID, material);
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

    ConvexModel tData;
    tData.A = real3(position.x(), position.y(), position.z());
    tData.B = real3(rx, ry, rz);
    tData.C = real3(0, 0, 0);
    tData.R = quaternion(rotation.e0(), rotation.e1(), rotation.e2(), rotation.e3());
    tData.type = ChCollisionShape::Type::BOX;
    mData.push_back(tData);

    auto shape = new ChCollisionShape(ChCollisionShape::Type::BOX, material);
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

    ConvexModel tData;
    tData.A = real3(position.x(), position.y(), position.z());
    tData.B = real3(rx, ry, rz);
    tData.C = real3(sphere_r, 0, 0);
    tData.R = quaternion(rotation.e0(), rotation.e1(), rotation.e2(), rotation.e3());
    tData.type = ChCollisionShape::Type::ROUNDEDBOX;
    mData.push_back(tData);

    auto shape = new ChCollisionShape(ChCollisionShape::Type::ROUNDEDBOX, material);
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

    ConvexModel tData;
    tData.A = real3(A.x() + position.x(), A.y() + position.y(), A.z() + position.z());
    tData.B = real3(B.x() + position.x(), B.y() + position.y(), B.z() + position.z());
    tData.C = real3(C.x() + position.x(), C.y() + position.y(), C.z() + position.z());
    tData.R = quaternion(rotation.e0(), rotation.e1(), rotation.e2(), rotation.e3());
    tData.type = ChCollisionShape::Type::TRIANGLE;
    mData.push_back(tData);

    auto shape = new ChCollisionShape(ChCollisionShape::Type::TRIANGLE, material);
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

    ConvexModel tData;
    tData.A = real3(position.x(), position.y(), position.z());
    tData.B = real3(rx, hy, rz);
    tData.C = real3(0, 0, 0);
    tData.R = quaternion(rotation.e0(), rotation.e1(), rotation.e2(), rotation.e3());
    tData.type = ChCollisionShape::Type::CYLINDER;
    mData.push_back(tData);

    auto shape = new ChCollisionShape(ChCollisionShape::Type::CYLINDER, material);
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

    ConvexModel tData;
    tData.A = real3(position.x(), position.y(), position.z());
    tData.B = real3(rx, hy, rz);
    tData.C = real3(sphere_r, 0, 0);
    tData.R = quaternion(rotation.e0(), rotation.e1(), rotation.e2(), rotation.e3());
    tData.type = ChCollisionShape::Type::ROUNDEDCYL;
    mData.push_back(tData);

    auto shape = new ChCollisionShape(ChCollisionShape::Type::ROUNDEDCYL, material);
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

    ConvexModel tData;
    tData.A = real3(position.x(), position.y(), position.z());
    tData.B = real3(rx, hy, rz);
    tData.C = real3(0, 0, 0);
    tData.R = quaternion(rotation.e0(), rotation.e1(), rotation.e2(), rotation.e3());
    tData.type = ChCollisionShape::Type::CONE;
    mData.push_back(tData);

    auto shape = new ChCollisionShape(ChCollisionShape::Type::CONE, material);
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

    ConvexModel tData;
    tData.A = real3(position.x(), position.y(), position.z());
    tData.B = real3(rx, hy, rz);
    tData.C = real3(sphere_r, 0, 0);
    tData.R = quaternion(rotation.e0(), rotation.e1(), rotation.e2(), rotation.e3());
    tData.type = ChCollisionShape::Type::ROUNDEDCONE;
    mData.push_back(tData);

    auto shape = new ChCollisionShape(ChCollisionShape::Type::ROUNDEDCONE, material);
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

    ConvexModel tData;
    tData.A = real3(position.x(), position.y(), position.z());
    tData.B = real3(radius, hlen, radius);
    tData.C = real3(0, 0, 0);
    tData.R = quaternion(rotation.e0(), rotation.e1(), rotation.e2(), rotation.e3());
    tData.type = ChCollisionShape::Type::CAPSULE;
    mData.push_back(tData);

    auto shape = new ChCollisionShape(ChCollisionShape::Type::CAPSULE, material);
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

    ConvexModel tData;
    tData.A = real3(position.x(), position.y(), position.z());
    tData.B = real3((chrono::real)pointlist.size(), (chrono::real)local_convex_data.size(), 0);
    tData.C = real3(0, 0, 0);
    tData.R = quaternion(rotation.e0(), rotation.e1(), rotation.e2(), rotation.e3());
    tData.type = ChCollisionShape::Type::CONVEX;

    mData.push_back(tData);

    for (int i = 0; i < pointlist.size(); i++) {
        local_convex_data.push_back(real3(pointlist[i].x(), pointlist[i].y(), pointlist[i].z()));
    }

    auto shape = new ChCollisionShape(ChCollisionShape::Type::CONVEX, material);
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

    ConvexModel tData;
    for (int i = 0; i < trimesh->getNumTriangles(); i++) {
        geometry::ChTriangle temptri = trimesh->getTriangle(i);
        tData.A = real3(temptri.p1.x() + position.x(), temptri.p1.y() + position.y(), temptri.p1.z() + position.z());
        tData.B = real3(temptri.p2.x() + position.x(), temptri.p2.y() + position.y(), temptri.p2.z() + position.z());
        tData.C = real3(temptri.p3.x() + position.x(), temptri.p3.y() + position.y(), temptri.p3.z() + position.z());
        tData.R = quaternion(rotation.e0(), rotation.e1(), rotation.e2(), rotation.e3());
        tData.type = ChCollisionShape::Type::TRIANGLE;
        mData.push_back(tData);

        auto shape = new ChCollisionShape(ChCollisionShape::Type::TRIANGLE, material);
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
    const real3& p = mData[index].A;
    const quaternion& q = mData[index].R;
    return ChCoordsys<>(ChVector<>((double)p.x, (double)p.y, (double)p.z),
                        ChQuaternion<>((double)q.w, (double)q.x, (double)q.y, (double)q.z));
}

std::vector<double> ChCollisionModelParallel::GetShapeDimensions(int index) const {
    assert(index < GetNumShapes());

    const ConvexModel& data = mData[index];

    std::vector<double> dims;
    switch (m_shapes[index]->GetType()) {
        case ChCollisionShape::Type::SPHERE:
            dims = {data.B.x};
            break;
        case ChCollisionShape::Type::ELLIPSOID:
            dims = {data.B.x, data.B.y, data.B.z};
            break;
        case ChCollisionShape::Type::BOX:
            dims = {data.B.x, data.B.y, data.B.z};
            break;
        case ChCollisionShape::Type::CYLINDER:
            dims = {data.B.x, data.B.z, data.B.y};
            break;
        case ChCollisionShape::Type::CAPSULE:
            dims = {data.B.x, data.B.y};
            break;
        case ChCollisionShape::Type::CONE:
            dims = {data.B.x, data.B.z, data.B.y};
            break;
        case ChCollisionShape::Type::ROUNDEDBOX:
            dims = {data.B.x, data.B.y, data.B.z, data.C.x};
            break;
        case ChCollisionShape::Type::ROUNDEDCYL:
            dims = {data.B.x, data.B.z, data.B.y, data.C.x};
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
