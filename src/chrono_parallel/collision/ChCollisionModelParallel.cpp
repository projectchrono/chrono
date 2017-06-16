// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2016 projectchrono.org
// All right reserved.
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
// =============================================================================

#include "chrono_parallel/collision/ChCollisionModelParallel.h"

#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChBodyAuxRef.h"
#include "chrono/physics/ChSystem.h"

namespace chrono {
namespace collision {

ChCollisionModelParallel::ChCollisionModelParallel() : nObjects(0) {
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
    nObjects = 0;
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

bool ChCollisionModelParallel::AddSphere(double radius, const ChVector<>& pos) {
    ChFrame<> frame;
    TransformToCOG(GetBody(), pos, ChMatrix33<>(1), frame);
    const ChVector<>& position = frame.GetPos();

    nObjects++;
    ConvexModel tData;
    tData.A = real3(position.x(), position.y(), position.z());
    tData.B = real3(radius, 0, 0);
    tData.C = real3(0, 0, 0);
    tData.R = quaternion(1, 0, 0, 0);
    tData.type = SPHERE;
    mData.push_back(tData);

    return true;
}

bool ChCollisionModelParallel::AddEllipsoid(double rx,
                                            double ry,
                                            double rz,
                                            const ChVector<>& pos,
                                            const ChMatrix33<>& rot) {
    ChFrame<> frame;
    TransformToCOG(GetBody(), pos, rot, frame);
    const ChVector<>& position = frame.GetPos();
    const ChQuaternion<>& rotation = frame.GetRot();

    nObjects++;
    ConvexModel tData;
    tData.A = real3(position.x(), position.y(), position.z());
    tData.B = real3(rx, ry, rz);
    tData.C = real3(0, 0, 0);
    tData.R = quaternion(rotation.e0(), rotation.e1(), rotation.e2(), rotation.e3());
    tData.type = ELLIPSOID;
    mData.push_back(tData);

    return true;
}

bool ChCollisionModelParallel::AddBox(double rx, double ry, double rz, const ChVector<>& pos, const ChMatrix33<>& rot) {
    ChFrame<> frame;
    TransformToCOG(GetBody(), pos, rot, frame);
    const ChVector<>& position = frame.GetPos();
    const ChQuaternion<>& rotation = frame.GetRot();

    nObjects++;
    ConvexModel tData;
    tData.A = real3(position.x(), position.y(), position.z());
    tData.B = real3(rx, ry, rz);
    tData.C = real3(0, 0, 0);
    tData.R = quaternion(rotation.e0(), rotation.e1(), rotation.e2(), rotation.e3());
    tData.type = BOX;
    mData.push_back(tData);

    return true;
}

bool ChCollisionModelParallel::AddRoundedBox(double rx,
                                             double ry,
                                             double rz,
                                             double sphere_r,
                                             const ChVector<>& pos,
                                             const ChMatrix33<>& rot) {
    ChFrame<> frame;
    TransformToCOG(GetBody(), pos, rot, frame);
    const ChVector<>& position = frame.GetPos();
    const ChQuaternion<>& rotation = frame.GetRot();

    nObjects++;
    ConvexModel tData;
    tData.A = real3(position.x(), position.y(), position.z());
    tData.B = real3(rx, ry, rz);
    tData.C = real3(sphere_r, 0, 0);
    tData.R = quaternion(rotation.e0(), rotation.e1(), rotation.e2(), rotation.e3());
    tData.type = ROUNDEDBOX;
    mData.push_back(tData);

    return true;
}

bool ChCollisionModelParallel::AddTriangle(ChVector<> A,
                                           ChVector<> B,
                                           ChVector<> C,
                                           const ChVector<>& pos,
                                           const ChMatrix33<>& rot) {
    ChFrame<> frame;
    TransformToCOG(GetBody(), pos, rot, frame);
    const ChVector<>& position = frame.GetPos();
    const ChQuaternion<>& rotation = frame.GetRot();

    nObjects++;
    ConvexModel tData;
    tData.A = real3(A.x() + position.x(), A.y() + position.y(), A.z() + position.z());
    tData.B = real3(B.x() + position.x(), B.y() + position.y(), B.z() + position.z());
    tData.C = real3(C.x() + position.x(), C.y() + position.y(), C.z() + position.z());
    tData.R = quaternion(rotation.e0(), rotation.e1(), rotation.e2(), rotation.e3());
    tData.type = TRIANGLEMESH;
    mData.push_back(tData);

    return true;
}

bool ChCollisionModelParallel::AddCylinder(double rx,
                                           double rz,
                                           double hy,
                                           const ChVector<>& pos,
                                           const ChMatrix33<>& rot) {
    ChFrame<> frame;
    TransformToCOG(GetBody(), pos, rot, frame);
    const ChVector<>& position = frame.GetPos();
    const ChQuaternion<>& rotation = frame.GetRot();

    nObjects++;
    ConvexModel tData;
    tData.A = real3(position.x(), position.y(), position.z());
    tData.B = real3(rx, hy, rz);
    tData.C = real3(0, 0, 0);
    tData.R = quaternion(rotation.e0(), rotation.e1(), rotation.e2(), rotation.e3());
    tData.type = CYLINDER;
    mData.push_back(tData);

    return true;
}

bool ChCollisionModelParallel::AddRoundedCylinder(double rx,
                                                  double rz,
                                                  double hy,
                                                  double sphere_r,
                                                  const ChVector<>& pos,
                                                  const ChMatrix33<>& rot) {
    ChFrame<> frame;
    TransformToCOG(GetBody(), pos, rot, frame);
    const ChVector<>& position = frame.GetPos();
    const ChQuaternion<>& rotation = frame.GetRot();

    nObjects++;
    ConvexModel tData;
    tData.A = real3(position.x(), position.y(), position.z());
    tData.B = real3(rx, hy, rz);
    tData.C = real3(sphere_r, 0, 0);
    tData.R = quaternion(rotation.e0(), rotation.e1(), rotation.e2(), rotation.e3());
    tData.type = ROUNDEDCYL;
    mData.push_back(tData);

    return true;
}

bool ChCollisionModelParallel::AddCone(double rx,
                                       double rz,
                                       double hy,
                                       const ChVector<>& pos,
                                       const ChMatrix33<>& rot) {
    ChFrame<> frame;
    TransformToCOG(GetBody(), pos, rot, frame);
    const ChVector<>& position = frame.GetPos();
    const ChQuaternion<>& rotation = frame.GetRot();

    nObjects++;
    ConvexModel tData;
    tData.A = real3(position.x(), position.y(), position.z());
    tData.B = real3(rx, hy, rz);
    tData.C = real3(0, 0, 0);
    tData.R = quaternion(rotation.e0(), rotation.e1(), rotation.e2(), rotation.e3());
    tData.type = CONE;
    mData.push_back(tData);

    return true;
}

bool ChCollisionModelParallel::AddRoundedCone(double rx,
                                              double rz,
                                              double hy,
                                              double sphere_r,
                                              const ChVector<>& pos,
                                              const ChMatrix33<>& rot) {
    ChFrame<> frame;
    TransformToCOG(GetBody(), pos, rot, frame);
    const ChVector<>& position = frame.GetPos();
    const ChQuaternion<>& rotation = frame.GetRot();

    nObjects++;
    ConvexModel tData;
    tData.A = real3(position.x(), position.y(), position.z());
    tData.B = real3(rx, hy, rz);
    tData.C = real3(sphere_r, 0, 0);
    tData.R = quaternion(rotation.e0(), rotation.e1(), rotation.e2(), rotation.e3());
    tData.type = ROUNDEDCONE;
    mData.push_back(tData);

    return true;
}

bool ChCollisionModelParallel::AddCapsule(double radius, double hlen, const ChVector<>& pos, const ChMatrix33<>& rot) {
    ChFrame<> frame;
    TransformToCOG(GetBody(), pos, rot, frame);
    const ChVector<>& position = frame.GetPos();
    const ChQuaternion<>& rotation = frame.GetRot();

    nObjects++;
    ConvexModel tData;
    tData.A = real3(position.x(), position.y(), position.z());
    tData.B = real3(radius, hlen, radius);
    tData.C = real3(0, 0, 0);
    tData.R = quaternion(rotation.e0(), rotation.e1(), rotation.e2(), rotation.e3());
    tData.type = CAPSULE;
    mData.push_back(tData);

    return true;
}

bool ChCollisionModelParallel::AddConvexHull(std::vector<ChVector<double> >& pointlist,
                                             const ChVector<>& pos,
                                             const ChMatrix33<>& rot) {
    ChFrame<> frame;
    TransformToCOG(GetBody(), pos, rot, frame);
    const ChVector<>& position = frame.GetPos();
    const ChQuaternion<>& rotation = frame.GetRot();

    nObjects++;
    ConvexModel tData;
    tData.A = real3(position.x(), position.y(), position.z());
    tData.B = real3((chrono::real)pointlist.size(), (chrono::real)local_convex_data.size(), 0);
    tData.C = real3(0, 0, 0);
    tData.R = quaternion(rotation.e0(), rotation.e1(), rotation.e2(), rotation.e3());
    tData.type = CONVEX;

    mData.push_back(tData);

    for (int i = 0; i < pointlist.size(); i++) {
        local_convex_data.push_back(real3(pointlist[i].x(), pointlist[i].y(), pointlist[i].z()));
    }

    return true;
}

bool ChCollisionModelParallel::AddBarrel(double Y_low,
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
bool ChCollisionModelParallel::AddTriangleMesh(const geometry::ChTriangleMesh& trimesh,
                                               bool is_static,
                                               bool is_convex,
                                               const ChVector<>& pos,
                                               const ChMatrix33<>& rot,
                                               double sphereswept_thickness) {
    ChFrame<> frame;
    TransformToCOG(GetBody(), pos, rot, frame);
    const ChVector<>& position = frame.GetPos();
    const ChQuaternion<>& rotation = frame.GetRot();

    nObjects += trimesh.getNumTriangles();
    ConvexModel tData;
    for (int i = 0; i < trimesh.getNumTriangles(); i++) {
        geometry::ChTriangle temptri = trimesh.getTriangle(i);
        tData.A = real3(temptri.p1.x() + position.x(), temptri.p1.y() + position.y(), temptri.p1.z() + position.z());
        tData.B = real3(temptri.p2.x() + position.x(), temptri.p2.y() + position.y(), temptri.p2.z() + position.z());
        tData.C = real3(temptri.p3.x() + position.x(), temptri.p3.y() + position.y(), temptri.p3.z() + position.z());
        tData.R = quaternion(rotation.e0(), rotation.e1(), rotation.e2(), rotation.e3());
        tData.type = TRIANGLEMESH;

        mData.push_back(tData);
    }

    return true;
}

bool ChCollisionModelParallel::AddCopyOfAnotherModel(ChCollisionModel* another) {
    // NOT SUPPORTED
    return false;
}
void ChCollisionModelParallel::GetAABB(ChVector<>& bbmin, ChVector<>& bbmax) const {}

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
