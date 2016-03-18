// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Hammad Mazhar, Arman Pazouki
// =============================================================================
//
// Description: class for a parallel collision model
// =============================================================================
#include "chrono_parallel/collision/ChCCollisionModelParallel.h"
#include "physics/ChBody.h"
#include "physics/ChBodyAuxRef.h"
#include "physics/ChSystem.h"

namespace chrono {
namespace collision {

ChCollisionModelParallel::ChCollisionModelParallel() : nObjects(0), total_volume(0) {
    model_safe_margin = 0;
}

ChCollisionModelParallel::~ChCollisionModelParallel() {
    mData.clear();
}

int ChCollisionModelParallel::ClearModel() {
    if (GetPhysicsItem()->GetSystem() && GetPhysicsItem()->GetCollide()) {
        GetPhysicsItem()->GetSystem()->GetCollisionSystem()->Remove(this);
    }

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
    ConvexShape tData;
    tData.A = R3(position.x, position.y, position.z);
    tData.B = R3(radius, 0, 0);
    tData.C = R3(0, 0, 0);
    tData.R = R4(1, 0, 0, 0);
    tData.type = SPHERE;
    tData.margin = model_safe_margin;
    mData.push_back(tData);
    total_volume += 4.0 / 3.0 * CH_C_PI * pow(radius, 3.0);

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
    ConvexShape tData;
    tData.A = R3(position.x, position.y, position.z);
    tData.B = R3(rx, ry, rz);
    tData.C = R3(0, 0, 0);
    tData.R = R4(rotation.e0, rotation.e1, rotation.e2, rotation.e3);
    tData.type = ELLIPSOID;
    tData.margin = model_safe_margin;
    mData.push_back(tData);
    total_volume += 4.0 / 3.0 * CH_C_PI * rx * ry * rz;
    return true;
}

bool ChCollisionModelParallel::AddBox(double rx, double ry, double rz, const ChVector<>& pos, const ChMatrix33<>& rot) {
    ChFrame<> frame;
    TransformToCOG(GetBody(), pos, rot, frame);
    const ChVector<>& position = frame.GetPos();
    const ChQuaternion<>& rotation = frame.GetRot();

    nObjects++;
    ConvexShape tData;
    tData.A = R3(position.x, position.y, position.z);
    tData.B = R3(rx, ry, rz);
    tData.C = R3(0, 0, 0);
    tData.R = R4(rotation.e0, rotation.e1, rotation.e2, rotation.e3);
    tData.type = BOX;
    tData.margin = model_safe_margin;
    mData.push_back(tData);
    total_volume += rx * 2 * ry * 2 * rz * 2;
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
    ConvexShape tData;
    tData.A = R3(position.x, position.y, position.z);
    tData.B = R3(rx, ry, rz);
    tData.C = R3(sphere_r, 0, 0);
    tData.R = R4(rotation.e0, rotation.e1, rotation.e2, rotation.e3);
    tData.type = ROUNDEDBOX;
    tData.margin = model_safe_margin;
    mData.push_back(tData);
    total_volume += rx * 2 * ry * 2 * rz * 2;
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

    double mass = GetBody()->GetMass();

    nObjects++;
    ConvexShape tData;
    tData.A = R3(A.x + position.x, A.y + position.y, A.z + position.z);
    tData.B = R3(B.x + position.x, B.y + position.y, B.z + position.z);
    tData.C = R3(C.x + position.x, C.y + position.y, C.z + position.z);
    tData.R = R4(rotation.e0, rotation.e1, rotation.e2, rotation.e3);
    tData.type = TRIANGLEMESH;
    tData.margin = model_safe_margin;
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
    ConvexShape tData;
    tData.A = R3(position.x, position.y, position.z);
    tData.B = R3(rx, hy, rz);
    tData.C = R3(0, 0, 0);
    tData.R = R4(rotation.e0, rotation.e1, rotation.e2, rotation.e3);
    tData.type = CYLINDER;
    tData.margin = model_safe_margin;
    mData.push_back(tData);
    total_volume += CH_C_PI * rx * rz * hy * 2;
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
    ConvexShape tData;
    tData.A = R3(position.x, position.y, position.z);
    tData.B = R3(rx, hy, rz);
    tData.C = R3(sphere_r, 0, 0);
    tData.R = R4(rotation.e0, rotation.e1, rotation.e2, rotation.e3);
    tData.type = ROUNDEDCYL;
    tData.margin = model_safe_margin;
    mData.push_back(tData);
    total_volume += CH_C_PI * rx * rz * hy * 2;
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
    ConvexShape tData;
    tData.A = R3(position.x, position.y, position.z);
    tData.B = R3(rx, hy, rz);
    tData.C = R3(0, 0, 0);
    tData.R = R4(rotation.e0, rotation.e1, rotation.e2, rotation.e3);
    tData.type = CONE;
    tData.margin = model_safe_margin;
    mData.push_back(tData);
    total_volume += 1 / 3.0 * rx * hy * 2;
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
    ConvexShape tData;
    tData.A = R3(position.x, position.y, position.z);
    tData.B = R3(rx, hy, rz);
    tData.C = R3(sphere_r, 0, 0);
    tData.R = R4(rotation.e0, rotation.e1, rotation.e2, rotation.e3);
    tData.type = ROUNDEDCONE;
    tData.margin = model_safe_margin;
    mData.push_back(tData);
    total_volume += 1 / 3.0 * rx * hy * 2;
    return true;
}

bool ChCollisionModelParallel::AddCapsule(double radius, double hlen, const ChVector<>& pos, const ChMatrix33<>& rot) {
    ChFrame<> frame;
    TransformToCOG(GetBody(), pos, rot, frame);
    const ChVector<>& position = frame.GetPos();
    const ChQuaternion<>& rotation = frame.GetRot();

    nObjects++;
    ConvexShape tData;
    tData.A = R3(position.x, position.y, position.z);
    tData.B = R3(radius, hlen, radius);
    tData.C = R3(0, 0, 0);
    tData.R = R4(rotation.e0, rotation.e1, rotation.e2, rotation.e3);
    tData.type = CAPSULE;
    tData.margin = model_safe_margin;
    mData.push_back(tData);

    total_volume += 2 * CH_C_PI * radius * radius * (hlen + 2 * radius / 3);

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
    ConvexShape tData;
    tData.A = R3(position.x, position.y, position.z);
    tData.B = R3(pointlist.size(), local_convex_data.size(), 0);
    tData.C = R3(0, 0, 0);
    tData.R = R4(rotation.e0, rotation.e1, rotation.e2, rotation.e3);
    tData.type = CONVEX;
    tData.margin = model_safe_margin;
    mData.push_back(tData);
    total_volume += 0;

    for (int i = 0; i < pointlist.size(); i++) {
        local_convex_data.push_back(R3(pointlist[i].x, pointlist[i].y, pointlist[i].z));
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
    ConvexShape tData;
    for (int i = 0; i < trimesh.getNumTriangles(); i++) {
        geometry::ChTriangle temptri = trimesh.getTriangle(i);
        tData.A = R3(temptri.p1.x + position.x, temptri.p1.y + position.y, temptri.p1.z + position.z);
        tData.B = R3(temptri.p2.x + position.x, temptri.p2.y + position.y, temptri.p2.z + position.z);
        tData.C = R3(temptri.p3.x + position.x, temptri.p3.y + position.y, temptri.p3.z + position.z);
        tData.R = R4(rotation.e0, rotation.e1, rotation.e2, rotation.e3);
        tData.type = TRIANGLEMESH;
        tData.margin = model_safe_margin;
        mData.push_back(tData);
    }

    return true;
}

bool ChCollisionModelParallel::AddCopyOfAnotherModel(ChCollisionModel* another) {
    // NOT SUPPORTED
    return false;
}
void ChCollisionModelParallel::GetAABB(ChVector<>& bbmin, ChVector<>& bbmax) const {
}

void ChCollisionModelParallel::SyncPosition() {
    ChBody* bpointer = GetBody();
    assert(bpointer);
    // assert(bpointer->GetSystem());
}

float ChCollisionModelParallel::getVolume() {
    return total_volume;
}

void ChCollisionModelParallel::SetContactable(ChContactable* mc) {
    // Invoke the base class method.
    ChCollisionModel::SetContactable(mc);

    // Currently, a ChCollisionModelParallel can only be a associated with a rigid body.
    mbody = dynamic_cast<ChBody*>(mc);
    assert(mbody);
}

}  // END_OF_NAMESPACE____
}  // END_OF_NAMESPACE____
