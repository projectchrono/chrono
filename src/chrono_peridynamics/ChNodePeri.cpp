// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#include "chrono/physics/ChSystem.h"
#include "chrono_peridynamics/ChNodePeri.h"
#include "chrono/collision/bullet/ChCollisionModelBullet.h"

namespace chrono {

using namespace fea;
using namespace peridynamics;

// Register into the object factory, to enable run-time dynamic creation and persistence
// CH_FACTORY_REGISTER(ChNodePeri)

ChNodePeri::ChNodePeri() : volume(1e-6), h_rad(1.6e-6), coll_rad(0.005), vol_size(0.01) {
    SetMass(1e-6);
}

ChNodePeri::ChNodePeri(const ChNodePeri& other) : ChNodeFEAxyz(other) {
    h_rad = other.h_rad;
    SetMass(other.GetMass());
    volume = other.volume;
    vol_size = other.vol_size;
    F_peridyn = other.F_peridyn;
    is_boundary = other.is_boundary;
    is_colliding = other.is_colliding;
    is_fluid = other.is_fluid;
    is_requiring_bonds = other.is_requiring_bonds;

    variables = other.variables;
}

ChNodePeri::~ChNodePeri() {}

void ChNodePeri::SetHorizonRadius(double mr) {
    h_rad = mr;
    double aabb_rad = h_rad / 2;  // to avoid too many pairs: bounding boxes hemisizes will sum
    if (auto mshape = std::dynamic_pointer_cast<ChCollisionModelBullet>(GetCollisionModel()->GetShapeInstance(0).shape))
        mshape->SetSphereRadius(coll_rad, std::max(0.0, aabb_rad - coll_rad));
}

void ChNodePeri::SetCollisionRadius(double mr) {
    coll_rad = mr;
    double aabb_rad = h_rad / 2;  // to avoid too many pairs: bounding boxes hemisizes will sum
    if (auto mshape = std::dynamic_pointer_cast<ChCollisionModelBullet>(GetCollisionModel()->GetShapeInstance(0).shape))
        mshape->SetSphereRadius(coll_rad, std::max(0.0, aabb_rad - coll_rad));
}

// -----------------------------------------------------------------------------

void ChNodePeri::ContactableGetStateBlockPosLevel(ChState& x) {
    x.segment(0, 3) = this->pos.eigen();
}

void ChNodePeri::ContactableGetStateBlockVelLevel(ChStateDelta& w) {
    w.segment(0, 3) = this->pos_dt.eigen();
}

void ChNodePeri::ContactableIncrementState(const ChState& x, const ChStateDelta& dw, ChState& x_new) {
    NodeIntStateIncrement(0, x_new, x, 0, dw);
}

ChVector3d ChNodePeri::GetContactPoint(const ChVector3d& loc_point, const ChState& state_x) {
    return state_x.segment(0, 3);
}

ChVector3d ChNodePeri::GetContactPointSpeed(const ChVector3d& loc_point,
                                            const ChState& state_x,
                                            const ChStateDelta& state_w) {
    return state_w.segment(0, 3);
}

ChVector3d ChNodePeri::GetContactPointSpeed(const ChVector3d& abs_point) {
    return this->pos_dt;
}

ChFrame<> ChNodePeri::GetCollisionModelFrame() {
    return ChFrame<>(this->pos, QNULL);
}

void ChNodePeri::ContactForceLoadResidual_F(const ChVector3d& F,
                                            const ChVector3d& T,
                                            const ChVector3d& abs_point,
                                            ChVectorDynamic<>& R) {
    R.segment(NodeGetOffsetVelLevel(), 3) += F.eigen();
}

void ChNodePeri::ContactComputeQ(const ChVector3d& F,
                                 const ChVector3d& T,
                                 const ChVector3d& point,
                                 const ChState& state_x,
                                 ChVectorDynamic<>& Q,
                                 int offset) {
    Q.segment(offset, 3) = F.eigen();
    Q.segment(offset + 3, 3) = VNULL.eigen();
}

void ChNodePeri::ComputeJacobianForContactPart(const ChVector3d& abs_point,
                                               ChMatrix33<>& contact_plane,
                                               ChConstraintTuple* jacobian_tuple_N,
                                               ChConstraintTuple* jacobian_tuple_U,
                                               ChConstraintTuple* jacobian_tuple_V,
                                               bool second) {
    ChMatrix33<> Jx1 = contact_plane.transpose();
    if (!second)
        Jx1 *= -1;

    auto tuple_N = static_cast<ChConstraintTuple_1vars<3>*>(jacobian_tuple_N);
    auto tuple_U = static_cast<ChConstraintTuple_1vars<3>*>(jacobian_tuple_U);
    auto tuple_V = static_cast<ChConstraintTuple_1vars<3>*>(jacobian_tuple_V);

    tuple_N->Cq1().segment(0, 3) = Jx1.row(0);
    tuple_U->Cq1().segment(0, 3) = Jx1.row(1);
    tuple_V->Cq1().segment(0, 3) = Jx1.row(2);
}

}  // end namespace chrono
