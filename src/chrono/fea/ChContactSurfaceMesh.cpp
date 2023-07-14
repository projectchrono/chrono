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

#include "chrono/collision/ChCollisionModelBullet.h"
#include "chrono/core/ChMath.h"
#include "chrono/physics/ChSystem.h"

#include "chrono/fea/ChContactSurfaceMesh.h"
#include "chrono/fea/ChElementShellANCF_3423.h"
#include "chrono/fea/ChElementShellANCF_3443.h"
#include "chrono/fea/ChElementShellANCF_3833.h"
#include "chrono/fea/ChElementShellReissner4.h"
#include "chrono/fea/ChElementShellBST.h"
#include "chrono/fea/ChElementCableANCF.h"
#include "chrono/fea/ChElementBeamANCF_3243.h"
#include "chrono/fea/ChElementBeamANCF_3333.h"
#include "chrono/fea/ChElementBeamEuler.h"
#include "chrono/fea/ChHexahedronFace.h"
#include "chrono/fea/ChTetrahedronFace.h"
#include "chrono/fea/ChMesh.h"

#include <unordered_map>
#include <map>
#include <set>
#include <array>
#include <algorithm>

namespace chrono {
namespace fea {

// =============================================================================

ChContactTriangleXYZ::ChContactTriangleXYZ() : m_owns_node({true, true, true}), m_owns_edge({true, true, true}) {
    m_collision_model = new collision::ChCollisionModelBullet;
    m_collision_model->SetContactable(this);
}

ChContactTriangleXYZ::ChContactTriangleXYZ(const std::array<std::shared_ptr<ChNodeFEAxyz>, 3>& nodes,
                                           ChContactSurface* container)
    : m_nodes(nodes), m_container(container), m_owns_node({true, true, true}), m_owns_edge({true, true, true}) {
    m_collision_model = new collision::ChCollisionModelBullet;
    m_collision_model->SetContactable(this);
}

ChPhysicsItem* ChContactTriangleXYZ::GetPhysicsItem() {
    return m_container->GetPhysicsItem();
}

// interface to ChLoadableUV

// Gets all the DOFs packed in a single vector (position part).
void ChContactTriangleXYZ::LoadableGetStateBlock_x(int block_offset, ChState& mD) {
    mD.segment(block_offset + 0, 3) = m_nodes[0]->GetPos().eigen();
    mD.segment(block_offset + 3, 3) = m_nodes[1]->GetPos().eigen();
    mD.segment(block_offset + 6, 3) = m_nodes[2]->GetPos().eigen();
}

// Gets all the DOFs packed in a single vector (velocity part).
void ChContactTriangleXYZ::LoadableGetStateBlock_w(int block_offset, ChStateDelta& mD) {
    mD.segment(block_offset + 0, 3) = m_nodes[0]->GetPos_dt().eigen();
    mD.segment(block_offset + 3, 3) = m_nodes[1]->GetPos_dt().eigen();
    mD.segment(block_offset + 6, 3) = m_nodes[2]->GetPos_dt().eigen();
}

/// Increment all DOFs using a delta.
void ChContactTriangleXYZ::LoadableStateIncrement(const unsigned int off_x,
                                                  ChState& x_new,
                                                  const ChState& x,
                                                  const unsigned int off_v,
                                                  const ChStateDelta& Dv) {
    m_nodes[0]->NodeIntStateIncrement(off_x + 0, x_new, x, off_v, Dv);
    m_nodes[1]->NodeIntStateIncrement(off_x + 3, x_new, x, off_v + 3, Dv);
    m_nodes[2]->NodeIntStateIncrement(off_x + 6, x_new, x, off_v + 6, Dv);
}
// Get the pointers to the contained ChVariables, appending to the mvars vector.
void ChContactTriangleXYZ::LoadableGetVariables(std::vector<ChVariables*>& mvars) {
    mvars.push_back(&m_nodes[0]->Variables());
    mvars.push_back(&m_nodes[1]->Variables());
    mvars.push_back(&m_nodes[2]->Variables());
}

void ChContactTriangleXYZ::ContactableGetStateBlock_x(ChState& x) {
    x.segment(0, 3) = m_nodes[0]->pos.eigen();
    x.segment(3, 3) = m_nodes[1]->pos.eigen();
    x.segment(6, 3) = m_nodes[2]->pos.eigen();
}

void ChContactTriangleXYZ::ContactableGetStateBlock_w(ChStateDelta& w) {
    w.segment(0, 3) = m_nodes[0]->pos_dt.eigen();
    w.segment(3, 3) = m_nodes[1]->pos_dt.eigen();
    w.segment(6, 3) = m_nodes[2]->pos_dt.eigen();
}

void ChContactTriangleXYZ::ContactableIncrementState(const ChState& x, const ChStateDelta& dw, ChState& x_new) {
    m_nodes[0]->NodeIntStateIncrement(0, x_new, x, 0, dw);
    m_nodes[1]->NodeIntStateIncrement(3, x_new, x, 3, dw);
    m_nodes[2]->NodeIntStateIncrement(6, x_new, x, 6, dw);
}

ChVector<> ChContactTriangleXYZ::GetContactPoint(const ChVector<>& loc_point, const ChState& state_x) {
    // Note: because the reference coordinate system for a ChcontactTriangleXYZ is the identity,
    // the given point loc_point is actually expressed in the global frame. In this case, we
    // calculate the output point here by assuming that its barycentric coordinates do not change
    // with a change in the states of this object.
    double s2, s3;
    ComputeUVfromP(loc_point, s2, s3);
    double s1 = 1 - s2 - s3;

    ChVector<> A1(state_x.segment(0, 3));
    ChVector<> A2(state_x.segment(3, 3));
    ChVector<> A3(state_x.segment(6, 3));

    return s1 * A1 + s2 * A2 + s3 * A3;
}

ChVector<> ChContactTriangleXYZ::GetContactPointSpeed(const ChVector<>& loc_point,
                                                      const ChState& state_x,
                                                      const ChStateDelta& state_w) {
    // Note: because the reference coordinate system for a ChcontactTriangleXYZ is the identity,
    // the given point loc_point is actually expressed in the global frame. In this case, we
    // calculate the output point here by assuming that its barycentric coordinates do not change
    // with a change in the states of this object.
    double s2, s3;
    ComputeUVfromP(loc_point, s2, s3);
    double s1 = 1 - s2 - s3;

    ChVector<> A1_dt(state_w.segment(0, 3));
    ChVector<> A2_dt(state_w.segment(3, 3));
    ChVector<> A3_dt(state_w.segment(6, 3));

    return s1 * A1_dt + s2 * A2_dt + s3 * A3_dt;
}

ChVector<> ChContactTriangleXYZ::GetContactPointSpeed(const ChVector<>& abs_point) {
    double s2, s3;
    ComputeUVfromP(abs_point, s2, s3);
    double s1 = 1 - s2 - s3;
    return (s1 * m_nodes[0]->pos_dt + s2 * m_nodes[1]->pos_dt + s3 * m_nodes[2]->pos_dt);
}

void ChContactTriangleXYZ::ContactForceLoadResidual_F(const ChVector<>& F,
                                                      const ChVector<>& abs_point,
                                                      ChVectorDynamic<>& R) {
    double s2, s3;
    ComputeUVfromP(abs_point, s2, s3);
    double s1 = 1 - s2 - s3;
    R.segment(m_nodes[0]->NodeGetOffsetW(), 3) += F.eigen() * s1;
    R.segment(m_nodes[1]->NodeGetOffsetW(), 3) += F.eigen() * s2;
    R.segment(m_nodes[2]->NodeGetOffsetW(), 3) += F.eigen() * s3;
}

void ChContactTriangleXYZ::ContactForceLoadQ(const ChVector<>& F,
                                             const ChVector<>& point,
                                             const ChState& state_x,
                                             ChVectorDynamic<>& Q,
                                             int offset) {
    // Calculate barycentric coordinates
    ChVector<> A1(state_x.segment(0, 3));
    ChVector<> A2(state_x.segment(3, 3));
    ChVector<> A3(state_x.segment(6, 3));

    double s2, s3;
    bool is_into;
    ChVector<> p_projected;
    /*double dist =*/collision::utils::PointTriangleDistance(point, A1, A2, A3, s2, s3, is_into, p_projected);
    double s1 = 1 - s2 - s3;
    Q.segment(offset + 0, 3) = F.eigen() * s1;
    Q.segment(offset + 3, 3) = F.eigen() * s2;
    Q.segment(offset + 6, 3) = F.eigen() * s3;
}

void ChContactTriangleXYZ::ComputeJacobianForContactPart(const ChVector<>& abs_point,
                                                         ChMatrix33<>& contact_plane,
                                                         type_constraint_tuple& jacobian_tuple_N,
                                                         type_constraint_tuple& jacobian_tuple_U,
                                                         type_constraint_tuple& jacobian_tuple_V,
                                                         bool second) {
    // compute the triangular area-parameters s1 s2 s3:
    double s2, s3;
    bool is_into;
    ChVector<> p_projected;
    /*double dist =*/collision::utils::PointTriangleDistance(abs_point, GetNode(0)->pos, GetNode(1)->pos,
                                                             GetNode(2)->pos, s2, s3, is_into, p_projected);
    double s1 = 1 - s2 - s3;

    ChMatrix33<> Jx1 = contact_plane.transpose();
    if (!second)
        Jx1 *= -1;

    jacobian_tuple_N.Get_Cq_1().segment(0, 3) = Jx1.row(0);
    jacobian_tuple_U.Get_Cq_1().segment(0, 3) = Jx1.row(1);
    jacobian_tuple_V.Get_Cq_1().segment(0, 3) = Jx1.row(2);
    jacobian_tuple_N.Get_Cq_1() *= s1;
    jacobian_tuple_U.Get_Cq_1() *= s1;
    jacobian_tuple_V.Get_Cq_1() *= s1;
    jacobian_tuple_N.Get_Cq_2().segment(0, 3) = Jx1.row(0);
    jacobian_tuple_U.Get_Cq_2().segment(0, 3) = Jx1.row(1);
    jacobian_tuple_V.Get_Cq_2().segment(0, 3) = Jx1.row(2);
    jacobian_tuple_N.Get_Cq_2() *= s2;
    jacobian_tuple_U.Get_Cq_2() *= s2;
    jacobian_tuple_V.Get_Cq_2() *= s2;
    jacobian_tuple_N.Get_Cq_3().segment(0, 3) = Jx1.row(0);
    jacobian_tuple_U.Get_Cq_3().segment(0, 3) = Jx1.row(1);
    jacobian_tuple_V.Get_Cq_3().segment(0, 3) = Jx1.row(2);
    jacobian_tuple_N.Get_Cq_3() *= s3;
    jacobian_tuple_U.Get_Cq_3() *= s3;
    jacobian_tuple_V.Get_Cq_3() *= s3;
}

unsigned int ChContactTriangleXYZ::GetSubBlockOffset(int nblock) {
    if (nblock == 0)
        return GetNode(0)->NodeGetOffsetW();
    if (nblock == 1)
        return GetNode(1)->NodeGetOffsetW();
    if (nblock == 2)
        return GetNode(2)->NodeGetOffsetW();
    return 0;
}

bool ChContactTriangleXYZ::IsSubBlockActive(int nblock) const {
    if (nblock == 0)
        return !GetNode(0)->IsFixed();
    if (nblock == 1)
        return !GetNode(1)->IsFixed();
    if (nblock == 2)
        return !GetNode(2)->IsFixed();

    return false;
}

// Evaluate N'*F , where N is the shape function evaluated at (U,V) coordinates of the surface.
void ChContactTriangleXYZ::ComputeNF(
    const double U,              // parametric coordinate in surface
    const double V,              // parametric coordinate in surface
    ChVectorDynamic<>& Qi,       // Return result of Q = N'*F  here
    double& detJ,                // Return det[J] here
    const ChVectorDynamic<>& F,  // Input F vector, size is =n. field coords.
    ChVectorDynamic<>* state_x,  // if != 0, update state (pos. part) to this, then evaluate Q
    ChVectorDynamic<>* state_w   // if != 0, update state (speed part) to this, then evaluate Q
) {
    ChMatrixNM<double, 1, 3> N;
    // shape functions (U and V in 0..1 as triangle integration)
    N(0) = 1 - U - V;
    N(1) = U;
    N(2) = V;

    // determinant of jacobian is also =2*areaoftriangle, also length of cross product of sides
    ChVector<> p0 = GetNode(0)->GetPos();
    ChVector<> p1 = GetNode(1)->GetPos();
    ChVector<> p2 = GetNode(2)->GetPos();
    detJ = (Vcross(p2 - p0, p1 - p0)).Length();

    Qi.segment(0, 3) = N(0) * F.segment(0, 3);
    Qi.segment(3, 3) = N(1) * F.segment(0, 3);
    Qi.segment(6, 3) = N(2) * F.segment(0, 3);
}

ChVector<> ChContactTriangleXYZ::ComputeNormal(const double U, const double V) {
    ChVector<> p0 = GetNode(0)->GetPos();
    ChVector<> p1 = GetNode(1)->GetPos();
    ChVector<> p2 = GetNode(2)->GetPos();
    return Vcross(p1 - p0, p2 - p0).GetNormalized();
}

void ChContactTriangleXYZ::ComputeUVfromP(const ChVector<> P, double& u, double& v) {
    bool is_into;
    ChVector<> p_projected;
    /*double dist =*/collision::utils::PointTriangleDistance(P, m_nodes[0]->pos, m_nodes[1]->pos, m_nodes[2]->pos, u, v,
                                                             is_into,
                                                             p_projected);
}

// =============================================================================

ChContactTriangleXYZROT::ChContactTriangleXYZROT() : m_owns_node({true, true, true}), m_owns_edge({true, true, true}) {
    m_collision_model = new collision::ChCollisionModelBullet;
    m_collision_model->SetContactable(this);
}

ChContactTriangleXYZROT::ChContactTriangleXYZROT(const std::array<std::shared_ptr<ChNodeFEAxyzrot>, 3>& nodes,
                                                 ChContactSurface* container)
    : m_nodes(nodes), m_container(container), m_owns_node({true, true, true}), m_owns_edge({true, true, true}) {
    m_collision_model = new collision::ChCollisionModelBullet;
    m_collision_model->SetContactable(this);
}

ChPhysicsItem* ChContactTriangleXYZROT::GetPhysicsItem() {
    return m_container->GetPhysicsItem();
}

// interface to ChLoadableUV

// Gets all the DOFs packed in a single vector (position part).
void ChContactTriangleXYZROT::LoadableGetStateBlock_x(int block_offset, ChState& mD) {
    mD.segment(block_offset + 0, 3) = m_nodes[0]->GetPos().eigen();
    mD.segment(block_offset + 3, 4) = m_nodes[0]->GetRot().eigen();

    mD.segment(block_offset + 7, 3) = m_nodes[1]->GetPos().eigen();
    mD.segment(block_offset + 10, 4) = m_nodes[1]->GetRot().eigen();

    mD.segment(block_offset + 14, 3) = m_nodes[2]->GetPos().eigen();
    mD.segment(block_offset + 17, 4) = m_nodes[2]->GetRot().eigen();
}

// Gets all the DOFs packed in a single vector (velocity part).
void ChContactTriangleXYZROT::LoadableGetStateBlock_w(int block_offset, ChStateDelta& mD) {
    mD.segment(block_offset + 0, 3) = m_nodes[0]->GetPos_dt().eigen();
    mD.segment(block_offset + 3, 3) = m_nodes[0]->GetWvel_loc().eigen();

    mD.segment(block_offset + 6, 3) = m_nodes[1]->GetPos_dt().eigen();
    mD.segment(block_offset + 9, 3) = m_nodes[1]->GetWvel_loc().eigen();

    mD.segment(block_offset + 12, 3) = m_nodes[2]->GetPos_dt().eigen();
    mD.segment(block_offset + 15, 3) = m_nodes[2]->GetWvel_loc().eigen();
}

// Increment all DOFs using a delta.
void ChContactTriangleXYZROT::LoadableStateIncrement(const unsigned int off_x,
                                                     ChState& x_new,
                                                     const ChState& x,
                                                     const unsigned int off_v,
                                                     const ChStateDelta& Dv) {
    m_nodes[0]->NodeIntStateIncrement(off_x, x_new, x, off_v, Dv);
    m_nodes[1]->NodeIntStateIncrement(off_x + 7, x_new, x, off_v + 6, Dv);
    m_nodes[2]->NodeIntStateIncrement(off_x + 14, x_new, x, off_v + 12, Dv);
}

// Get the pointers to the contained ChVariables, appending to the mvars vector.
void ChContactTriangleXYZROT::LoadableGetVariables(std::vector<ChVariables*>& mvars) {
    mvars.push_back(&m_nodes[0]->Variables());
    mvars.push_back(&m_nodes[1]->Variables());
    mvars.push_back(&m_nodes[2]->Variables());
}

void ChContactTriangleXYZROT::ContactableGetStateBlock_x(ChState& x) {
    x.segment(0, 3) = m_nodes[0]->GetPos().eigen();
    x.segment(3, 4) = m_nodes[0]->GetRot().eigen();

    x.segment(7, 3) = m_nodes[1]->GetPos().eigen();
    x.segment(10, 4) = m_nodes[1]->GetRot().eigen();

    x.segment(14, 3) = m_nodes[2]->GetPos().eigen();
    x.segment(17, 4) = m_nodes[2]->GetRot().eigen();
}

void ChContactTriangleXYZROT::ContactableGetStateBlock_w(ChStateDelta& w) {
    w.segment(0, 3) = m_nodes[0]->GetPos_dt().eigen();
    w.segment(3, 3) = m_nodes[0]->GetWvel_loc().eigen();

    w.segment(6, 3) = m_nodes[1]->GetPos_dt().eigen();
    w.segment(9, 3) = m_nodes[1]->GetWvel_loc().eigen();

    w.segment(12, 3) = m_nodes[2]->GetPos_dt().eigen();
    w.segment(15, 3) = m_nodes[2]->GetWvel_loc().eigen();
}

void ChContactTriangleXYZROT::ContactableIncrementState(const ChState& x, const ChStateDelta& dw, ChState& x_new) {
    m_nodes[0]->NodeIntStateIncrement(0, x_new, x, 0, dw);
    m_nodes[1]->NodeIntStateIncrement(7, x_new, x, 6, dw);
    m_nodes[2]->NodeIntStateIncrement(14, x_new, x, 12, dw);
}

ChVector<> ChContactTriangleXYZROT::GetContactPoint(const ChVector<>& loc_point, const ChState& state_x) {
    // Note: because the reference coordinate system for a ChContactTriangleXYZROT is the identity,
    // the given point loc_point is actually expressed in the global frame. In this case, we
    // calculate the output point here by assuming that its barycentric coordinates do not change
    // with a change in the states of this object.
    double s2, s3;
    ComputeUVfromP(loc_point, s2, s3);
    double s1 = 1 - s2 - s3;

    ChVector<> A1(state_x.segment(0, 3));
    ChVector<> A2(state_x.segment(7, 3));
    ChVector<> A3(state_x.segment(14, 3));

    return s1 * A1 + s2 * A2 + s3 * A3;
}

ChVector<> ChContactTriangleXYZROT::GetContactPointSpeed(const ChVector<>& loc_point,
                                                         const ChState& state_x,
                                                         const ChStateDelta& state_w) {
    // Note: because the reference coordinate system for a ChContactTriangleXYZROT is the identity,
    // the given point loc_point is actually expressed in the global frame. In this case, we
    // calculate the output point here by assuming that its barycentric coordinates do not change
    // with a change in the states of this object.
    double s2, s3;
    ComputeUVfromP(loc_point, s2, s3);
    double s1 = 1 - s2 - s3;

    ChVector<> A1_dt(state_w.segment(0, 3));
    ChVector<> A2_dt(state_w.segment(6, 3));
    ChVector<> A3_dt(state_w.segment(12, 3));

    return s1 * A1_dt + s2 * A2_dt + s3 * A3_dt;
}

ChVector<> ChContactTriangleXYZROT::GetContactPointSpeed(const ChVector<>& abs_point) {
    double s2, s3;
    ComputeUVfromP(abs_point, s2, s3);
    double s1 = 1 - s2 - s3;
    return (s1 * m_nodes[0]->GetPos_dt() + s2 * m_nodes[1]->GetPos_dt() + s3 * m_nodes[2]->GetPos_dt());
}

void ChContactTriangleXYZROT::ContactForceLoadResidual_F(const ChVector<>& F,
                                                         const ChVector<>& abs_point,
                                                         ChVectorDynamic<>& R) {
    double s2, s3;
    ComputeUVfromP(abs_point, s2, s3);
    double s1 = 1 - s2 - s3;
    R.segment(m_nodes[0]->NodeGetOffsetW(), 3) += F.eigen() * s1;
    R.segment(m_nodes[1]->NodeGetOffsetW(), 3) += F.eigen() * s2;
    R.segment(m_nodes[2]->NodeGetOffsetW(), 3) += F.eigen() * s3;
}

void ChContactTriangleXYZROT::ContactForceLoadQ(const ChVector<>& F,
                                                const ChVector<>& point,
                                                const ChState& state_x,
                                                ChVectorDynamic<>& Q,
                                                int offset) {
    // Calculate barycentric coordinates
    ChVector<> A1(state_x.segment(0, 3));
    ChVector<> A2(state_x.segment(7, 3));
    ChVector<> A3(state_x.segment(14, 3));

    double s2, s3;
    bool is_into;
    ChVector<> p_projected;
    /*double dist =*/collision::utils::PointTriangleDistance(point, A1, A2, A3, s2, s3, is_into, p_projected);
    double s1 = 1 - s2 - s3;
    Q.segment(offset + 0, 3) = F.eigen() * s1;
    Q.segment(offset + 6, 3) = F.eigen() * s2;
    Q.segment(offset + 12, 3) = F.eigen() * s3;
}

void ChContactTriangleXYZROT::ComputeJacobianForContactPart(const ChVector<>& abs_point,
                                                            ChMatrix33<>& contact_plane,
                                                            type_constraint_tuple& jacobian_tuple_N,
                                                            type_constraint_tuple& jacobian_tuple_U,
                                                            type_constraint_tuple& jacobian_tuple_V,
                                                            bool second) {
    // compute the triangular area-parameters s1 s2 s3:
    double s2, s3;
    bool is_into;
    ChVector<> p_projected;
    /*double dist =*/collision::utils::PointTriangleDistance(abs_point, GetNode(0)->coord.pos, GetNode(1)->coord.pos,
                                                             GetNode(2)->coord.pos, s2, s3, is_into, p_projected);
    double s1 = 1 - s2 - s3;

    ChMatrix33<> Jx1 = contact_plane.transpose();
    if (!second)
        Jx1 *= -1;

    jacobian_tuple_N.Get_Cq_1().segment(0, 3) = Jx1.row(0);
    jacobian_tuple_U.Get_Cq_1().segment(0, 3) = Jx1.row(1);
    jacobian_tuple_V.Get_Cq_1().segment(0, 3) = Jx1.row(2);
    jacobian_tuple_N.Get_Cq_1() *= s1;
    jacobian_tuple_U.Get_Cq_1() *= s1;
    jacobian_tuple_V.Get_Cq_1() *= s1;
    jacobian_tuple_N.Get_Cq_2().segment(0, 3) = Jx1.row(0);
    jacobian_tuple_U.Get_Cq_2().segment(0, 3) = Jx1.row(1);
    jacobian_tuple_V.Get_Cq_2().segment(0, 3) = Jx1.row(2);
    jacobian_tuple_N.Get_Cq_2() *= s2;
    jacobian_tuple_U.Get_Cq_2() *= s2;
    jacobian_tuple_V.Get_Cq_2() *= s2;
    jacobian_tuple_N.Get_Cq_3().segment(0, 3) = Jx1.row(0);
    jacobian_tuple_U.Get_Cq_3().segment(0, 3) = Jx1.row(1);
    jacobian_tuple_V.Get_Cq_3().segment(0, 3) = Jx1.row(2);
    jacobian_tuple_N.Get_Cq_3() *= s3;
    jacobian_tuple_U.Get_Cq_3() *= s3;
    jacobian_tuple_V.Get_Cq_3() *= s3;
}

unsigned int ChContactTriangleXYZROT::GetSubBlockOffset(int nblock) {
    if (nblock == 0)
        return GetNode(0)->NodeGetOffsetW();
    if (nblock == 1)
        return GetNode(1)->NodeGetOffsetW();
    if (nblock == 2)
        return GetNode(2)->NodeGetOffsetW();
    return 0;
}

bool ChContactTriangleXYZROT::IsSubBlockActive(int nblock) const {
    if (nblock == 0)
        return !GetNode(0)->IsFixed();
    if (nblock == 1)
        return !GetNode(1)->IsFixed();
    if (nblock == 2)
        return !GetNode(2)->IsFixed();

    return false;
}

// Evaluate N'*F , where N is the shape function evaluated at (U,V) coordinates of the surface.
void ChContactTriangleXYZROT::ComputeNF(
    const double U,              // parametric coordinate in surface
    const double V,              // parametric coordinate in surface
    ChVectorDynamic<>& Qi,       // Return result of Q = N'*F  here
    double& detJ,                // Return det[J] here
    const ChVectorDynamic<>& F,  // Input F vector, size is =n. field coords.
    ChVectorDynamic<>* state_x,  // if != 0, update state (pos. part) to this, then evaluate Q
    ChVectorDynamic<>* state_w   // if != 0, update state (speed part) to this, then evaluate Q
) {
    ChMatrixNM<double, 1, 3> N;
    // shape functions (U and V in 0..1 as triangle integration)
    N(0) = 1 - U - V;
    N(1) = U;
    N(2) = V;

    // determinant of jacobian is also =2*areaoftriangle, also length of cross product of sides
    ChVector<> p0 = GetNode(0)->GetPos();
    ChVector<> p1 = GetNode(1)->GetPos();
    ChVector<> p2 = GetNode(2)->GetPos();
    detJ = (Vcross(p2 - p0, p1 - p0)).Length();

    Qi.segment(0, 3) = N(0) * F.segment(0, 3);
    Qi.segment(3, 3) = N(0) * F.segment(3, 3);

    Qi.segment(6, 3) = N(1) * F.segment(0, 3);
    Qi.segment(9, 3) = N(1) * F.segment(3, 3);

    Qi.segment(12, 3) = N(2) * F.segment(0, 3);
    Qi.segment(15, 3) = N(2) * F.segment(3, 3);
}

ChVector<> ChContactTriangleXYZROT::ComputeNormal(const double U, const double V) {
    ChVector<> p0 = GetNode(0)->GetPos();
    ChVector<> p1 = GetNode(1)->GetPos();
    ChVector<> p2 = GetNode(2)->GetPos();
    return Vcross(p1 - p0, p2 - p0).GetNormalized();
}

void ChContactTriangleXYZROT::ComputeUVfromP(const ChVector<> P, double& u, double& v) {
    bool is_into;
    ChVector<> p_projected;
    /*double dist =*/collision::utils::PointTriangleDistance(P, m_nodes[0]->GetPos(), m_nodes[1]->GetPos(),
                                                             m_nodes[2]->GetPos(), u, v, is_into, p_projected);
}

// =============================================================================

ChContactSurfaceMesh::ChContactSurfaceMesh(std::shared_ptr<ChMaterialSurface> material, ChMesh* mesh)
    : ChContactSurface(material, mesh) {}

void ChContactSurfaceMesh::AddFace(std::shared_ptr<ChNodeFEAxyz> node1,
                                   std::shared_ptr<ChNodeFEAxyz> node2,
                                   std::shared_ptr<ChNodeFEAxyz> node3,
                                   std::shared_ptr<ChNodeFEAxyz> edge_node1,
                                   std::shared_ptr<ChNodeFEAxyz> edge_node2,
                                   std::shared_ptr<ChNodeFEAxyz> edge_node3,
                                   bool owns_node1,
                                   bool owns_node2,
                                   bool owns_node3,
                                   bool owns_edge1,
                                   bool owns_edge2,
                                   bool owns_edge3,
                                   double sphere_swept) {
    assert(node1);
    assert(node2);
    assert(node3);

    auto contact_triangle = chrono_types::make_shared<ChContactTriangleXYZ>();
    contact_triangle->SetNodes({{node1, node2, node3}});
    contact_triangle->SetNodeOwnership({owns_node1, owns_node2, owns_node3});
    contact_triangle->SetEdgeOwnership({owns_edge1, owns_edge2, owns_edge3});
    contact_triangle->SetContactSurface(this);

    auto collision_model = static_cast<collision::ChCollisionModelBullet*>(contact_triangle->GetCollisionModel());
    collision_model->ClearModel();
    collision_model->AddTriangleProxy(m_material,                                   // contact material
                                      &node1->pos, &node2->pos, &node3->pos,        // face nodes
                                      edge_node1 ? &edge_node1->pos : &node1->pos,  // edge node 1
                                      edge_node2 ? &edge_node2->pos : &node2->pos,  // edge node 2
                                      edge_node3 ? &edge_node3->pos : &node3->pos,  // edge node 3
                                      owns_node1, owns_node2, owns_node3,           // face owns nodes?
                                      owns_edge1, owns_edge2, owns_edge3,           // face owns edges?
                                      sphere_swept                                  // thickness
    );
    contact_triangle->GetCollisionModel()->BuildModel();

    vfaces.push_back(contact_triangle);
}

void ChContactSurfaceMesh::ConstructFromTrimesh(std::shared_ptr<geometry::ChTriangleMeshConnected> trimesh,
                                                double sphere_swept) {
    std::vector<std::shared_ptr<fea::ChNodeFEAxyz>> nodes;
    for (const auto& v : trimesh->getCoordsVertices()) {
        nodes.push_back(chrono_types::make_shared<fea::ChNodeFEAxyz>(v));
    }

    std::vector<NodeTripletXYZ> triangles_ptrs;
    for (const auto& tri : trimesh->getIndicesVertexes()) {
        const auto& node0 = nodes[tri[0]];
        const auto& node1 = nodes[tri[1]];
        const auto& node2 = nodes[tri[2]];
        triangles_ptrs.push_back({{node0, node1, node2}});
    }

    AddFacesFromTripletsXYZ(triangles_ptrs, sphere_swept);
}

void ChContactSurfaceMesh::AddFacesFromBoundary(double sphere_swept, bool ccw) {
    if (!m_physics_item)
        return;
    auto mesh = dynamic_cast<ChMesh*>(m_physics_item);
    if (!mesh)
        return;

    std::vector<std::array<std::shared_ptr<ChNodeFEAxyz>, 3>> triangles_ptrs;
    std::vector<std::array<std::shared_ptr<ChNodeFEAxyzrot>, 3>> triangles_rot_ptrs;

    // Boundary faces of TETRAHEDRONS
    std::multimap<std::array<ChNodeFEAxyz*, 3>, ChTetrahedronFace> face_map;

    for (unsigned int ie = 0; ie < mesh->GetNelements(); ++ie) {
        if (auto mtetra = std::dynamic_pointer_cast<ChElementTetrahedron>(mesh->GetElement(ie))) {
            for (int nface = 0; nface < 4; ++nface) {
                ChTetrahedronFace mface(mtetra, nface);
                std::array<ChNodeFEAxyz*, 3> mface_key = {mface.GetNodeN(0).get(), mface.GetNodeN(1).get(),
                                                          mface.GetNodeN(2).get()};
                std::sort(mface_key.begin(), mface_key.end());
                face_map.insert({mface_key, mface});
            }
        }
    }
    for (unsigned int ie = 0; ie < mesh->GetNelements(); ++ie) {
        if (auto mtetra = std::dynamic_pointer_cast<ChElementTetrahedron>(mesh->GetElement(ie))) {
            for (int nface = 0; nface < 4; ++nface) {
                ChTetrahedronFace mface(mtetra, nface);
                std::array<ChNodeFEAxyz*, 3> mface_key = {mface.GetNodeN(0).get(), mface.GetNodeN(1).get(),
                                                          mface.GetNodeN(2).get()};
                std::sort(mface_key.begin(), mface_key.end());
                if (face_map.count(mface_key) == 1) {
                    // Found a face that is not shared.. so it is a boundary face.
                    triangles_ptrs.push_back({{mface.GetNodeN(0), mface.GetNodeN(1), mface.GetNodeN(2)}});
                }
            }
        }
    }

    // Boundary faces of HEXAHEDRONS
    std::multimap<std::array<ChNodeFEAxyz*, 4>, ChHexahedronFace> face_map_brick;

    for (unsigned int ie = 0; ie < mesh->GetNelements(); ++ie) {
        if (auto mbrick = std::dynamic_pointer_cast<ChElementHexahedron>(mesh->GetElement(ie))) {
            for (int nface = 0; nface < 6; ++nface) {
                ChHexahedronFace mface(mbrick, nface);
                std::array<ChNodeFEAxyz*, 4> mface_key = {mface.GetNodeN(0).get(), mface.GetNodeN(1).get(),
                                                          mface.GetNodeN(2).get(), mface.GetNodeN(3).get()};
                std::sort(mface_key.begin(), mface_key.end());
                face_map_brick.insert({mface_key, mface});
            }
        }
    }
    for (unsigned int ie = 0; ie < mesh->GetNelements(); ++ie) {
        if (auto mbrick = std::dynamic_pointer_cast<ChElementHexahedron>(mesh->GetElement(ie))) {
            for (int nface = 0; nface < 6; ++nface) {   // Each of the 6 faces of a brick
                ChHexahedronFace mface(mbrick, nface);  // Create a face of the element
                std::array<ChNodeFEAxyz*, 4> mface_key = {mface.GetNodeN(0).get(), mface.GetNodeN(1).get(),
                                                          mface.GetNodeN(2).get(), mface.GetNodeN(3).get()};
                std::sort(mface_key.begin(), mface_key.end());
                if (face_map_brick.count(mface_key) == 1) {
                    // Found a face that is not shared.. so it is a boundary face: Make two triangles out of that face
                    triangles_ptrs.push_back({{mface.GetNodeN(0), mface.GetNodeN(1), mface.GetNodeN(2)}});
                    triangles_ptrs.push_back({{mface.GetNodeN(0), mface.GetNodeN(2), mface.GetNodeN(3)}});
                }
            }
        }
    }

    // Skin of ANCF SHELLS
    for (unsigned int ie = 0; ie < mesh->GetNelements(); ++ie) {
        if (auto mshell = std::dynamic_pointer_cast<ChElementShellANCF_3423>(mesh->GetElement(ie))) {
            std::shared_ptr<ChNodeFEAxyz> nA = mshell->GetNodeA();
            std::shared_ptr<ChNodeFEAxyz> nB = mshell->GetNodeB();
            std::shared_ptr<ChNodeFEAxyz> nC = mshell->GetNodeC();
            std::shared_ptr<ChNodeFEAxyz> nD = mshell->GetNodeD();
            if (ccw) {
                triangles_ptrs.push_back({{nA, nD, nB}});
                triangles_ptrs.push_back({{nB, nD, nC}});
            } else {
                triangles_ptrs.push_back({{nA, nB, nD}});
                triangles_ptrs.push_back({{nB, nC, nD}});
            }
        }
    }

    for (unsigned int ie = 0; ie < mesh->GetNelements(); ++ie) {
        if (auto mshell = std::dynamic_pointer_cast<ChElementShellANCF_3443>(mesh->GetElement(ie))) {
            std::shared_ptr<ChNodeFEAxyz> nA = mshell->GetNodeA();
            std::shared_ptr<ChNodeFEAxyz> nB = mshell->GetNodeB();
            std::shared_ptr<ChNodeFEAxyz> nC = mshell->GetNodeC();
            std::shared_ptr<ChNodeFEAxyz> nD = mshell->GetNodeD();
            if (ccw) {
                triangles_ptrs.push_back({{nA, nD, nB}});
                triangles_ptrs.push_back({{nB, nD, nC}});
            } else {
                triangles_ptrs.push_back({{nA, nB, nD}});
                triangles_ptrs.push_back({{nB, nC, nD}});
            }
        }
    }

    for (unsigned int ie = 0; ie < mesh->GetNelements(); ++ie) {
        if (auto mshell = std::dynamic_pointer_cast<ChElementShellANCF_3833>(mesh->GetElement(ie))) {
            auto nA = mshell->GetNodeA();
            auto nB = mshell->GetNodeB();
            auto nC = mshell->GetNodeC();
            auto nD = mshell->GetNodeD();
            auto nE = mshell->GetNodeE();
            auto nF = mshell->GetNodeF();
            auto nG = mshell->GetNodeG();
            auto nH = mshell->GetNodeH();
            if (ccw) {
                triangles_ptrs.push_back({{nA, nH, nE}});
                triangles_ptrs.push_back({{nB, nE, nF}});
                triangles_ptrs.push_back({{nC, nF, nG}});
                triangles_ptrs.push_back({{nD, nG, nH}});
                triangles_ptrs.push_back({{nH, nG, nE}});
                triangles_ptrs.push_back({{nF, nE, nG}});
            } else {
                triangles_ptrs.push_back({{nA, nE, nH}});
                triangles_ptrs.push_back({{nB, nF, nE}});
                triangles_ptrs.push_back({{nC, nG, nF}});
                triangles_ptrs.push_back({{nD, nH, nG}});
                triangles_ptrs.push_back({{nH, nE, nG}});
                triangles_ptrs.push_back({{nF, nG, nE}});
            }
        }
    }

    // Skin of REISSNER SHELLS:
    for (unsigned int ie = 0; ie < mesh->GetNelements(); ++ie) {
        if (auto mshell = std::dynamic_pointer_cast<ChElementShellReissner4>(mesh->GetElement(ie))) {
            std::shared_ptr<ChNodeFEAxyzrot> nA = mshell->GetNodeA();
            std::shared_ptr<ChNodeFEAxyzrot> nB = mshell->GetNodeB();
            std::shared_ptr<ChNodeFEAxyzrot> nC = mshell->GetNodeC();
            std::shared_ptr<ChNodeFEAxyzrot> nD = mshell->GetNodeD();
            if (ccw) {
                triangles_rot_ptrs.push_back({{nA, nD, nB}});
                triangles_rot_ptrs.push_back({{nB, nD, nC}});
            } else {
                triangles_rot_ptrs.push_back({{nA, nB, nD}});
                triangles_rot_ptrs.push_back({{nB, nC, nD}});
            }
        }
    }

    // Skin of BST shells
    for (unsigned int ie = 0; ie < mesh->GetNelements(); ++ie) {
        if (auto mshell = std::dynamic_pointer_cast<ChElementShellBST>(mesh->GetElement(ie))) {
            auto n0 = mshell->GetNodeTriangleN(0);
            auto n1 = mshell->GetNodeTriangleN(1);
            auto n2 = mshell->GetNodeTriangleN(2);
            if (ccw) {
                triangles_ptrs.push_back({{n0, n1, n2}});
            } else {
                triangles_ptrs.push_back({{n0, n2, n1}});
            }
        }
    }

    // EULER BEAMS (handles as a skinny triangle, with sphere swept radii, i.e. a capsule):
    for (unsigned int ie = 0; ie < mesh->GetNelements(); ++ie) {
        if (auto mbeam = std::dynamic_pointer_cast<ChElementBeamEuler>(mesh->GetElement(ie))) {
            std::shared_ptr<ChNodeFEAxyzrot> nA = mbeam->GetNodeA();
            std::shared_ptr<ChNodeFEAxyzrot> nB = mbeam->GetNodeB();

            auto contact_triangle = chrono_types::make_shared<ChContactTriangleXYZROT>();
            contact_triangle->SetNodes({{nA, nB, nB}});
            contact_triangle->SetNodeOwnership({false, false, false});
            contact_triangle->SetEdgeOwnership({true, false, true});
            contact_triangle->SetContactSurface(this);
            vfaces_rot.push_back(contact_triangle);

            double capsule_radius =
                collision::ChCollisionModel::GetDefaultSuggestedMargin();  // fallback for no draw profile
            if (auto mdrawshape = mbeam->GetSection()->GetDrawShape()) {
                double ymin, ymax, zmin, zmax;
                mdrawshape->GetAABB(ymin, ymax, zmin, zmax);
                capsule_radius = 0.5 * sqrt(pow(ymax - ymin, 2) + pow(zmax - zmin, 2));
            }

            contact_triangle->GetCollisionModel()->ClearModel();
            ((collision::ChCollisionModelBullet*)contact_triangle->GetCollisionModel())
                ->AddTriangleProxy(m_material,                                      // contact material
                                   &nA->coord.pos, &nB->coord.pos, &nB->coord.pos,  // vertices
                                   0, 0, 0,                                         // no wing vertexes
                                   false, false, false,  // are vertexes owned by this triangle?
                                   true, false, true,    // are edges owned by this triangle?
                                   capsule_radius);
            contact_triangle->GetCollisionModel()->BuildModel();
        }
    }

    // ANCF BEAMS (handled as a skinny triangle, with sphere swept radii, i.e. a capsule):
    for (unsigned int ie = 0; ie < mesh->GetNelements(); ++ie) {
        if (auto cableANCF = std::dynamic_pointer_cast<ChElementCableANCF>(mesh->GetElement(ie))) {
            std::shared_ptr<ChNodeFEAxyzD> nA = cableANCF->GetNodeA();
            std::shared_ptr<ChNodeFEAxyzD> nB = cableANCF->GetNodeB();

            auto contact_triangle = chrono_types::make_shared<ChContactTriangleXYZ>();
            contact_triangle->SetNodes({{nA, nB, nB}});
            contact_triangle->SetNodeOwnership({false, false, false});
            contact_triangle->SetEdgeOwnership({true, false, true});
            contact_triangle->SetContactSurface(this);
            vfaces.push_back(contact_triangle);

            double capsule_radius =
                collision::ChCollisionModel::GetDefaultSuggestedMargin();  // fallback for no draw profile
            if (auto mdrawshape = cableANCF->GetSection()->GetDrawShape()) {
                double ymin, ymax, zmin, zmax;
                mdrawshape->GetAABB(ymin, ymax, zmin, zmax);
                capsule_radius = 0.5 * sqrt(pow(ymax - ymin, 2) + pow(zmax - zmin, 2));
            }

            contact_triangle->GetCollisionModel()->ClearModel();
            ((collision::ChCollisionModelBullet*)contact_triangle->GetCollisionModel())
                ->AddTriangleProxy(m_material,                    // contact materials
                                   &nA->pos, &nB->pos, &nB->pos,  // vertices
                                   0, 0, 0,                       // no wing vertexes
                                   false, false, false,           // are vertexes owned by this triangle?
                                   true, false, true,             // are edges owned by this triangle?
                                   capsule_radius);
            contact_triangle->GetCollisionModel()->BuildModel();
        } else if (auto beam3243 = std::dynamic_pointer_cast<ChElementBeamANCF_3243>(mesh->GetElement(ie))) {
            std::shared_ptr<ChNodeFEAxyzD> nA = beam3243->GetNodeA();
            std::shared_ptr<ChNodeFEAxyzD> nB = beam3243->GetNodeB();

            auto contact_triangle = chrono_types::make_shared<ChContactTriangleXYZ>();
            contact_triangle->SetNodes({{nA, nB, nB}});
            contact_triangle->SetNodeOwnership({false, false, false});
            contact_triangle->SetEdgeOwnership({true, false, true});
            contact_triangle->SetContactSurface(this);
            vfaces.push_back(contact_triangle);

            double capsule_radius = 0.5 * sqrt(pow(beam3243->GetThicknessY(), 2) + pow(beam3243->GetThicknessZ(), 2));

            contact_triangle->GetCollisionModel()->ClearModel();
            ((collision::ChCollisionModelBullet*)contact_triangle->GetCollisionModel())
                ->AddTriangleProxy(m_material,                    // contact materials
                                   &nA->pos, &nB->pos, &nB->pos,  // vertices
                                   0, 0, 0,                       // no wing vertexes
                                   false, false, false,           // are vertexes owned by this triangle?
                                   true, false, true,             // are edges owned by this triangle?
                                   capsule_radius);
            contact_triangle->GetCollisionModel()->BuildModel();
        } else if (auto beam3333 = std::dynamic_pointer_cast<ChElementBeamANCF_3333>(mesh->GetElement(ie))) {
            std::shared_ptr<ChNodeFEAxyzD> nA = beam3333->GetNodeA();
            std::shared_ptr<ChNodeFEAxyzD> nB = beam3333->GetNodeB();

            auto contact_triangle = chrono_types::make_shared<ChContactTriangleXYZ>();
            contact_triangle->SetNodes({{nA, nB, nB}});
            contact_triangle->SetNodeOwnership({false, false, false});
            contact_triangle->SetEdgeOwnership({true, false, true});
            contact_triangle->SetContactSurface(this);
            vfaces.push_back(contact_triangle);

            double capsule_radius = 0.5 * sqrt(pow(beam3333->GetThicknessY(), 2) + pow(beam3333->GetThicknessZ(), 2));

            contact_triangle->GetCollisionModel()->ClearModel();
            ((collision::ChCollisionModelBullet*)contact_triangle->GetCollisionModel())
                ->AddTriangleProxy(m_material,                    // contact materials
                                   &nA->pos, &nB->pos, &nB->pos,  // vertices
                                   0, 0, 0,                       // no wing vertexes
                                   false, false, false,           // are vertexes owned by this triangle?
                                   true, false, true,             // are edges owned by this triangle?
                                   capsule_radius);
            contact_triangle->GetCollisionModel()->BuildModel();
        }
    }

    // Create collision triangles from node triplets
    AddFacesFromTripletsXYZ(triangles_ptrs, sphere_swept);
    AddFacesFromTripletsXYZrot(triangles_rot_ptrs, sphere_swept);
}

void ChContactSurfaceMesh::AddFacesFromTripletsXYZ(const std::vector<NodeTripletXYZ>& triangles_ptrs,
                                                   double sphere_swept) {
    std::vector<std::array<ChNodeFEAxyz*, 3>> triangles;
    for (const auto& tri : triangles_ptrs) {
        triangles.push_back({{tri[0].get(), tri[1].get(), tri[2].get()}});
    }

    // Compute triangles connectivity
    std::multimap<std::pair<ChNodeFEAxyz*, ChNodeFEAxyz*>, int> edge_map;

    for (int it = 0; it < triangles.size(); ++it) {
        // edges = pairs of vertexes indexes
        std::pair<ChNodeFEAxyz*, ChNodeFEAxyz*> medgeA(triangles[it][0], triangles[it][1]);
        std::pair<ChNodeFEAxyz*, ChNodeFEAxyz*> medgeB(triangles[it][1], triangles[it][2]);
        std::pair<ChNodeFEAxyz*, ChNodeFEAxyz*> medgeC(triangles[it][2], triangles[it][0]);
        // vertex indexes in edges: always in increasing order to avoid ambiguous duplicated edges
        if (medgeA.first > medgeA.second)
            medgeA = std::pair<ChNodeFEAxyz*, ChNodeFEAxyz*>(medgeA.second, medgeA.first);
        if (medgeB.first > medgeB.second)
            medgeB = std::pair<ChNodeFEAxyz*, ChNodeFEAxyz*>(medgeB.second, medgeB.first);
        if (medgeC.first > medgeC.second)
            medgeC = std::pair<ChNodeFEAxyz*, ChNodeFEAxyz*>(medgeC.second, medgeC.first);
        edge_map.insert({medgeA, it});
        edge_map.insert({medgeB, it});
        edge_map.insert({medgeC, it});
    }

    // Create a map of neighboring triangles, vector of:
    // [Ti TieA TieB TieC]
    std::vector<std::array<int, 4>> tri_map;
    tri_map.resize(triangles.size());

    for (int it = 0; it < triangles.size(); ++it) {
        tri_map[it][0] = it;
        tri_map[it][1] = -1;  // default no neighbor
        tri_map[it][2] = -1;  // default no neighbor
        tri_map[it][3] = -1;  // default no neighbor
        // edges = pairs of vertexes indexes
        std::pair<ChNodeFEAxyz*, ChNodeFEAxyz*> medgeA(triangles[it][0], triangles[it][1]);
        std::pair<ChNodeFEAxyz*, ChNodeFEAxyz*> medgeB(triangles[it][1], triangles[it][2]);
        std::pair<ChNodeFEAxyz*, ChNodeFEAxyz*> medgeC(triangles[it][2], triangles[it][0]);
        // vertex indexes in edges: always in increasing order to avoid ambiguous duplicated edges
        if (medgeA.first > medgeA.second)
            medgeA = std::pair<ChNodeFEAxyz*, ChNodeFEAxyz*>(medgeA.second, medgeA.first);
        if (medgeB.first > medgeB.second)
            medgeB = std::pair<ChNodeFEAxyz*, ChNodeFEAxyz*>(medgeB.second, medgeB.first);
        if (medgeC.first > medgeC.second)
            medgeC = std::pair<ChNodeFEAxyz*, ChNodeFEAxyz*>(medgeC.second, medgeC.first);
        auto retA = edge_map.equal_range(medgeA);
        for (auto fedge = retA.first; fedge != retA.second; ++fedge) {
            if (fedge->second != it) {
                tri_map[it][1] = fedge->second;
                break;
            }
        }
        auto retB = edge_map.equal_range(medgeB);
        for (auto fedge = retB.first; fedge != retB.second; ++fedge) {
            if (fedge->second != it) {
                tri_map[it][2] = fedge->second;
                break;
            }
        }
        auto retC = edge_map.equal_range(medgeC);
        for (auto fedge = retC.first; fedge != retC.second; ++fedge) {
            if (fedge->second != it) {
                tri_map[it][3] = fedge->second;
                break;
            }
        }
    }

    std::map<std::pair<ChNodeFEAxyz*, ChNodeFEAxyz*>, std::pair<int, int>> winged_edges;
    bool allow_single_wing = true;

    for (auto aedge = edge_map.begin(); aedge != edge_map.end(); ++aedge) {
        auto ret = edge_map.equal_range(aedge->first);
        int nt = 0;
        std::pair<ChNodeFEAxyz*, ChNodeFEAxyz*> wingedge;
        std::pair<int, int> wingtri;
        wingtri.first = -1;
        wingtri.second = -1;
        for (auto fedge = ret.first; fedge != ret.second; ++fedge) {
            if (fedge->second == -1)
                break;
            wingedge.first = fedge->first.first;
            wingedge.second = fedge->first.second;
            if (nt == 0)
                wingtri.first = fedge->second;
            if (nt == 1)
                wingtri.second = fedge->second;
            ++nt;
            if (nt == 2)
                break;
        }
        if ((nt == 2) || ((nt == 1) && allow_single_wing)) {
            winged_edges.insert(std::pair<std::pair<ChNodeFEAxyz*, ChNodeFEAxyz*>, std::pair<int, int>>(
                wingedge, wingtri));  // ok found winged edge!
            aedge->second = -1;       // deactivate this way otherwise found again by sister
        }
    }

    // Create triangles with collision models
    std::set<ChNodeFEAxyz*> added_vertexes;

    // iterate on triangles
    for (int it = 0; it < triangles.size(); ++it) {
        // edges = pairs of vertexes indexes
        std::pair<ChNodeFEAxyz*, ChNodeFEAxyz*> medgeA(triangles[it][0], triangles[it][1]);
        std::pair<ChNodeFEAxyz*, ChNodeFEAxyz*> medgeB(triangles[it][1], triangles[it][2]);
        std::pair<ChNodeFEAxyz*, ChNodeFEAxyz*> medgeC(triangles[it][2], triangles[it][0]);
        // vertex indexes in edges: always in increasing order to avoid ambiguous duplicated edges
        if (medgeA.first > medgeA.second)
            medgeA = std::pair<ChNodeFEAxyz*, ChNodeFEAxyz*>(medgeA.second, medgeA.first);
        if (medgeB.first > medgeB.second)
            medgeB = std::pair<ChNodeFEAxyz*, ChNodeFEAxyz*>(medgeB.second, medgeB.first);
        if (medgeC.first > medgeC.second)
            medgeC = std::pair<ChNodeFEAxyz*, ChNodeFEAxyz*>(medgeC.second, medgeC.first);
        auto wingedgeA = winged_edges.find(medgeA);
        auto wingedgeB = winged_edges.find(medgeB);
        auto wingedgeC = winged_edges.find(medgeC);

        ChNodeFEAxyz* i_wingvertex_A = 0;
        ChNodeFEAxyz* i_wingvertex_B = 0;
        ChNodeFEAxyz* i_wingvertex_C = 0;

        if (tri_map[it][1] != -1) {
            i_wingvertex_A = triangles[tri_map[it][1]][0];
            if (triangles[tri_map[it][1]][1] != wingedgeA->first.first &&
                triangles[tri_map[it][1]][1] != wingedgeA->first.second)
                i_wingvertex_A = triangles[tri_map[it][1]][1];
            if (triangles[tri_map[it][1]][2] != wingedgeA->first.first &&
                triangles[tri_map[it][1]][2] != wingedgeA->first.second)
                i_wingvertex_A = triangles[tri_map[it][1]][2];
        }

        if (tri_map[it][2] != -1) {
            i_wingvertex_B = triangles[tri_map[it][2]][0];
            if (triangles[tri_map[it][2]][1] != wingedgeB->first.first &&
                triangles[tri_map[it][2]][1] != wingedgeB->first.second)
                i_wingvertex_B = triangles[tri_map[it][2]][1];
            if (triangles[tri_map[it][2]][2] != wingedgeB->first.first &&
                triangles[tri_map[it][2]][2] != wingedgeB->first.second)
                i_wingvertex_B = triangles[tri_map[it][2]][2];
        }

        if (tri_map[it][3] != -1) {
            i_wingvertex_C = triangles[tri_map[it][3]][0];
            if (triangles[tri_map[it][3]][1] != wingedgeC->first.first &&
                triangles[tri_map[it][3]][1] != wingedgeC->first.second)
                i_wingvertex_C = triangles[tri_map[it][3]][1];
            if (triangles[tri_map[it][3]][2] != wingedgeC->first.first &&
                triangles[tri_map[it][3]][2] != wingedgeC->first.second)
                i_wingvertex_C = triangles[tri_map[it][3]][2];
        }

        bool owns_node1 = (added_vertexes.find(triangles[it][0]) == added_vertexes.end());
        bool owns_node2 = (added_vertexes.find(triangles[it][1]) == added_vertexes.end());
        bool owns_node3 = (added_vertexes.find(triangles[it][2]) == added_vertexes.end());
        
        bool owns_edge1 = (wingedgeA->second.first != -1);
        bool owns_edge2 = (wingedgeB->second.first != -1);
        bool owns_edge3 = (wingedgeC->second.first != -1);

        auto contact_triangle = chrono_types::make_shared<ChContactTriangleXYZ>();
        contact_triangle->SetNodes({{triangles_ptrs[it][0], triangles_ptrs[it][1], triangles_ptrs[it][2]}});
        contact_triangle->SetNodeOwnership({owns_node1, owns_node2, owns_node3});
        contact_triangle->SetEdgeOwnership({owns_edge1, owns_edge2, owns_edge3});
        contact_triangle->SetContactSurface(this);
        vfaces.push_back(contact_triangle);

        contact_triangle->GetCollisionModel()->ClearModel();
        ((collision::ChCollisionModelBullet*)contact_triangle->GetCollisionModel())
            ->AddTriangleProxy(m_material,  // contact material
                               &triangles[it][0]->pos, &triangles[it][1]->pos, &triangles[it][2]->pos,
                               // if no wing vertex (ie. 'free' edge), point to vertex opposite to the edge
                               wingedgeA->second.second != -1 ? &i_wingvertex_A->pos : &triangles[it][2]->pos,
                               wingedgeB->second.second != -1 ? &i_wingvertex_B->pos : &triangles[it][0]->pos,
                               wingedgeC->second.second != -1 ? &i_wingvertex_C->pos : &triangles[it][1]->pos,
                               owns_node1, owns_node2, owns_node3,
                               // are edges owned by this triangle? (if not, they belong to a neighboring triangle)
                               owns_edge1, owns_edge2, owns_edge3, sphere_swept);
        contact_triangle->GetCollisionModel()->BuildModel();

        // Mark added vertexes
        added_vertexes.insert(triangles[it][0]);
        added_vertexes.insert(triangles[it][1]);
        added_vertexes.insert(triangles[it][2]);

        // Mark added edges, setting to -1 the 'ti' id of 1st triangle in winged edge {{vi,vj}{ti,tj}}
        wingedgeA->second.first = -1;
        wingedgeB->second.first = -1;
        wingedgeC->second.first = -1;
    }
}

void ChContactSurfaceMesh::AddFacesFromTripletsXYZrot(const std::vector<NodeTripletXYZrot>& triangles_ptrs,
                                                      double sphere_swept) {
    std::vector<std::array<ChNodeFEAxyzrot*, 3>> triangles;
    for (const auto& tri : triangles_ptrs) {
        triangles.push_back({{tri[0].get(), tri[1].get(), tri[2].get()}});
    }

    // compute connectivity also for triangles with rotational dofs:

    std::multimap<std::pair<ChNodeFEAxyzrot*, ChNodeFEAxyzrot*>, int> edge_map_rot;

    for (int it = 0; it < triangles.size(); ++it) {
        // edges = pairs of vertexes indexes
        std::pair<ChNodeFEAxyzrot*, ChNodeFEAxyzrot*> medgeA(triangles[it][0], triangles[it][1]);
        std::pair<ChNodeFEAxyzrot*, ChNodeFEAxyzrot*> medgeB(triangles[it][1], triangles[it][2]);
        std::pair<ChNodeFEAxyzrot*, ChNodeFEAxyzrot*> medgeC(triangles[it][2], triangles[it][0]);
        // vertex indexes in edges: always in increasing order to avoid ambiguous duplicated edges
        if (medgeA.first > medgeA.second)
            medgeA = std::pair<ChNodeFEAxyzrot*, ChNodeFEAxyzrot*>(medgeA.second, medgeA.first);
        if (medgeB.first > medgeB.second)
            medgeB = std::pair<ChNodeFEAxyzrot*, ChNodeFEAxyzrot*>(medgeB.second, medgeB.first);
        if (medgeC.first > medgeC.second)
            medgeC = std::pair<ChNodeFEAxyzrot*, ChNodeFEAxyzrot*>(medgeC.second, medgeC.first);
        edge_map_rot.insert({medgeA, it});
        edge_map_rot.insert({medgeB, it});
        edge_map_rot.insert({medgeC, it});
    }

    // Create a map of neighboring triangles, vector of:
    // [Ti TieA TieB TieC]
    std::vector<std::array<int, 4>> tri_map_rot;
    tri_map_rot.resize(triangles.size());

    for (int it = 0; it < triangles.size(); ++it) {
        tri_map_rot[it][0] = it;
        tri_map_rot[it][1] = -1;  // default no neighbor
        tri_map_rot[it][2] = -1;  // default no neighbor
        tri_map_rot[it][3] = -1;  // default no neighbor
        // edges = pairs of vertexes indexes
        std::pair<ChNodeFEAxyzrot*, ChNodeFEAxyzrot*> medgeA(triangles[it][0], triangles[it][1]);
        std::pair<ChNodeFEAxyzrot*, ChNodeFEAxyzrot*> medgeB(triangles[it][1], triangles[it][2]);
        std::pair<ChNodeFEAxyzrot*, ChNodeFEAxyzrot*> medgeC(triangles[it][2], triangles[it][0]);
        // vertex indexes in edges: always in increasing order to avoid ambiguous duplicated edges
        if (medgeA.first > medgeA.second)
            medgeA = std::pair<ChNodeFEAxyzrot*, ChNodeFEAxyzrot*>(medgeA.second, medgeA.first);
        if (medgeB.first > medgeB.second)
            medgeB = std::pair<ChNodeFEAxyzrot*, ChNodeFEAxyzrot*>(medgeB.second, medgeB.first);
        if (medgeC.first > medgeC.second)
            medgeC = std::pair<ChNodeFEAxyzrot*, ChNodeFEAxyzrot*>(medgeC.second, medgeC.first);
        auto retA = edge_map_rot.equal_range(medgeA);
        for (auto fedge = retA.first; fedge != retA.second; ++fedge) {
            if (fedge->second != it) {
                tri_map_rot[it][1] = fedge->second;
                break;
            }
        }
        auto retB = edge_map_rot.equal_range(medgeB);
        for (auto fedge = retB.first; fedge != retB.second; ++fedge) {
            if (fedge->second != it) {
                tri_map_rot[it][2] = fedge->second;
                break;
            }
        }
        auto retC = edge_map_rot.equal_range(medgeC);
        for (auto fedge = retC.first; fedge != retC.second; ++fedge) {
            if (fedge->second != it) {
                tri_map_rot[it][3] = fedge->second;
                break;
            }
        }
    }

    std::map<std::pair<ChNodeFEAxyzrot*, ChNodeFEAxyzrot*>, std::pair<int, int>> winged_edges_rot;
    bool allow_single_wing_rot = true;

    for (auto aedge = edge_map_rot.begin(); aedge != edge_map_rot.end(); ++aedge) {
        auto ret = edge_map_rot.equal_range(aedge->first);
        int nt = 0;
        std::pair<ChNodeFEAxyzrot*, ChNodeFEAxyzrot*> wingedge;
        std::pair<int, int> wingtri;
        wingtri.first = -1;
        wingtri.second = -1;
        for (auto fedge = ret.first; fedge != ret.second; ++fedge) {
            if (fedge->second == -1)
                break;
            wingedge.first = fedge->first.first;
            wingedge.second = fedge->first.second;
            if (nt == 0)
                wingtri.first = fedge->second;
            if (nt == 1)
                wingtri.second = fedge->second;
            ++nt;
            if (nt == 2)
                break;
        }
        if ((nt == 2) || ((nt == 1) && allow_single_wing_rot)) {
            winged_edges_rot.insert(std::pair<std::pair<ChNodeFEAxyzrot*, ChNodeFEAxyzrot*>, std::pair<int, int>>(
                wingedge, wingtri));  // ok found winged edge!
            aedge->second = -1;       // deactivate this way otherwise found again by sister
        }
    }

    // ....repeat: create triangles with collision models for nodes with rotationaldofs too:

    std::set<ChNodeFEAxyzrot*> added_vertexes_rot;

    // iterate on triangles
    for (int it = 0; it < triangles.size(); ++it) {
        // edges = pairs of vertexes indexes
        std::pair<ChNodeFEAxyzrot*, ChNodeFEAxyzrot*> medgeA(triangles[it][0], triangles[it][1]);
        std::pair<ChNodeFEAxyzrot*, ChNodeFEAxyzrot*> medgeB(triangles[it][1], triangles[it][2]);
        std::pair<ChNodeFEAxyzrot*, ChNodeFEAxyzrot*> medgeC(triangles[it][2], triangles[it][0]);
        // vertex indexes in edges: always in increasing order to avoid ambiguous duplicated edges
        if (medgeA.first > medgeA.second)
            medgeA = std::pair<ChNodeFEAxyzrot*, ChNodeFEAxyzrot*>(medgeA.second, medgeA.first);
        if (medgeB.first > medgeB.second)
            medgeB = std::pair<ChNodeFEAxyzrot*, ChNodeFEAxyzrot*>(medgeB.second, medgeB.first);
        if (medgeC.first > medgeC.second)
            medgeC = std::pair<ChNodeFEAxyzrot*, ChNodeFEAxyzrot*>(medgeC.second, medgeC.first);
        auto wingedgeA = winged_edges_rot.find(medgeA);
        auto wingedgeB = winged_edges_rot.find(medgeB);
        auto wingedgeC = winged_edges_rot.find(medgeC);

        ChNodeFEAxyzrot* i_wingvertex_A = 0;
        ChNodeFEAxyzrot* i_wingvertex_B = 0;
        ChNodeFEAxyzrot* i_wingvertex_C = 0;

        if (tri_map_rot[it][1] != -1) {
            i_wingvertex_A = triangles[tri_map_rot[it][1]][0];
            if (triangles[tri_map_rot[it][1]][1] != wingedgeA->first.first &&
                triangles[tri_map_rot[it][1]][1] != wingedgeA->first.second)
                i_wingvertex_A = triangles[tri_map_rot[it][1]][1];
            if (triangles[tri_map_rot[it][1]][2] != wingedgeA->first.first &&
                triangles[tri_map_rot[it][1]][2] != wingedgeA->first.second)
                i_wingvertex_A = triangles[tri_map_rot[it][1]][2];
        }

        if (tri_map_rot[it][2] != -1) {
            i_wingvertex_B = triangles[tri_map_rot[it][2]][0];
            if (triangles[tri_map_rot[it][2]][1] != wingedgeB->first.first &&
                triangles[tri_map_rot[it][2]][1] != wingedgeB->first.second)
                i_wingvertex_B = triangles[tri_map_rot[it][2]][1];
            if (triangles[tri_map_rot[it][2]][2] != wingedgeB->first.first &&
                triangles[tri_map_rot[it][2]][2] != wingedgeB->first.second)
                i_wingvertex_B = triangles[tri_map_rot[it][2]][2];
        }

        if (tri_map_rot[it][3] != -1) {
            i_wingvertex_C = triangles[tri_map_rot[it][3]][0];
            if (triangles[tri_map_rot[it][3]][1] != wingedgeC->first.first &&
                triangles[tri_map_rot[it][3]][1] != wingedgeC->first.second)
                i_wingvertex_C = triangles[tri_map_rot[it][3]][1];
            if (triangles[tri_map_rot[it][3]][2] != wingedgeC->first.first &&
                triangles[tri_map_rot[it][3]][2] != wingedgeC->first.second)
                i_wingvertex_C = triangles[tri_map_rot[it][3]][2];
        }

        bool owns_node1 = (added_vertexes_rot.find(triangles[it][0]) == added_vertexes_rot.end());
        bool owns_node2 = (added_vertexes_rot.find(triangles[it][1]) == added_vertexes_rot.end());
        bool owns_node3 = (added_vertexes_rot.find(triangles[it][2]) == added_vertexes_rot.end());

        bool owns_edge1 = (wingedgeA->second.first != -1);
        bool owns_edge2 = (wingedgeB->second.first != -1);
        bool owns_edge3 = (wingedgeC->second.first != -1);

        auto contact_triangle_rot = chrono_types::make_shared<ChContactTriangleXYZROT>();
        contact_triangle_rot->SetNodes({{triangles_ptrs[it][0], triangles_ptrs[it][1], triangles_ptrs[it][2]}});
        contact_triangle_rot->SetNodeOwnership({owns_node1, owns_node2, owns_node3});
        contact_triangle_rot->SetEdgeOwnership({owns_edge1, owns_edge2, owns_edge3});
        contact_triangle_rot->SetContactSurface(this);
        vfaces_rot.push_back(contact_triangle_rot);

        contact_triangle_rot->GetCollisionModel()->ClearModel();
        ((collision::ChCollisionModelBullet*)contact_triangle_rot->GetCollisionModel())
            ->AddTriangleProxy(
                m_material,  // contact material
                &triangles[it][0]->coord.pos, &triangles[it][1]->coord.pos, &triangles[it][2]->coord.pos,
                // if no wing vertex (ie. 'free' edge), point to vertex opposite to the edge
                wingedgeA->second.second != -1 ? &i_wingvertex_A->coord.pos : &triangles[it][2]->coord.pos,
                wingedgeB->second.second != -1 ? &i_wingvertex_B->coord.pos : &triangles[it][0]->coord.pos,
                wingedgeC->second.second != -1 ? &i_wingvertex_C->coord.pos : &triangles[it][1]->coord.pos,
                owns_node1, owns_node2, owns_node3,
                // are edges owned by this triangle? (if not, they belong to a neighboring triangle)
                owns_edge1, owns_edge2, owns_edge3, sphere_swept);
        contact_triangle_rot->GetCollisionModel()->BuildModel();

        // Mark added vertexes
        added_vertexes_rot.insert(triangles[it][0]);
        added_vertexes_rot.insert(triangles[it][1]);
        added_vertexes_rot.insert(triangles[it][2]);

        // Mark added edges, setting to -1 the 'ti' id of 1st triangle in winged edge {{vi,vj}{ti,tj}}
        wingedgeA->second.first = -1;
        wingedgeB->second.first = -1;
        wingedgeC->second.first = -1;
    }
}

unsigned int ChContactSurfaceMesh::GetNumVertices() const {
    std::map<ChNodeFEAxyz*, size_t> ptr_ind_map;
    size_t count = 0;
    for (size_t i = 0; i < vfaces.size(); ++i) {
        if (!ptr_ind_map.count(vfaces[i]->GetNode(0).get())) {
            ptr_ind_map.insert({vfaces[i]->GetNode(0).get(), count});
            count++;
        }
        if (!ptr_ind_map.count(vfaces[i]->GetNode(1).get())) {
            ptr_ind_map.insert({vfaces[i]->GetNode(1).get(), count});
            count++;
        }
        if (!ptr_ind_map.count(vfaces[i]->GetNode(2).get())) {
            ptr_ind_map.insert({vfaces[i]->GetNode(2).get(), count});
            count++;
        }
    }

    std::map<ChNodeFEAxyzrot*, size_t> ptr_ind_map_rot;
    size_t count_rot = 0;
    for (size_t i = 0; i < vfaces_rot.size(); ++i) {
        if (!ptr_ind_map_rot.count(vfaces_rot[i]->GetNode(0).get())) {
            ptr_ind_map_rot.insert({vfaces_rot[i]->GetNode(0).get(), count_rot});
            count_rot++;
        }
        if (!ptr_ind_map_rot.count(vfaces_rot[i]->GetNode(1).get())) {
            ptr_ind_map_rot.insert({vfaces_rot[i]->GetNode(1).get(), count_rot});
            count_rot++;
        }
        if (!ptr_ind_map_rot.count(vfaces_rot[i]->GetNode(2).get())) {
            ptr_ind_map_rot.insert({vfaces_rot[i]->GetNode(2).get(), count_rot});
            count_rot++;
        }
    }

    return (unsigned int)(count + count_rot);
}

void ChContactSurfaceMesh::SurfaceSyncCollisionModels() {
    for (unsigned int j = 0; j < vfaces.size(); j++) {
        vfaces[j]->GetCollisionModel()->SyncPosition();
    }
    for (unsigned int j = 0; j < vfaces_rot.size(); j++) {
        vfaces_rot[j]->GetCollisionModel()->SyncPosition();
    }
}

void ChContactSurfaceMesh::SurfaceAddCollisionModelsToSystem(ChSystem* msys) {
    assert(msys);
    SurfaceSyncCollisionModels();
    for (unsigned int j = 0; j < vfaces.size(); j++) {
        msys->GetCollisionSystem()->Add(vfaces[j]->GetCollisionModel());
    }
    for (unsigned int j = 0; j < vfaces_rot.size(); j++) {
        msys->GetCollisionSystem()->Add(vfaces_rot[j]->GetCollisionModel());
    }
}

void ChContactSurfaceMesh::SurfaceRemoveCollisionModelsFromSystem(ChSystem* msys) {
    assert(msys);
    for (unsigned int j = 0; j < vfaces.size(); j++) {
        msys->GetCollisionSystem()->Remove(vfaces[j]->GetCollisionModel());
    }
    for (unsigned int j = 0; j < vfaces_rot.size(); j++) {
        msys->GetCollisionSystem()->Remove(vfaces_rot[j]->GetCollisionModel());
    }
}

void ChContactSurfaceMesh::OutputSimpleMesh(std::vector<ChVector<>>& vert_pos,
                                            std::vector<ChVector<>>& vert_vel,
                                            std::vector<ChVector<int>>& triangles,
                                            std::vector<ChVector<bool>>& owns_node,
                                            std::vector<ChVector<bool>>& owns_edge) const {
    vert_pos.clear();
    vert_vel.clear();
    triangles.clear();
    owns_node.clear();
    owns_edge.clear();

    int vertex_index = 0;

    // FEA nodes with position DOFs
    {
        std::map<ChNodeFEAxyz*, int> ptr_ind_map;  // map from pointer-based mesh to index-based mesh

        for (const auto& tri : vfaces) {
            if (ptr_ind_map.insert({tri->GetNode(0).get(), vertex_index}).second) {
                vert_pos.push_back(tri->GetNode(0)->GetPos());
                vert_vel.push_back(tri->GetNode(0)->GetPos_dt());
                ++vertex_index;
            }
            if (ptr_ind_map.insert({tri->GetNode(1).get(), vertex_index}).second) {
                vert_pos.push_back(tri->GetNode(1)->GetPos());
                vert_vel.push_back(tri->GetNode(1)->GetPos_dt());
                ++vertex_index;
            }
            if (ptr_ind_map.insert({tri->GetNode(2).get(), vertex_index}).second) {
                vert_pos.push_back(tri->GetNode(2)->GetPos());
                vert_vel.push_back(tri->GetNode(2)->GetPos_dt());
                ++vertex_index;
            }
        }

        for (const auto& tri : vfaces) {
            triangles.push_back({ptr_ind_map.at(tri->GetNode(0).get()),  //
                                 ptr_ind_map.at(tri->GetNode(1).get()),  //
                                 ptr_ind_map.at(tri->GetNode(2).get())});
            owns_node.push_back({tri->OwnsNode(0), tri->OwnsNode(1), tri->OwnsNode(2)});
            owns_edge.push_back({tri->OwnsEdge(0), tri->OwnsEdge(1), tri->OwnsEdge(2)});
        }
    }

    // FEA nodes with position and rotation DOFs
    {
        std::map<ChNodeFEAxyzrot*, int> ptr_ind_map;  // map from pointer-based mesh to index-based mesh

        for (const auto& tri : vfaces_rot) {
            if (ptr_ind_map.insert({tri->GetNode(0).get(), vertex_index}).second) {
                vert_pos.push_back(tri->GetNode(0)->GetPos());
                vert_vel.push_back(tri->GetNode(0)->GetPos_dt());
                ++vertex_index;
            }
            if (ptr_ind_map.insert({tri->GetNode(1).get(), vertex_index}).second) {
                vert_pos.push_back(tri->GetNode(1)->GetPos());
                vert_vel.push_back(tri->GetNode(1)->GetPos_dt());
                ++vertex_index;
            }
            if (ptr_ind_map.insert({tri->GetNode(2).get(), vertex_index}).second) {
                vert_pos.push_back(tri->GetNode(2)->GetPos());
                vert_vel.push_back(tri->GetNode(2)->GetPos_dt());
                ++vertex_index;
            }
        }

        for (const auto& tri : vfaces_rot) {
            triangles.push_back({ptr_ind_map.at(tri->GetNode(0).get()),  //
                                 ptr_ind_map.at(tri->GetNode(1).get()),  //
                                 ptr_ind_map.at(tri->GetNode(2).get())});
            owns_node.push_back({tri->OwnsNode(0), tri->OwnsNode(1), tri->OwnsNode(2)});
            owns_edge.push_back({tri->OwnsEdge(0), tri->OwnsEdge(1), tri->OwnsEdge(2)});
        }
    }
}

}  // end namespace fea
}  // end namespace chrono
