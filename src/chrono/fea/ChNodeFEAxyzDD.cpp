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
// Authors: Antonio Recuero
// =============================================================================

#include "chrono/fea/ChNodeFEAxyzDD.h"

namespace chrono {
namespace fea {

ChNodeFEAxyzDD::ChNodeFEAxyzDD(ChVector<> initial_pos, ChVector<> initial_dir, ChVector<> initial_curv)
    : ChNodeFEAxyzD(initial_pos, initial_dir), DD(initial_curv), DD_dt(VNULL), DD_dtdt(VNULL) {
    variables_DD = new ChVariablesGenericDiagonalMass(3);
    // default: no atomic mass associated to fea node, the fea element will add mass matrix
    variables_DD->GetMassDiagonal().setZero();
}

ChNodeFEAxyzDD::ChNodeFEAxyzDD(const ChNodeFEAxyzDD& other) : ChNodeFEAxyzD(other) {
    variables_DD = new ChVariablesGenericDiagonalMass(3);
    (*variables_DD) = (*other.variables_DD);
    DD = other.DD;
    DD_dt = other.DD_dt;
    DD_dtdt = other.DD_dtdt;
}

ChNodeFEAxyzDD::~ChNodeFEAxyzDD() {
    delete variables_DD;
}

// -----------------------------------------------------------------------------

ChNodeFEAxyzDD& ChNodeFEAxyzDD::operator=(const ChNodeFEAxyzDD& other) {
    if (&other == this)
        return *this;

    ChNodeFEAxyzD::operator=(other);

    DD = other.DD;
    DD_dt = other.DD_dt;
    DD_dtdt = other.DD_dtdt;
    (*variables_DD) = (*other.variables_DD);
    return *this;
}

// -----------------------------------------------------------------------------

void ChNodeFEAxyzDD::SetNoSpeedNoAcceleration() {
    ChNodeFEAxyzD::SetNoSpeedNoAcceleration();

    DD_dt = VNULL;
    DD_dtdt = VNULL;
}

void ChNodeFEAxyzDD::SetFixed(bool mev) {
    ChNodeFEAxyzD::SetFixed(mev);
    variables_DD->SetDisabled(mev);
}

// -----------------------------------------------------------------------------

void ChNodeFEAxyzDD::NodeIntStateGather(const unsigned int off_x,
                                        ChState& x,
                                        const unsigned int off_v,
                                        ChStateDelta& v,
                                        double& T) {
    x.segment(off_x + 0, 3) = pos.eigen();
    x.segment(off_x + 3, 3) = D.eigen();
    x.segment(off_x + 6, 3) = DD.eigen();

    v.segment(off_v + 0, 3) = pos_dt.eigen();
    v.segment(off_v + 3, 3) = D_dt.eigen();
    v.segment(off_v + 6, 3) = DD_dt.eigen();
}

void ChNodeFEAxyzDD::NodeIntStateScatter(const unsigned int off_x,
                                         const ChState& x,
                                         const unsigned int off_v,
                                         const ChStateDelta& v,
                                         const double T) {
    SetPos(x.segment(off_x, 3));
    SetD(x.segment(off_x + 3, 3));
    SetDD(x.segment(off_x + 6, 3));

    SetPos_dt(v.segment(off_v, 3));
    SetD_dt(v.segment(off_v + 3, 3));
    SetDD_dt(v.segment(off_v + 6, 3));
}

void ChNodeFEAxyzDD::NodeIntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) {
    a.segment(off_a + 0, 3) = pos_dtdt.eigen();
    a.segment(off_a + 3, 3) = D_dtdt.eigen();
    a.segment(off_a + 6, 3) = DD_dtdt.eigen();
}

void ChNodeFEAxyzDD::NodeIntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) {
    SetPos_dtdt(a.segment(off_a, 3));
    SetD_dtdt(a.segment(off_a + 3, 3));
    SetDD_dtdt(a.segment(off_a + 6, 3));
}

void ChNodeFEAxyzDD::NodeIntStateIncrement(const unsigned int off_x,
                                           ChState& x_new,
                                           const ChState& x,
                                           const unsigned int off_v,
                                           const ChStateDelta& Dv) {
    x_new(off_x + 0) = x(off_x + 0) + Dv(off_v + 0);
    x_new(off_x + 1) = x(off_x + 1) + Dv(off_v + 1);
    x_new(off_x + 2) = x(off_x + 2) + Dv(off_v + 2);
    x_new(off_x + 3) = x(off_x + 3) + Dv(off_v + 3);
    x_new(off_x + 4) = x(off_x + 4) + Dv(off_v + 4);
    x_new(off_x + 5) = x(off_x + 5) + Dv(off_v + 5);
    x_new(off_x + 6) = x(off_x + 6) + Dv(off_v + 6);
    x_new(off_x + 7) = x(off_x + 7) + Dv(off_v + 7);
    x_new(off_x + 8) = x(off_x + 8) + Dv(off_v + 8);
}

void ChNodeFEAxyzDD::NodeIntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) {
    R.segment(off + 0, 3) += c * Force.eigen();
    R.segment(off + 3, 3).setZero();  // TODO something about applied nodal torque..
    R.segment(off + 6, 3).setZero();  // TODO something about applied nodal torque..
}

void ChNodeFEAxyzDD::NodeIntLoadResidual_Mv(const unsigned int off,
                                            ChVectorDynamic<>& R,
                                            const ChVectorDynamic<>& w,
                                            const double c) {
    R(off + 0) += c * GetMass() * w(off + 0);
    R(off + 1) += c * GetMass() * w(off + 1);
    R(off + 2) += c * GetMass() * w(off + 2);
    R(off + 3) += c * GetMassDiagonal()(0) * w(off + 3);  // unuseful? mass for D isalways zero..
    R(off + 4) += c * GetMassDiagonal()(1) * w(off + 4);
    R(off + 5) += c * GetMassDiagonal()(2) * w(off + 5);
    R(off + 6) += c * GetMassDiagonalDD()(0) * w(off + 6);
    R(off + 7) += c * GetMassDiagonalDD()(1) * w(off + 7);
    R(off + 8) += c * GetMassDiagonalDD()(2) * w(off + 8);
}

void ChNodeFEAxyzDD::NodeIntToDescriptor(const unsigned int off_v, const ChStateDelta& v, const ChVectorDynamic<>& R) {
    ChNodeFEAxyzD::NodeIntToDescriptor(off_v, v, R);
    variables_DD->Get_qb().segment(0, 3) = v.segment(off_v + 6, 3);
    variables_DD->Get_fb().segment(0, 3) = R.segment(off_v + 6, 3);
}

void ChNodeFEAxyzDD::NodeIntFromDescriptor(const unsigned int off_v, ChStateDelta& v) {
    ChNodeFEAxyzD::NodeIntFromDescriptor(off_v, v);
    v.segment(off_v + 6, 3) = variables_DD->Get_qb().segment(0, 3);
}

// -----------------------------------------------------------------------------

void ChNodeFEAxyzDD::InjectVariables(ChSystemDescriptor& mdescriptor) {
    ChNodeFEAxyzD::InjectVariables(mdescriptor);
    mdescriptor.InsertVariables(variables_DD);
}

void ChNodeFEAxyzDD::VariablesFbReset() {
    ChNodeFEAxyzD::VariablesFbReset();
    variables_DD->Get_fb().setZero();
}

void ChNodeFEAxyzDD::VariablesFbLoadForces(double factor) {
    ChNodeFEAxyzD::VariablesFbLoadForces(factor);
    ////variables_D->Get_fb().segment(3, 3) += VNULL.eigen();  // TODO something related to inertia?
}

void ChNodeFEAxyzDD::VariablesQbLoadSpeed() {
    ChNodeFEAxyzD::VariablesQbLoadSpeed();
    variables_DD->Get_qb().segment(0,3) = DD_dt.eigen();
}

void ChNodeFEAxyzDD::VariablesQbSetSpeed(double step) {
    ChNodeFEAxyzD::VariablesQbSetSpeed(step);

    ChVector<> oldDD_dt = DD_dt;
    SetDD_dt(variables_DD->Get_qb().segment(0, 3));
    if (step) {
        SetDD_dtdt((DD_dt - oldDD_dt) / step);
    }
}

void ChNodeFEAxyzDD::VariablesFbIncrementMq() {
    ChNodeFEAxyzD::VariablesFbIncrementMq();
    variables_DD->Compute_inc_Mb_v(variables_DD->Get_fb(), variables_DD->Get_qb());
}

void ChNodeFEAxyzDD::VariablesQbIncrementPosition(double step) {
    ChNodeFEAxyzD::VariablesQbIncrementPosition(step);

    ChVector<> newspeed_DD(variables_DD->Get_qb().segment(0, 3));

    // ADVANCE POSITION: pos' = pos + dt * vel
    SetDD(GetDD() + newspeed_DD * step);
}

// -----------------------------------------------------------------------------

void ChNodeFEAxyzDD::ComputeNF(
    const double U,              ///< x coordinate of application point in absolute space
    const double V,              ///< y coordinate of application point in absolute space
    const double W,              ///< z coordinate of application point in absolute space
    ChVectorDynamic<>& Qi,       ///< Return result of N'*F  here, maybe with offset block_offset
    double& detJ,                ///< Return det[J] here
    const ChVectorDynamic<>& F,  ///< Input F vector, containing Force xyz in absolute coords and a 'pseudo' torque.
    ChVectorDynamic<>* state_x,  ///< if != 0, update state (pos. part) to this, then evaluate Q
    ChVectorDynamic<>* state_w   ///< if != 0, update state (speed part) to this, then evaluate Q
    ) {
    // ChVector<> abs_pos(U,V,W); not needed, nodes has no torque. Assuming load is applied to node center
    Qi.segment(0, 6) = F.segment(0, 6);  // [absF ; absPseudoTorque]
    detJ = 1;  // not needed because not used in quadrature.
}

// -----------------------------------------------------------------------------

void ChNodeFEAxyzDD::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChNodeFEAxyzDD>();
    // serialize parent class
    ChNodeFEAxyzD::ArchiveOUT(marchive);
    // serialize all member data:
    marchive << CHNVP(DD);
    marchive << CHNVP(DD_dt);
    marchive << CHNVP(DD_dtdt);
}

void ChNodeFEAxyzDD::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChNodeFEAxyzDD>();
    // deserialize parent class
    ChNodeFEAxyzD::ArchiveIN(marchive);
    // stream in all member data:
    marchive >> CHNVP(DD);
    marchive >> CHNVP(DD_dt);
    marchive >> CHNVP(DD_dtdt);
}

}  // end namespace fea
}  // end namespace chrono
