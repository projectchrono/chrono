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
// Authors: Mike Taylor, Antonio Recuero
// =============================================================================

#include "chrono/fea/ChNodeFEAxyzDDD.h"

namespace chrono {
namespace fea {

ChNodeFEAxyzDDD::ChNodeFEAxyzDDD(ChVector<> initial_pos, ChVector<> initial_dir_u, ChVector<> initial_dir_v, ChVector<> initial_dir_w)
    : ChNodeFEAxyzDD(initial_pos, initial_dir_u, initial_dir_v), DDD(initial_dir_w), DDD_dt(VNULL), DDD_dtdt(VNULL) {
    variables_DDD = new ChVariablesGenericDiagonalMass(3);
    // default: no atomic mass associated to fea node, the fea element will add mass matrix
    variables_DDD->GetMassDiagonal().setZero();
}

ChNodeFEAxyzDDD::ChNodeFEAxyzDDD(const ChNodeFEAxyzDDD& other) : ChNodeFEAxyzDD(other) {
    variables_DDD = new ChVariablesGenericDiagonalMass(3);
    (*variables_DDD) = (*other.variables_DDD);
    DDD = other.DDD;
    DDD_dt = other.DDD_dt;
    DDD_dtdt = other.DDD_dtdt;
}

ChNodeFEAxyzDDD::~ChNodeFEAxyzDDD() {
    delete variables_DDD;
}

// -----------------------------------------------------------------------------

ChNodeFEAxyzDDD& ChNodeFEAxyzDDD::operator=(const ChNodeFEAxyzDDD& other) {
    if (&other == this)
        return *this;

    ChNodeFEAxyzDD::operator=(other);

    DDD = other.DDD;
    DDD_dt = other.DDD_dt;
    DDD_dtdt = other.DDD_dtdt;
    (*variables_DDD) = (*other.variables_DDD);
    return *this;
}

// -----------------------------------------------------------------------------

void ChNodeFEAxyzDDD::SetNoSpeedNoAcceleration() {
    ChNodeFEAxyzDD::SetNoSpeedNoAcceleration();

    DDD_dt = VNULL;
    DDD_dtdt = VNULL;
}

void ChNodeFEAxyzDDD::SetFixed(bool mev) {
    ChNodeFEAxyzDD::SetFixed(mev);
    variables_DDD->SetDisabled(mev);
}

// -----------------------------------------------------------------------------

void ChNodeFEAxyzDDD::NodeIntStateGather(const unsigned int off_x,
                                        ChState& x,
                                        const unsigned int off_v,
                                        ChStateDelta& v,
                                        double& T) {
    x.segment(off_x + 0, 3) = pos.eigen();
    x.segment(off_x + 3, 3) = D.eigen();
    x.segment(off_x + 6, 3) = DD.eigen();
	x.segment(off_x + 9, 3) = DDD.eigen();

    v.segment(off_v + 0, 3) = pos_dt.eigen();
    v.segment(off_v + 3, 3) = D_dt.eigen();
    v.segment(off_v + 6, 3) = DD_dt.eigen();
	v.segment(off_v + 9, 3) = DDD_dt.eigen();
}

void ChNodeFEAxyzDDD::NodeIntStateScatter(const unsigned int off_x,
                                         const ChState& x,
                                         const unsigned int off_v,
                                         const ChStateDelta& v,
                                         const double T) {
    SetPos(x.segment(off_x, 3));
    SetD(x.segment(off_x + 3, 3));
    SetDD(x.segment(off_x + 6, 3));
	SetDDD(x.segment(off_x + 9, 3));

    SetPos_dt(v.segment(off_v, 3));
    SetD_dt(v.segment(off_v + 3, 3));
    SetDD_dt(v.segment(off_v + 6, 3));
	SetDDD_dt(v.segment(off_v + 9, 3));
}

void ChNodeFEAxyzDDD::NodeIntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) {
    a.segment(off_a + 0, 3) = pos_dtdt.eigen();
    a.segment(off_a + 3, 3) = D_dtdt.eigen();
    a.segment(off_a + 6, 3) = DD_dtdt.eigen();
	a.segment(off_a + 9, 3) = DDD_dtdt.eigen();
}

void ChNodeFEAxyzDDD::NodeIntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) {
    SetPos_dtdt(a.segment(off_a, 3));
    SetD_dtdt(a.segment(off_a + 3, 3));
    SetDD_dtdt(a.segment(off_a + 6, 3));
	SetDDD_dtdt(a.segment(off_a + 9, 3));
}

void ChNodeFEAxyzDDD::NodeIntStateIncrement(const unsigned int off_x,
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
	x_new(off_x + 9) = x(off_x + 9) + Dv(off_v + 9);
	x_new(off_x + 10) = x(off_x + 10) + Dv(off_v + 10);
	x_new(off_x + 11) = x(off_x + 11) + Dv(off_v + 11);
}

void ChNodeFEAxyzDDD::NodeIntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) {
    R.segment(off + 0, 3) += c * Force.eigen();
    R.segment(off + 3, 3).setZero();  // TODO something about applied nodal torque..
    R.segment(off + 6, 3).setZero();  // TODO something about applied nodal torque..
	R.segment(off + 9, 3).setZero();  // TODO something about applied nodal torque..
}

void ChNodeFEAxyzDDD::NodeIntLoadResidual_Mv(const unsigned int off,
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
    R(off + 9) += c * GetMassDiagonalDDD()(0) * w(off + 9);
    R(off + 10) += c * GetMassDiagonalDDD()(1) * w(off + 10);
    R(off + 11) += c * GetMassDiagonalDDD()(2) * w(off + 11);
}

void ChNodeFEAxyzDDD::NodeIntToDescriptor(const unsigned int off_v, const ChStateDelta& v, const ChVectorDynamic<>& R) {
    ChNodeFEAxyzDD::NodeIntToDescriptor(off_v, v, R);
    variables_DDD->Get_qb().segment(0, 3) = v.segment(off_v + 9, 3);
    variables_DDD->Get_fb().segment(0, 3) = R.segment(off_v + 9, 3);
}

void ChNodeFEAxyzDDD::NodeIntFromDescriptor(const unsigned int off_v, ChStateDelta& v) {
    ChNodeFEAxyzDD::NodeIntFromDescriptor(off_v, v);
    v.segment(off_v + 9, 3) = variables_DDD->Get_qb().segment(0, 3);
}

// -----------------------------------------------------------------------------

void ChNodeFEAxyzDDD::InjectVariables(ChSystemDescriptor& mdescriptor) {
    ChNodeFEAxyzDD::InjectVariables(mdescriptor);
    mdescriptor.InsertVariables(variables_DDD);
}

void ChNodeFEAxyzDDD::VariablesFbReset() {
    ChNodeFEAxyzDD::VariablesFbReset();
    variables_DDD->Get_fb().setZero();
}

void ChNodeFEAxyzDDD::VariablesFbLoadForces(double factor) {
    ChNodeFEAxyzDD::VariablesFbLoadForces(factor);
    ////variables_D->Get_fb().segment(3, 3) += VNULL.eigen();  // TODO something related to inertia?
}

void ChNodeFEAxyzDDD::VariablesQbLoadSpeed() {
    ChNodeFEAxyzDD::VariablesQbLoadSpeed();
    variables_DDD->Get_qb().segment(0,3) = DDD_dt.eigen();
}

void ChNodeFEAxyzDDD::VariablesQbSetSpeed(double step) {
    ChNodeFEAxyzDD::VariablesQbSetSpeed(step);

    ChVector<> oldDDD_dt = DDD_dt;
    SetDDD_dt(variables_DDD->Get_qb().segment(0, 3));
    if (step) {
        SetDDD_dtdt((DDD_dt - oldDDD_dt) / step);
    }
}

void ChNodeFEAxyzDDD::VariablesFbIncrementMq() {
    ChNodeFEAxyzDD::VariablesFbIncrementMq();
    variables_DDD->Compute_inc_Mb_v(variables_DDD->Get_fb(), variables_DDD->Get_qb());
}

void ChNodeFEAxyzDDD::VariablesQbIncrementPosition(double step) {
    ChNodeFEAxyzDD::VariablesQbIncrementPosition(step);

    ChVector<> newspeed_DDD(variables_DDD->Get_qb().segment(0, 3));

    // ADVANCE POSITION: pos' = pos + dt * vel
    SetDDD(GetDDD() + newspeed_DDD * step);
}

// -----------------------------------------------------------------------------

void ChNodeFEAxyzDDD::ComputeNF(
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

void ChNodeFEAxyzDDD::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChNodeFEAxyzDD>();
    // serialize parent class
    ChNodeFEAxyzDD::ArchiveOUT(marchive);
    // serialize all member data:
    marchive << CHNVP(DDD);
    marchive << CHNVP(DDD_dt);
    marchive << CHNVP(DDD_dtdt);
}

void ChNodeFEAxyzDDD::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    /*int version = */ marchive.VersionRead<ChNodeFEAxyzDDD>();
    // deserialize parent class
    ChNodeFEAxyzDD::ArchiveIN(marchive);
    // stream in all member data:
    marchive >> CHNVP(DDD);
    marchive >> CHNVP(DDD_dt);
    marchive >> CHNVP(DDD_dtdt);
}

}  // end namespace fea
}  // end namespace chrono
