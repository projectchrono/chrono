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
// Authors: Andrea Favali, Alessandro Tasora, Radu Serban
// =============================================================================

#include "chrono_fea/ChNodeFEAxyzD.h"

namespace chrono {
namespace fea {

ChNodeFEAxyzD::ChNodeFEAxyzD(ChVector<> initial_pos, ChVector<> initial_dir)
    : ChNodeFEAxyz(initial_pos), D(initial_dir), D_dt(VNULL), D_dtdt(VNULL) {
    variables_D = new ChVariablesGenericDiagonalMass(3);
    // default: no atomic mass associated to fea node, the fea element will add mass matrix
    variables_D->GetMassDiagonal().FillElem(0.0);
}

ChNodeFEAxyzD::ChNodeFEAxyzD(const ChNodeFEAxyzD& other) : ChNodeFEAxyz(other) {
    variables_D = new ChVariablesGenericDiagonalMass(3);
    (*variables_D) = (*other.variables_D);
    D = other.D;
    D_dt = other.D_dt;
    D_dtdt = other.D_dtdt;
}

ChNodeFEAxyzD::~ChNodeFEAxyzD() {
    delete variables_D;
}

// -----------------------------------------------------------------------------

ChNodeFEAxyzD& ChNodeFEAxyzD::operator=(const ChNodeFEAxyzD& other) {
    if (&other == this)
        return *this;

    ChNodeFEAxyz::operator=(other);

    D = other.D;
    D_dt = other.D_dt;
    D_dtdt = other.D_dtdt;
    (*variables_D) = (*other.variables_D);
    return *this;
}

// -----------------------------------------------------------------------------

void ChNodeFEAxyzD::SetNoSpeedNoAcceleration() {
    ChNodeFEAxyz::SetNoSpeedNoAcceleration();

    D_dt = VNULL;
    D_dtdt = VNULL;
}

void ChNodeFEAxyzD::SetFixed(bool mev) {
    variables.SetDisabled(mev);
    variables_D->SetDisabled(mev);
}

// -----------------------------------------------------------------------------

void ChNodeFEAxyzD::NodeIntStateGather(const unsigned int off_x,
                                       ChState& x,
                                       const unsigned int off_v,
                                       ChStateDelta& v,
                                       double& T) {
    x.PasteVector(pos, off_x, 0);
    x.PasteVector(D, off_x + 3, 0);
    v.PasteVector(pos_dt, off_v, 0);
    v.PasteVector(D_dt, off_v + 3, 0);
}

void ChNodeFEAxyzD::NodeIntStateScatter(const unsigned int off_x,
                                        const ChState& x,
                                        const unsigned int off_v,
                                        const ChStateDelta& v,
                                        const double T) {
    SetPos(x.ClipVector(off_x, 0));
    SetD(x.ClipVector(off_x + 3, 0));
    SetPos_dt(v.ClipVector(off_v, 0));
    SetD_dt(v.ClipVector(off_v + 3, 0));
}

void ChNodeFEAxyzD::NodeIntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) {
    a.PasteVector(pos_dtdt, off_a, 0);
    a.PasteVector(D_dtdt, off_a + 3, 0);
}

void ChNodeFEAxyzD::NodeIntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) {
    SetPos_dtdt(a.ClipVector(off_a, 0));
    SetD_dtdt(a.ClipVector(off_a + 3, 0));
}

void ChNodeFEAxyzD::NodeIntStateIncrement(const unsigned int off_x,
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
}

void ChNodeFEAxyzD::NodeIntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) {
    R.PasteSumVector(Force * c, off, 0);
    R.PasteSumVector(VNULL, off + 3, 0);  // TODO something about applied nodal torque..
}

void ChNodeFEAxyzD::NodeIntLoadResidual_Mv(const unsigned int off,
                                           ChVectorDynamic<>& R,
                                           const ChVectorDynamic<>& w,
                                           const double c) {
    R(off + 0) += c * GetMass() * w(off + 0);
    R(off + 1) += c * GetMass() * w(off + 1);
    R(off + 2) += c * GetMass() * w(off + 2);
    R(off + 3) += c * GetMassDiagonal()(0) * w(off + 3);  // unuseful? mass for D isalways zero..
    R(off + 4) += c * GetMassDiagonal()(1) * w(off + 4);
    R(off + 5) += c * GetMassDiagonal()(2) * w(off + 5);
}

void ChNodeFEAxyzD::NodeIntToDescriptor(const unsigned int off_v, const ChStateDelta& v, const ChVectorDynamic<>& R) {
    ChNodeFEAxyz::NodeIntToDescriptor(off_v, v, R);
    variables_D->Get_qb().PasteClippedMatrix(v, off_v + 3, 0, 3, 1, 0, 0);
    variables_D->Get_fb().PasteClippedMatrix(R, off_v + 3, 0, 3, 1, 0, 0);
}

void ChNodeFEAxyzD::NodeIntFromDescriptor(const unsigned int off_v, ChStateDelta& v) {
    ChNodeFEAxyz::NodeIntFromDescriptor(off_v, v);
    v.PasteMatrix(variables_D->Get_qb(), off_v + 3, 0);
}

// -----------------------------------------------------------------------------

void ChNodeFEAxyzD::InjectVariables(ChSystemDescriptor& mdescriptor) {
    ChNodeFEAxyz::InjectVariables(mdescriptor);
    mdescriptor.InsertVariables(variables_D);
}

void ChNodeFEAxyzD::VariablesFbReset() {
    ChNodeFEAxyz::VariablesFbReset();
    variables_D->Get_fb().FillElem(0.0);
}

void ChNodeFEAxyzD::VariablesFbLoadForces(double factor) {
    ChNodeFEAxyz::VariablesFbLoadForces(factor);
    // variables_D->Get_fb().PasteSumVector( VNULL ,3,0); // TODO something related to inertia?
}

void ChNodeFEAxyzD::VariablesQbLoadSpeed() {
    ChNodeFEAxyz::VariablesQbLoadSpeed();
    variables_D->Get_qb().PasteVector(D_dt, 0, 0);
}

void ChNodeFEAxyzD::VariablesQbSetSpeed(double step) {
    ChNodeFEAxyz::VariablesQbSetSpeed(step);

    ChVector<> oldD_dt = D_dt;
    SetD_dt(variables_D->Get_qb().ClipVector(0, 0));
    if (step) {
        SetD_dtdt((D_dt - oldD_dt) / step);
    }
}

void ChNodeFEAxyzD::VariablesFbIncrementMq() {
    ChNodeFEAxyz::VariablesFbIncrementMq();
    variables_D->Compute_inc_Mb_v(variables_D->Get_fb(), variables_D->Get_qb());
}

void ChNodeFEAxyzD::VariablesQbIncrementPosition(double step) {
    ChNodeFEAxyz::VariablesQbIncrementPosition(step);

    ChVector<> newspeed_D = variables_D->Get_qb().ClipVector(0, 0);

    // ADVANCE POSITION: pos' = pos + dt * vel
    SetD(GetD() + newspeed_D * step);
}

// -----------------------------------------------------------------------------

void ChNodeFEAxyzD::ComputeNF(
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
    ChVector<> absF = F.ClipVector(0, 0);
    ChVector<> absPseudoTorque = F.ClipVector(3, 0);
    Qi.PasteVector(absF, 0, 0);
    Qi.PasteVector(absPseudoTorque, 3, 0);
    detJ = 1;  // not needed because not used in quadrature.
}

// -----------------------------------------------------------------------------

void ChNodeFEAxyzD::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChNodeFEAxyzD>();
    // serialize parent class
    ChNodeFEAxyz::ArchiveOUT(marchive);
    // serialize all member data:
    marchive << CHNVP(D);
    marchive << CHNVP(D_dt);
    marchive << CHNVP(D_dtdt);
}

void ChNodeFEAxyzD::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChNodeFEAxyzD>();
    // deserialize parent class
    ChNodeFEAxyz::ArchiveIN(marchive);
    // stream in all member data:
    marchive >> CHNVP(D);
    marchive >> CHNVP(D_dt);
    marchive >> CHNVP(D_dtdt);
}

}  // end namespace fea
}  // end namespace chrono
