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
// Authors: Andrea Favali, Alessandro Tasora, Radu Serban
// =============================================================================

#include "chrono/fea/ChNodeFEAxyzD.h"

namespace chrono {
namespace fea {

ChNodeFEAxyzD::ChNodeFEAxyzD(ChVector<> initial_pos, ChVector<> initial_dir)
    : ChNodeFEAxyz(initial_pos), m_dof_actual(0), D(initial_dir), D_dt(VNULL), D_dtdt(VNULL) {
    variables_D = new ChVariablesGenericDiagonalMass(3);
    // default: no atomic mass associated to fea node, the fea element will add mass matrix
    variables_D->GetMassDiagonal().setZero();
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

void ChNodeFEAxyzD::SetFixed(bool fixed) {
    ChNodeFEAxyz::SetFixed(fixed);
    variables_D->SetDisabled(fixed);
}

bool ChNodeFEAxyzD::IsFixed() const {
    return ChNodeFEAxyz::IsFixed() && variables_D->IsDisabled();
}

void ChNodeFEAxyzD::SetFixedD(bool fixed) {
    variables_D->SetDisabled(fixed);
}

bool ChNodeFEAxyzD::IsFixedD() const {
    return variables_D->IsDisabled();
}

void ChNodeFEAxyzD::SetupInitial(ChSystem* system) {
    if (IsFixed())
        m_dof_actual = 0;
    else if (IsFixedD())
        m_dof_actual = 3;
    else
        m_dof_actual = 6;
}

// -----------------------------------------------------------------------------

void ChNodeFEAxyzD::NodeIntStateGather(const unsigned int off_x,
                                       ChState& x,
                                       const unsigned int off_v,
                                       ChStateDelta& v,
                                       double& T) {
    ChNodeFEAxyz::NodeIntStateGather(off_x, x, off_v, v, T);
    if (!IsFixedD()) {
        x.segment(off_x + 3, 3) = D.eigen();
        v.segment(off_v + 3, 3) = D_dt.eigen();
    }
}

void ChNodeFEAxyzD::NodeIntStateScatter(const unsigned int off_x,
                                        const ChState& x,
                                        const unsigned int off_v,
                                        const ChStateDelta& v,
                                        const double T) {
    ChNodeFEAxyz::NodeIntStateScatter(off_x, x, off_v, v, T);
    if (!IsFixedD()) {
        SetD(x.segment(off_x + 3, 3));
        SetD_dt(v.segment(off_v + 3, 3));
    }
}

void ChNodeFEAxyzD::NodeIntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) {
    ChNodeFEAxyz::NodeIntStateGatherAcceleration(off_a, a);
    if (!IsFixedD()) {
        a.segment(off_a + 3, 3) = D_dtdt.eigen();
    }
}

void ChNodeFEAxyzD::NodeIntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) {
    ChNodeFEAxyz::NodeIntStateScatterAcceleration(off_a, a);
    if (!IsFixedD()) {
        SetD_dtdt(a.segment(off_a + 3, 3));
    }
}

void ChNodeFEAxyzD::NodeIntStateIncrement(const unsigned int off_x,
                                          ChState& x_new,
                                          const ChState& x,
                                          const unsigned int off_v,
                                          const ChStateDelta& Dv) {
    ChNodeFEAxyz::NodeIntStateIncrement(off_x, x_new, x, off_v, Dv);
    if (!IsFixedD()) {
        x_new(off_x + 3) = x(off_x + 3) + Dv(off_v + 3);
        x_new(off_x + 4) = x(off_x + 4) + Dv(off_v + 4);
        x_new(off_x + 5) = x(off_x + 5) + Dv(off_v + 5);
    }
}

void ChNodeFEAxyzD::NodeIntStateGetIncrement(const unsigned int off_x,
                                             const ChState& x_new,
                                             const ChState& x,
                                             const unsigned int off_v,
                                             ChStateDelta& Dv) {
    ChNodeFEAxyz::NodeIntStateGetIncrement(off_x, x_new, x, off_v, Dv);
    if (!IsFixedD()) {
        Dv(off_v + 3) = x_new(off_x + 3) - x(off_x + 3);
        Dv(off_v + 4) = x_new(off_x + 4) - x(off_x + 4);
        Dv(off_v + 5) = x_new(off_x + 5) - x(off_x + 5);
    }
}

void ChNodeFEAxyzD::NodeIntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) {
    ChNodeFEAxyz::NodeIntLoadResidual_F(off, R, c);
    if (!IsFixedD()) {
        R.segment(off + 3, 3).setZero();  // TODO something about applied nodal torque..
    }
}

void ChNodeFEAxyzD::NodeIntLoadResidual_Mv(const unsigned int off,
                                           ChVectorDynamic<>& R,
                                           const ChVectorDynamic<>& w,
                                           const double c) {
    ChNodeFEAxyz::NodeIntLoadResidual_Mv(off, R, w, c);
    if (!IsFixedD()) {
        R(off + 3) += c * GetMassDiagonalD()(0) * w(off + 3);  // unuseful? mass for D isalways zero..
        R(off + 4) += c * GetMassDiagonalD()(1) * w(off + 4);
        R(off + 5) += c * GetMassDiagonalD()(2) * w(off + 5);
    }
}

void ChNodeFEAxyzD::NodeIntToDescriptor(const unsigned int off_v, const ChStateDelta& v, const ChVectorDynamic<>& R) {
    ChNodeFEAxyz::NodeIntToDescriptor(off_v, v, R);
    if (!IsFixedD()) {
        variables_D->Get_qb().segment(0, 3) = v.segment(off_v + 3, 3);
        variables_D->Get_fb().segment(0, 3) = R.segment(off_v + 3, 3);
    }
}

void ChNodeFEAxyzD::NodeIntFromDescriptor(const unsigned int off_v, ChStateDelta& v) {
    ChNodeFEAxyz::NodeIntFromDescriptor(off_v, v);
    if (!IsFixedD()) {
        v.segment(off_v + 3, 3) = variables_D->Get_qb().segment(0, 3);
    }
}

// -----------------------------------------------------------------------------

void ChNodeFEAxyzD::InjectVariables(ChSystemDescriptor& descriptor) {
    ChNodeFEAxyz::InjectVariables(descriptor);
    if (!IsFixedD()) {
        descriptor.InsertVariables(variables_D);
    }
}

void ChNodeFEAxyzD::VariablesFbReset() {
    ChNodeFEAxyz::VariablesFbReset();
    if (!IsFixedD()) {
        variables_D->Get_fb().setZero();
    }
}

void ChNodeFEAxyzD::VariablesFbLoadForces(double factor) {
    ChNodeFEAxyz::VariablesFbLoadForces(factor);
    ////if (!IsFixedD()) {
    ////    variables_D->Get_fb().segment(3, 3) += VNULL.eigen();  // TODO something related to inertia?
    ////}
}

void ChNodeFEAxyzD::VariablesQbLoadSpeed() {
    ChNodeFEAxyz::VariablesQbLoadSpeed();
    if (!IsFixedD()) {
        variables_D->Get_qb().segment(0, 3) = D_dt.eigen();
    }
}

void ChNodeFEAxyzD::VariablesQbSetSpeed(double step) {
    ChNodeFEAxyz::VariablesQbSetSpeed(step);
    if (!IsFixedD()) {
        ChVector<> oldD_dt = D_dt;
        SetD_dt(variables_D->Get_qb().segment(0, 3));
        if (step) {
            SetD_dtdt((D_dt - oldD_dt) / step);
        }
    }
}

void ChNodeFEAxyzD::VariablesFbIncrementMq() {
    ChNodeFEAxyz::VariablesFbIncrementMq();
    if (!IsFixedD()) {
        variables_D->Compute_inc_Mb_v(variables_D->Get_fb(), variables_D->Get_qb());
    }
}

void ChNodeFEAxyzD::VariablesQbIncrementPosition(double step) {
    ChNodeFEAxyz::VariablesQbIncrementPosition(step);
    if (!IsFixedD()) {
        // ADVANCE POSITION: pos' = pos + dt * vel
        ChVector<> newspeed_D(variables_D->Get_qb().segment(0, 3));
        SetD(GetD() + newspeed_D * step);
    }
}

// -----------------------------------------------------------------------------

void ChNodeFEAxyzD::LoadableGetStateBlock_x(int block_offset, ChState& S) {
    ChNodeFEAxyz::LoadableGetStateBlock_x(block_offset, S);
    if (!IsFixedD()) {
        S.segment(block_offset + 3, 3) = D.eigen();
    }
}

void ChNodeFEAxyzD::LoadableGetStateBlock_w(int block_offset, ChStateDelta& S) {
    ChNodeFEAxyz::LoadableGetStateBlock_w(block_offset, S);
    if (!IsFixedD()) {
        S.segment(block_offset + 3, 3) = D_dt.eigen();
    }
}

void ChNodeFEAxyzD::LoadableStateIncrement(const unsigned int off_x,
                                           ChState& x_new,
                                           const ChState& x,
                                           const unsigned int off_v,
                                           const ChStateDelta& Dv) {
    NodeIntStateIncrement(off_x, x_new, x, off_v, Dv);
}

void ChNodeFEAxyzD::LoadableGetVariables(std::vector<ChVariables*>& vars) {
    ChNodeFEAxyz::LoadableGetVariables(vars);
    if (!IsFixedD()) {
        vars.push_back(variables_D);
    }
}

void ChNodeFEAxyzD::ComputeNF(
    const double U,              // x coordinate of application point in absolute space
    const double V,              // y coordinate of application point in absolute space
    const double W,              // z coordinate of application point in absolute space
    ChVectorDynamic<>& Qi,       // Return result of N'*F  here, maybe with offset block_offset
    double& detJ,                // Return det[J] here
    const ChVectorDynamic<>& F,  // Input F vector, containing Force xyz in absolute coords and a 'pseudo' torque.
    ChVectorDynamic<>* state_x,  // if != 0, update state (pos. part) to this, then evaluate Q
    ChVectorDynamic<>* state_w   // if != 0, update state (speed part) to this, then evaluate Q
) {
    Qi.segment(0, m_dof_actual) = F.segment(0, m_dof_actual);
    detJ = 1;  // not needed because not used in quadrature.
}

// -----------------------------------------------------------------------------

void ChNodeFEAxyzD::ArchiveOUT(ChArchiveOut& archive) {
    // version number
    archive.VersionWrite<ChNodeFEAxyzD>();
    // serialize parent class
    ChNodeFEAxyz::ArchiveOUT(archive);
    // serialize all member data:
    archive << CHNVP(D);
    archive << CHNVP(D_dt);
    archive << CHNVP(D_dtdt);
}

void ChNodeFEAxyzD::ArchiveIN(ChArchiveIn& archive) {
    // version number
    /*int version = */ archive.VersionRead<ChNodeFEAxyzD>();
    // deserialize parent class
    ChNodeFEAxyz::ArchiveIN(archive);
    // stream in all member data:
    archive >> CHNVP(D);
    archive >> CHNVP(D_dt);
    archive >> CHNVP(D_dtdt);
}

}  // end namespace fea
}  // end namespace chrono
