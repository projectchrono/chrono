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

void ChNodeFEAxyzDD::SetFixed(bool fixed) {
    ChNodeFEAxyzD::SetFixed(fixed);
    variables_DD->SetDisabled(fixed);
}

bool ChNodeFEAxyzDD::IsFixed() const {
    return ChNodeFEAxyzD::IsFixed() && variables_DD->IsDisabled();
}

void ChNodeFEAxyzDD::SetFixedDD(bool fixed) {
    variables_DD->SetDisabled(fixed);
}

bool ChNodeFEAxyzDD::IsFixedDD() const {
    return variables_DD->IsDisabled();
}

void ChNodeFEAxyzDD::SetupInitial(ChSystem* system) {
    // If the 3rd derivative vectore is free, ensure the 1st derivative vector is also free.
    if (!IsFixedDD())
        SetFixedD(false);

    ChNodeFEAxyzD::SetupInitial(system);

    if (IsFixed())
        m_dof_actual = 0;
    else if (IsFixedD())
        m_dof_actual = 3;
    else if (IsFixedDD())
        m_dof_actual = 6;
    else
        m_dof_actual = 9;
}

// -----------------------------------------------------------------------------

void ChNodeFEAxyzDD::NodeIntStateGather(const unsigned int off_x,
                                        ChState& x,
                                        const unsigned int off_v,
                                        ChStateDelta& v,
                                        double& T) {
    ChNodeFEAxyzD::NodeIntStateGather(off_x, x, off_v, v, T);
    if (!IsFixedDD()) {
        x.segment(off_x + 6, 3) = DD.eigen();
        v.segment(off_v + 6, 3) = DD_dt.eigen();
    }
}

void ChNodeFEAxyzDD::NodeIntStateScatter(const unsigned int off_x,
                                         const ChState& x,
                                         const unsigned int off_v,
                                         const ChStateDelta& v,
                                         const double T) {
    ChNodeFEAxyzD::NodeIntStateScatter(off_x, x, off_v, v, T);
    if (!IsFixedDD()) {
        SetDD(x.segment(off_x + 6, 3));
        SetDD_dt(v.segment(off_v + 6, 3));
    }
}

void ChNodeFEAxyzDD::NodeIntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) {
    ChNodeFEAxyzD::NodeIntStateGatherAcceleration(off_a, a);
    if (!IsFixedDD()) {
        a.segment(off_a + 6, 3) = DD_dtdt.eigen();
    }
}

void ChNodeFEAxyzDD::NodeIntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) {
    ChNodeFEAxyzD::NodeIntStateScatterAcceleration(off_a, a);
    if (!IsFixedDD()) {
        SetDD_dtdt(a.segment(off_a + 6, 3));
    }
}

void ChNodeFEAxyzDD::NodeIntStateIncrement(const unsigned int off_x,
                                           ChState& x_new,
                                           const ChState& x,
                                           const unsigned int off_v,
                                           const ChStateDelta& Dv) {
    ChNodeFEAxyzD::NodeIntStateIncrement(off_x, x_new, x, off_v, Dv);
    if (!IsFixedDD()) {
        x_new(off_x + 6) = x(off_x + 6) + Dv(off_v + 6);
        x_new(off_x + 7) = x(off_x + 7) + Dv(off_v + 7);
        x_new(off_x + 8) = x(off_x + 8) + Dv(off_v + 8);
    }
}

void ChNodeFEAxyzDD::NodeIntStateGetIncrement(const unsigned int off_x,
                                              const ChState& x_new,
                                              const ChState& x,
                                              const unsigned int off_v,
                                              ChStateDelta& Dv) {
    ChNodeFEAxyzD::NodeIntStateGetIncrement(off_x, x_new, x, off_v, Dv);
    if (!IsFixedDD()) {
        Dv(off_v + 6) = x_new(off_x + 6) - x(off_x + 6);
        Dv(off_v + 7) = x_new(off_x + 7) - x(off_x + 7);
        Dv(off_v + 8) = x_new(off_x + 8) - x(off_x + 8);
    }
}

void ChNodeFEAxyzDD::NodeIntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) {
    ChNodeFEAxyzD::NodeIntLoadResidual_F(off, R, c);
    if (!IsFixedDD()) {
        R.segment(off + 6, 3).setZero();  // TODO something about applied nodal torque..
    }
}

void ChNodeFEAxyzDD::NodeIntLoadResidual_Mv(const unsigned int off,
                                            ChVectorDynamic<>& R,
                                            const ChVectorDynamic<>& w,
                                            const double c) {
    ChNodeFEAxyzD::NodeIntLoadResidual_Mv(off, R, w, c);
    if (!IsFixedDD()) {
        R(off + 6) += c * GetMassDiagonalDD()(0) * w(off + 6);
        R(off + 7) += c * GetMassDiagonalDD()(1) * w(off + 7);
        R(off + 8) += c * GetMassDiagonalDD()(2) * w(off + 8);
    }
}

void ChNodeFEAxyzDD::NodeIntToDescriptor(const unsigned int off_v, const ChStateDelta& v, const ChVectorDynamic<>& R) {
    ChNodeFEAxyzD::NodeIntToDescriptor(off_v, v, R);
    if (!IsFixedDD()) {
        variables_DD->Get_qb().segment(0, 3) = v.segment(off_v + 6, 3);
        variables_DD->Get_fb().segment(0, 3) = R.segment(off_v + 6, 3);
    }
}

void ChNodeFEAxyzDD::NodeIntFromDescriptor(const unsigned int off_v, ChStateDelta& v) {
    ChNodeFEAxyzD::NodeIntFromDescriptor(off_v, v);
    if (!IsFixedDD()) {
        v.segment(off_v + 6, 3) = variables_DD->Get_qb().segment(0, 3);
    }
}

// -----------------------------------------------------------------------------

void ChNodeFEAxyzDD::InjectVariables(ChSystemDescriptor& descriptor) {
    ChNodeFEAxyzD::InjectVariables(descriptor);
    if (!IsFixedDD()) {
        descriptor.InsertVariables(variables_DD);
    }
}

void ChNodeFEAxyzDD::VariablesFbReset() {
    ChNodeFEAxyzD::VariablesFbReset();
    if (!IsFixedDD()) {
        variables_DD->Get_fb().setZero();
    }
}

void ChNodeFEAxyzDD::VariablesFbLoadForces(double factor) {
    ChNodeFEAxyzD::VariablesFbLoadForces(factor);
    ////if (!IsFixedDD()) {
    ////    variables_DD->Get_fb().segment(3, 3) += VNULL.eigen();  // TODO something related to inertia?
    ////}
}

void ChNodeFEAxyzDD::VariablesQbLoadSpeed() {
    ChNodeFEAxyzD::VariablesQbLoadSpeed();
    if (!IsFixedDD()) {
        variables_DD->Get_qb().segment(0, 3) = DD_dt.eigen();
    }
}

void ChNodeFEAxyzDD::VariablesQbSetSpeed(double step) {
    ChNodeFEAxyzD::VariablesQbSetSpeed(step);
    if (!IsFixedDD()) {
        ChVector<> oldDD_dt = DD_dt;
        SetDD_dt(variables_DD->Get_qb().segment(0, 3));
        if (step) {
            SetDD_dtdt((DD_dt - oldDD_dt) / step);
        }
    }
}

void ChNodeFEAxyzDD::VariablesFbIncrementMq() {
    ChNodeFEAxyzD::VariablesFbIncrementMq();
    if (!IsFixedDD()) {
        variables_DD->Compute_inc_Mb_v(variables_DD->Get_fb(), variables_DD->Get_qb());
    }
}

void ChNodeFEAxyzDD::VariablesQbIncrementPosition(double step) {
    ChNodeFEAxyzD::VariablesQbIncrementPosition(step);
    if (!IsFixedDD()) {
        // ADVANCE POSITION: pos' = pos + dt * vel
        ChVector<> newspeed_DD(variables_DD->Get_qb().segment(0, 3));
        SetDD(GetDD() + newspeed_DD * step);
    }
}

// -----------------------------------------------------------------------------

void ChNodeFEAxyzDD::LoadableGetStateBlock_x(int block_offset, ChState& S) {
    ChNodeFEAxyzD::LoadableGetStateBlock_x(block_offset, S);
    if (!IsFixedDD()) {
        S.segment(block_offset + 6, 3) = DD.eigen();
    }
}

void ChNodeFEAxyzDD::LoadableGetStateBlock_w(int block_offset, ChStateDelta& S) {
    ChNodeFEAxyzD::LoadableGetStateBlock_w(block_offset, S);
    if (!IsFixedDD()) {
        S.segment(block_offset + 6, 3) = DD_dt.eigen();
    }
}

void ChNodeFEAxyzDD::LoadableStateIncrement(const unsigned int off_x,
                                            ChState& x_new,
                                            const ChState& x,
                                            const unsigned int off_v,
                                            const ChStateDelta& Dv) {
    NodeIntStateIncrement(off_x, x_new, x, off_v, Dv);
}

void ChNodeFEAxyzDD::LoadableGetVariables(std::vector<ChVariables*>& vars) {
    ChNodeFEAxyzD::LoadableGetVariables(vars);
    if (!IsFixedDD()) {
        vars.push_back(variables_DD);
    }
}

void ChNodeFEAxyzDD::ComputeNF(
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

void ChNodeFEAxyzDD::ArchiveOut(ChArchiveOut& archive) {
    // version number
    archive.VersionWrite<ChNodeFEAxyzDD>();
    // serialize parent class
    ChNodeFEAxyzD::ArchiveOut(archive);
    // serialize all member data:
    archive << CHNVP(DD);
    archive << CHNVP(DD_dt);
    archive << CHNVP(DD_dtdt);
}

void ChNodeFEAxyzDD::ArchiveIn(ChArchiveIn& archive) {
    // version number
    /*int version = */ archive.VersionRead<ChNodeFEAxyzDD>();
    // deserialize parent class
    ChNodeFEAxyzD::ArchiveIn(archive);
    // stream in all member data:
    archive >> CHNVP(DD);
    archive >> CHNVP(DD_dt);
    archive >> CHNVP(DD_dtdt);
}

}  // end namespace fea
}  // end namespace chrono
