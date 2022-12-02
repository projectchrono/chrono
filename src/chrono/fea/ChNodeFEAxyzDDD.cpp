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

ChNodeFEAxyzDDD::ChNodeFEAxyzDDD(ChVector<> initial_pos,
                                 ChVector<> initial_dir_u,
                                 ChVector<> initial_dir_v,
                                 ChVector<> initial_dir_w)
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

void ChNodeFEAxyzDDD::SetFixed(bool fixed) {
    ChNodeFEAxyzDD::SetFixed(fixed);
    variables_DDD->SetDisabled(fixed);
}

bool ChNodeFEAxyzDDD::IsFixed() const {
    return ChNodeFEAxyzDD::IsFixed() && variables_DDD->IsDisabled();
}

void ChNodeFEAxyzDDD::SetFixedDDD(bool fixed) {
    variables_DDD->SetDisabled(fixed);
}

bool ChNodeFEAxyzDDD::IsFixedDDD() const {
    return variables_DDD->IsDisabled();
}

void ChNodeFEAxyzDDD::SetupInitial(ChSystem* system) {
    // If the 3rd derivative vectore is free, ensure all other derivative vectors are also free.
    if (!IsFixedDDD())
        SetFixedDD(false);

    ChNodeFEAxyzDD::SetupInitial(system);

    if (IsFixed())
        m_dof_actual = 0;
    else if (IsFixedD())
        m_dof_actual = 3;
    else if (IsFixedDD())
        m_dof_actual = 6;
    else if (IsFixedDDD())
        m_dof_actual = 9;
    else
        m_dof_actual = 12;
}

// -----------------------------------------------------------------------------

void ChNodeFEAxyzDDD::NodeIntStateGather(const unsigned int off_x,
                                         ChState& x,
                                         const unsigned int off_v,
                                         ChStateDelta& v,
                                         double& T) {
    ChNodeFEAxyzDD::NodeIntStateGather(off_x, x, off_v, v, T);

    if (!IsFixedDDD()) {
        x.segment(off_x + 9, 3) = DDD.eigen();
        v.segment(off_v + 9, 3) = DDD_dt.eigen();
    }
}

void ChNodeFEAxyzDDD::NodeIntStateScatter(const unsigned int off_x,
                                          const ChState& x,
                                          const unsigned int off_v,
                                          const ChStateDelta& v,
                                          const double T) {
    ChNodeFEAxyzDD::NodeIntStateScatter(off_x, x, off_v, v, T);
    if (!IsFixedDDD()) {
        SetDDD(x.segment(off_x + 9, 3));
        SetDDD_dt(v.segment(off_v + 9, 3));
    }
}

void ChNodeFEAxyzDDD::NodeIntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) {
    ChNodeFEAxyzDD::NodeIntStateGatherAcceleration(off_a, a);
    if (!IsFixedDDD()) {
        a.segment(off_a + 9, 3) = DDD_dtdt.eigen();
    }
}

void ChNodeFEAxyzDDD::NodeIntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) {
    ChNodeFEAxyzDD::NodeIntStateScatterAcceleration(off_a, a);
    SetDDD_dtdt(a.segment(off_a + 9, 3));
}

void ChNodeFEAxyzDDD::NodeIntStateIncrement(const unsigned int off_x,
                                            ChState& x_new,
                                            const ChState& x,
                                            const unsigned int off_v,
                                            const ChStateDelta& Dv) {
    ChNodeFEAxyzDD::NodeIntStateIncrement(off_x, x_new, x, off_v, Dv);
    if (!IsFixedDDD()) {
        x_new(off_x + 9) = x(off_x + 9) + Dv(off_v + 9);
        x_new(off_x + 10) = x(off_x + 10) + Dv(off_v + 10);
        x_new(off_x + 11) = x(off_x + 11) + Dv(off_v + 11);
    }
}

void ChNodeFEAxyzDDD::NodeIntStateGetIncrement(const unsigned int off_x,
                                               const ChState& x_new,
                                               const ChState& x,
                                               const unsigned int off_v,
                                               ChStateDelta& Dv) {
    ChNodeFEAxyzDD::NodeIntStateGetIncrement(off_x, x_new, x, off_v, Dv);
    if (!IsFixedDDD()) {
        Dv(off_v + 9) = x_new(off_x + 9) - x(off_x + 9);
        Dv(off_v + 10) = x_new(off_x + 10) - x(off_x + 10);
        Dv(off_v + 11) = x_new(off_x + 11) - x(off_x + 11);
    }
}

void ChNodeFEAxyzDDD::NodeIntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) {
    ChNodeFEAxyzDD::NodeIntLoadResidual_F(off, R, c);
    if (!IsFixedDDD()) {
        R.segment(off + 9, 3).setZero();  // TODO something about applied nodal torque..
    }
}

void ChNodeFEAxyzDDD::NodeIntLoadResidual_Mv(const unsigned int off,
                                             ChVectorDynamic<>& R,
                                             const ChVectorDynamic<>& w,
                                             const double c) {
    ChNodeFEAxyzDD::NodeIntLoadResidual_Mv(off, R, w, c);
    if (!IsFixedDDD()) {
        R(off + 9) += c * GetMassDiagonalDDD()(0) * w(off + 9);
        R(off + 10) += c * GetMassDiagonalDDD()(1) * w(off + 10);
        R(off + 11) += c * GetMassDiagonalDDD()(2) * w(off + 11);
    }
}

void ChNodeFEAxyzDDD::NodeIntToDescriptor(const unsigned int off_v, const ChStateDelta& v, const ChVectorDynamic<>& R) {
    ChNodeFEAxyzDD::NodeIntToDescriptor(off_v, v, R);
    if (!IsFixedDDD()) {
        variables_DDD->Get_qb().segment(0, 3) = v.segment(off_v + 9, 3);
        variables_DDD->Get_fb().segment(0, 3) = R.segment(off_v + 9, 3);
    }
}

void ChNodeFEAxyzDDD::NodeIntFromDescriptor(const unsigned int off_v, ChStateDelta& v) {
    ChNodeFEAxyzDD::NodeIntFromDescriptor(off_v, v);
    if (!IsFixedDDD()) {
        v.segment(off_v + 9, 3) = variables_DDD->Get_qb().segment(0, 3);
    }
}

// -----------------------------------------------------------------------------

void ChNodeFEAxyzDDD::InjectVariables(ChSystemDescriptor& descriptor) {
    ChNodeFEAxyzDD::InjectVariables(descriptor);
    if (!IsFixedDDD()) {
        descriptor.InsertVariables(variables_DDD);
    }
}

void ChNodeFEAxyzDDD::VariablesFbReset() {
    ChNodeFEAxyzDD::VariablesFbReset();
    if (!IsFixedDDD()) {
        variables_DDD->Get_fb().setZero();
    }
}

void ChNodeFEAxyzDDD::VariablesFbLoadForces(double factor) {
    ChNodeFEAxyzDD::VariablesFbLoadForces(factor);
    ////if (!IsFixedDDD()) {
    ////    variables_DDD->Get_fb().segment(3, 3) += VNULL.eigen();  // TODO something related to inertia?
    ////}
}

void ChNodeFEAxyzDDD::VariablesQbLoadSpeed() {
    ChNodeFEAxyzDD::VariablesQbLoadSpeed();
    if (!IsFixedDDD()) {
        variables_DDD->Get_qb().segment(0, 3) = DDD_dt.eigen();
    }
}

void ChNodeFEAxyzDDD::VariablesQbSetSpeed(double step) {
    ChNodeFEAxyzDD::VariablesQbSetSpeed(step);
    if (!IsFixedDDD()) {
        ChVector<> oldDDD_dt = DDD_dt;
        SetDDD_dt(variables_DDD->Get_qb().segment(0, 3));
        if (step) {
            SetDDD_dtdt((DDD_dt - oldDDD_dt) / step);
        }
    }
}

void ChNodeFEAxyzDDD::VariablesFbIncrementMq() {
    ChNodeFEAxyzDD::VariablesFbIncrementMq();
    if (!IsFixedDDD()) {
        variables_DDD->Compute_inc_Mb_v(variables_DDD->Get_fb(), variables_DDD->Get_qb());
    }
}

void ChNodeFEAxyzDDD::VariablesQbIncrementPosition(double step) {
    ChNodeFEAxyzDD::VariablesQbIncrementPosition(step);
    if (!IsFixedDDD()) {
        // ADVANCE POSITION: pos' = pos + dt * vel
        ChVector<> newspeed_DDD(variables_DDD->Get_qb().segment(0, 3));
        SetDDD(GetDDD() + newspeed_DDD * step);
    }
}

// -----------------------------------------------------------------------------

void ChNodeFEAxyzDDD::LoadableGetStateBlock_x(int block_offset, ChState& S) {
    ChNodeFEAxyzDD::LoadableGetStateBlock_x(block_offset, S);
    if (!IsFixedDDD()) {
        S.segment(block_offset + 9, 3) = DDD.eigen();
    }
}

void ChNodeFEAxyzDDD::LoadableGetStateBlock_w(int block_offset, ChStateDelta& S) {
    ChNodeFEAxyzDD::LoadableGetStateBlock_w(block_offset, S);
    if (!IsFixedDDD()) {
        S.segment(block_offset + 9, 3) = DDD_dt.eigen();
    }
}

void ChNodeFEAxyzDDD::LoadableStateIncrement(const unsigned int off_x,
                                             ChState& x_new,
                                             const ChState& x,
                                             const unsigned int off_v,
                                             const ChStateDelta& Dv) {
    NodeIntStateIncrement(off_x, x_new, x, off_v, Dv);
}

void ChNodeFEAxyzDDD::LoadableGetVariables(std::vector<ChVariables*>& vars) {
    ChNodeFEAxyzDD::LoadableGetVariables(vars);
    if (!IsFixedDDD()) {
        vars.push_back(&Variables_DDD());
    }
}

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
    Qi.segment(0, m_dof_actual) = F.segment(0, m_dof_actual);
    detJ = 1;  // not needed because not used in quadrature.
}

// -----------------------------------------------------------------------------

void ChNodeFEAxyzDDD::ArchiveOUT(ChArchiveOut& archive) {
    // version number
    archive.VersionWrite<ChNodeFEAxyzDDD>();
    // serialize parent class
    ChNodeFEAxyzDD::ArchiveOUT(archive);
    // serialize all member data:
    archive << CHNVP(DDD);
    archive << CHNVP(DDD_dt);
    archive << CHNVP(DDD_dtdt);
}

void ChNodeFEAxyzDDD::ArchiveIN(ChArchiveIn& archive) {
    // version number
    /*int version = */ archive.VersionRead<ChNodeFEAxyzDDD>();
    // deserialize parent class
    ChNodeFEAxyzDD::ArchiveIN(archive);
    // stream in all member data:
    archive >> CHNVP(DDD);
    archive >> CHNVP(DDD_dt);
    archive >> CHNVP(DDD_dtdt);
}

}  // end namespace fea
}  // end namespace chrono
