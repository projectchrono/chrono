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

ChNodeFEAxyzDDD::ChNodeFEAxyzDDD(ChVector3d initial_pos,
                                 ChVector3d initial_dir_u,
                                 ChVector3d initial_dir_v,
                                 ChVector3d initial_dir_w)
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

void ChNodeFEAxyzDDD::ForceToRest() {
    ChNodeFEAxyzDD::ForceToRest();

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

void ChNodeFEAxyzDDD::SetSlope3Fixed(bool fixed) {
    variables_DDD->SetDisabled(fixed);
}

bool ChNodeFEAxyzDDD::IsSlope3Fixed() const {
    return variables_DDD->IsDisabled();
}

void ChNodeFEAxyzDDD::SetupInitial(ChSystem* system) {
    // If the 3rd derivative vectore is free, ensure all other derivative vectors are also free.
    if (!IsSlope3Fixed())
        SetSlope2Fixed(false);

    ChNodeFEAxyzDD::SetupInitial(system);

    if (IsFixed())
        m_dof_actual = 0;
    else if (IsSlope1Fixed())
        m_dof_actual = 3;
    else if (IsSlope2Fixed())
        m_dof_actual = 6;
    else if (IsSlope3Fixed())
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

    if (!IsSlope3Fixed()) {
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
    if (!IsSlope3Fixed()) {
        SetSlope3(x.segment(off_x + 9, 3));
        SetSlope3Dt(v.segment(off_v + 9, 3));
    }
}

void ChNodeFEAxyzDDD::NodeIntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) {
    ChNodeFEAxyzDD::NodeIntStateGatherAcceleration(off_a, a);
    if (!IsSlope3Fixed()) {
        a.segment(off_a + 9, 3) = DDD_dtdt.eigen();
    }
}

void ChNodeFEAxyzDDD::NodeIntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) {
    ChNodeFEAxyzDD::NodeIntStateScatterAcceleration(off_a, a);
    SetSlope3Dt2(a.segment(off_a + 9, 3));
}

void ChNodeFEAxyzDDD::NodeIntStateIncrement(const unsigned int off_x,
                                            ChState& x_new,
                                            const ChState& x,
                                            const unsigned int off_v,
                                            const ChStateDelta& Dv) {
    ChNodeFEAxyzDD::NodeIntStateIncrement(off_x, x_new, x, off_v, Dv);
    if (!IsSlope3Fixed()) {
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
    if (!IsSlope3Fixed()) {
        Dv(off_v + 9) = x_new(off_x + 9) - x(off_x + 9);
        Dv(off_v + 10) = x_new(off_x + 10) - x(off_x + 10);
        Dv(off_v + 11) = x_new(off_x + 11) - x(off_x + 11);
    }
}

void ChNodeFEAxyzDDD::NodeIntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) {
    ChNodeFEAxyzDD::NodeIntLoadResidual_F(off, R, c);
    if (!IsSlope3Fixed()) {
        R.segment(off + 9, 3).setZero();  // TODO something about applied nodal torque..
    }
}

void ChNodeFEAxyzDDD::NodeIntLoadResidual_Mv(const unsigned int off,
                                             ChVectorDynamic<>& R,
                                             const ChVectorDynamic<>& w,
                                             const double c) {
    ChNodeFEAxyzDD::NodeIntLoadResidual_Mv(off, R, w, c);
    if (!IsSlope3Fixed()) {
        R(off + 9) += c * GetMassDiagonalSlope3()(0) * w(off + 9);
        R(off + 10) += c * GetMassDiagonalSlope3()(1) * w(off + 10);
        R(off + 11) += c * GetMassDiagonalSlope3()(2) * w(off + 11);
    }
}

void ChNodeFEAxyzDDD::NodeIntLoadLumpedMass_Md(const unsigned int off,
                                               ChVectorDynamic<>& Md,
                                               double& error,
                                               const double c) {
    ChNodeFEAxyzDD::NodeIntLoadLumpedMass_Md(off, Md, error, c);
    if (!IsSlope3Fixed()) {
        Md(off + 9) += c * GetMassDiagonalSlope3()(0);
        Md(off + 10) += c * GetMassDiagonalSlope3()(1);
        Md(off + 11) += c * GetMassDiagonalSlope3()(2);
    }
}

void ChNodeFEAxyzDDD::NodeIntToDescriptor(const unsigned int off_v, const ChStateDelta& v, const ChVectorDynamic<>& R) {
    ChNodeFEAxyzDD::NodeIntToDescriptor(off_v, v, R);
    if (!IsSlope3Fixed()) {
        variables_DDD->State().segment(0, 3) = v.segment(off_v + 9, 3);
        variables_DDD->Force().segment(0, 3) = R.segment(off_v + 9, 3);
    }
}

void ChNodeFEAxyzDDD::NodeIntFromDescriptor(const unsigned int off_v, ChStateDelta& v) {
    ChNodeFEAxyzDD::NodeIntFromDescriptor(off_v, v);
    if (!IsSlope3Fixed()) {
        v.segment(off_v + 9, 3) = variables_DDD->State().segment(0, 3);
    }
}

// -----------------------------------------------------------------------------

void ChNodeFEAxyzDDD::InjectVariables(ChSystemDescriptor& descriptor) {
    ChNodeFEAxyzDD::InjectVariables(descriptor);
    if (!IsSlope3Fixed()) {
        descriptor.InsertVariables(variables_DDD);
    }
}

void ChNodeFEAxyzDDD::VariablesFbReset() {
    ChNodeFEAxyzDD::VariablesFbReset();
    if (!IsSlope3Fixed()) {
        variables_DDD->Force().setZero();
    }
}

void ChNodeFEAxyzDDD::VariablesFbLoadForces(double factor) {
    ChNodeFEAxyzDD::VariablesFbLoadForces(factor);
    ////if (!IsSlope3Fixed()) {
    ////    variables_DDD->Force().segment(3, 3) += VNULL.eigen();  // TODO something related to inertia?
    ////}
}

void ChNodeFEAxyzDDD::VariablesQbLoadSpeed() {
    ChNodeFEAxyzDD::VariablesQbLoadSpeed();
    if (!IsSlope3Fixed()) {
        variables_DDD->State().segment(0, 3) = DDD_dt.eigen();
    }
}

void ChNodeFEAxyzDDD::VariablesQbSetSpeed(double step) {
    ChNodeFEAxyzDD::VariablesQbSetSpeed(step);
    if (!IsSlope3Fixed()) {
        ChVector3d oldDDD_dt = DDD_dt;
        SetSlope3Dt(variables_DDD->State().segment(0, 3));
        if (step) {
            SetSlope3Dt2((DDD_dt - oldDDD_dt) / step);
        }
    }
}

void ChNodeFEAxyzDDD::VariablesFbIncrementMq() {
    ChNodeFEAxyzDD::VariablesFbIncrementMq();
    if (!IsSlope3Fixed()) {
        variables_DDD->AddMassTimesVector(variables_DDD->Force(), variables_DDD->State());
    }
}

void ChNodeFEAxyzDDD::VariablesQbIncrementPosition(double step) {
    ChNodeFEAxyzDD::VariablesQbIncrementPosition(step);
    if (!IsSlope3Fixed()) {
        // ADVANCE POSITION: pos' = pos + dt * vel
        ChVector3d newspeed_DDD(variables_DDD->State().segment(0, 3));
        SetSlope3(GetSlope3() + newspeed_DDD * step);
    }
}

// -----------------------------------------------------------------------------

void ChNodeFEAxyzDDD::LoadableGetStateBlockPosLevel(int block_offset, ChState& S) {
    ChNodeFEAxyzDD::LoadableGetStateBlockPosLevel(block_offset, S);
    if (!IsSlope3Fixed()) {
        S.segment(block_offset + 9, 3) = DDD.eigen();
    }
}

void ChNodeFEAxyzDDD::LoadableGetStateBlockVelLevel(int block_offset, ChStateDelta& S) {
    ChNodeFEAxyzDD::LoadableGetStateBlockVelLevel(block_offset, S);
    if (!IsSlope3Fixed()) {
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
    if (!IsSlope3Fixed()) {
        vars.push_back(&VariablesSlope3());
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

void ChNodeFEAxyzDDD::ArchiveOut(ChArchiveOut& archive) {
    // version number
    archive.VersionWrite<ChNodeFEAxyzDDD>();
    // serialize parent class
    ChNodeFEAxyzDD::ArchiveOut(archive);
    // serialize all member data:
    archive << CHNVP(DDD);
    archive << CHNVP(DDD_dt);
    archive << CHNVP(DDD_dtdt);
}

void ChNodeFEAxyzDDD::ArchiveIn(ChArchiveIn& archive) {
    // version number
    /*int version = */ archive.VersionRead<ChNodeFEAxyzDDD>();
    // deserialize parent class
    ChNodeFEAxyzDD::ArchiveIn(archive);
    // stream in all member data:
    archive >> CHNVP(DDD);
    archive >> CHNVP(DDD_dt);
    archive >> CHNVP(DDD_dtdt);
}

}  // end namespace fea
}  // end namespace chrono
