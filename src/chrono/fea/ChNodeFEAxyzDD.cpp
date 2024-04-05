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

ChNodeFEAxyzDD::ChNodeFEAxyzDD(ChVector3d initial_pos, ChVector3d initial_dir, ChVector3d initial_curv)
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

void ChNodeFEAxyzDD::ForceToRest() {
    ChNodeFEAxyzD::ForceToRest();

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

void ChNodeFEAxyzDD::SetSlope2Fixed(bool fixed) {
    variables_DD->SetDisabled(fixed);
}

bool ChNodeFEAxyzDD::IsSlope2Fixed() const {
    return variables_DD->IsDisabled();
}

void ChNodeFEAxyzDD::SetupInitial(ChSystem* system) {
    // If the 2nd derivative vectore is free, ensure the 1st derivative vector is also free.
    if (!IsSlope2Fixed())
        SetSlope1Fixed(false);

    ChNodeFEAxyzD::SetupInitial(system);

    if (IsFixed())
        m_dof_actual = 0;
    else if (IsSlope1Fixed())
        m_dof_actual = 3;
    else if (IsSlope2Fixed())
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
    if (!IsSlope2Fixed()) {
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
    if (!IsSlope2Fixed()) {
        SetSlope2(x.segment(off_x + 6, 3));
        SetSlope2Dt(v.segment(off_v + 6, 3));
    }
}

void ChNodeFEAxyzDD::NodeIntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) {
    ChNodeFEAxyzD::NodeIntStateGatherAcceleration(off_a, a);
    if (!IsSlope2Fixed()) {
        a.segment(off_a + 6, 3) = DD_dtdt.eigen();
    }
}

void ChNodeFEAxyzDD::NodeIntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) {
    ChNodeFEAxyzD::NodeIntStateScatterAcceleration(off_a, a);
    if (!IsSlope2Fixed()) {
        SetSlope2Dt2(a.segment(off_a + 6, 3));
    }
}

void ChNodeFEAxyzDD::NodeIntStateIncrement(const unsigned int off_x,
                                           ChState& x_new,
                                           const ChState& x,
                                           const unsigned int off_v,
                                           const ChStateDelta& Dv) {
    ChNodeFEAxyzD::NodeIntStateIncrement(off_x, x_new, x, off_v, Dv);
    if (!IsSlope2Fixed()) {
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
    if (!IsSlope2Fixed()) {
        Dv(off_v + 6) = x_new(off_x + 6) - x(off_x + 6);
        Dv(off_v + 7) = x_new(off_x + 7) - x(off_x + 7);
        Dv(off_v + 8) = x_new(off_x + 8) - x(off_x + 8);
    }
}

void ChNodeFEAxyzDD::NodeIntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) {
    ChNodeFEAxyzD::NodeIntLoadResidual_F(off, R, c);
    if (!IsSlope2Fixed()) {
        R.segment(off + 6, 3).setZero();  // TODO something about applied nodal torque..
    }
}

void ChNodeFEAxyzDD::NodeIntLoadResidual_Mv(const unsigned int off,
                                            ChVectorDynamic<>& R,
                                            const ChVectorDynamic<>& w,
                                            const double c) {
    ChNodeFEAxyzD::NodeIntLoadResidual_Mv(off, R, w, c);
    if (!IsSlope2Fixed()) {
        R(off + 6) += c * GetMassDiagonalSlope2()(0) * w(off + 6);
        R(off + 7) += c * GetMassDiagonalSlope2()(1) * w(off + 7);
        R(off + 8) += c * GetMassDiagonalSlope2()(2) * w(off + 8);
    }
}

void ChNodeFEAxyzDD::NodeIntLoadLumpedMass_Md(const unsigned int off,
                                              ChVectorDynamic<>& Md,
                                              double& error,
                                              const double c) {
    ChNodeFEAxyzD::NodeIntLoadLumpedMass_Md(off, Md, error, c);
    if (!IsSlope2Fixed()) {
        Md(off + 6) += c * GetMassDiagonalSlope2()(0);
        Md(off + 7) += c * GetMassDiagonalSlope2()(1);
        Md(off + 8) += c * GetMassDiagonalSlope2()(2);
    }
}

void ChNodeFEAxyzDD::NodeIntToDescriptor(const unsigned int off_v, const ChStateDelta& v, const ChVectorDynamic<>& R) {
    ChNodeFEAxyzD::NodeIntToDescriptor(off_v, v, R);
    if (!IsSlope2Fixed()) {
        variables_DD->State().segment(0, 3) = v.segment(off_v + 6, 3);
        variables_DD->Force().segment(0, 3) = R.segment(off_v + 6, 3);
    }
}

void ChNodeFEAxyzDD::NodeIntFromDescriptor(const unsigned int off_v, ChStateDelta& v) {
    ChNodeFEAxyzD::NodeIntFromDescriptor(off_v, v);
    if (!IsSlope2Fixed()) {
        v.segment(off_v + 6, 3) = variables_DD->State().segment(0, 3);
    }
}

// -----------------------------------------------------------------------------

void ChNodeFEAxyzDD::InjectVariables(ChSystemDescriptor& descriptor) {
    ChNodeFEAxyzD::InjectVariables(descriptor);
    if (!IsSlope2Fixed()) {
        descriptor.InsertVariables(variables_DD);
    }
}

void ChNodeFEAxyzDD::VariablesFbReset() {
    ChNodeFEAxyzD::VariablesFbReset();
    if (!IsSlope2Fixed()) {
        variables_DD->Force().setZero();
    }
}

void ChNodeFEAxyzDD::VariablesFbLoadForces(double factor) {
    ChNodeFEAxyzD::VariablesFbLoadForces(factor);
    ////if (!IsSlope2Fixed()) {
    ////    variables_DD->Force().segment(3, 3) += VNULL.eigen();  // TODO something related to inertia?
    ////}
}

void ChNodeFEAxyzDD::VariablesQbLoadSpeed() {
    ChNodeFEAxyzD::VariablesQbLoadSpeed();
    if (!IsSlope2Fixed()) {
        variables_DD->State().segment(0, 3) = DD_dt.eigen();
    }
}

void ChNodeFEAxyzDD::VariablesQbSetSpeed(double step) {
    ChNodeFEAxyzD::VariablesQbSetSpeed(step);
    if (!IsSlope2Fixed()) {
        ChVector3d oldDD_dt = DD_dt;
        SetSlope2Dt(variables_DD->State().segment(0, 3));
        if (step) {
            SetSlope2Dt2((DD_dt - oldDD_dt) / step);
        }
    }
}

void ChNodeFEAxyzDD::VariablesFbIncrementMq() {
    ChNodeFEAxyzD::VariablesFbIncrementMq();
    if (!IsSlope2Fixed()) {
        variables_DD->AddMassTimesVector(variables_DD->Force(), variables_DD->State());
    }
}

void ChNodeFEAxyzDD::VariablesQbIncrementPosition(double step) {
    ChNodeFEAxyzD::VariablesQbIncrementPosition(step);
    if (!IsSlope2Fixed()) {
        // ADVANCE POSITION: pos' = pos + dt * vel
        ChVector3d newspeed_DD(variables_DD->State().segment(0, 3));
        SetSlope2(GetSlope2() + newspeed_DD * step);
    }
}

// -----------------------------------------------------------------------------

void ChNodeFEAxyzDD::LoadableGetStateBlockPosLevel(int block_offset, ChState& S) {
    ChNodeFEAxyzD::LoadableGetStateBlockPosLevel(block_offset, S);
    if (!IsSlope2Fixed()) {
        S.segment(block_offset + 6, 3) = DD.eigen();
    }
}

void ChNodeFEAxyzDD::LoadableGetStateBlockVelLevel(int block_offset, ChStateDelta& S) {
    ChNodeFEAxyzD::LoadableGetStateBlockVelLevel(block_offset, S);
    if (!IsSlope2Fixed()) {
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
    if (!IsSlope2Fixed()) {
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
