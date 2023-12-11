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

#include "chrono/fea/ChNodeFEAxyz.h"

namespace chrono {
namespace fea {

ChNodeFEAxyz::ChNodeFEAxyz(ChVector<> initial_pos) : ChNodeXYZ(initial_pos), X0(initial_pos), Force(VNULL) {
    variables.SetNodeMass(0);
}

ChNodeFEAxyz::ChNodeFEAxyz(const ChNodeFEAxyz& other) : ChNodeFEAbase(other), ChNodeXYZ(other) {
    X0 = other.X0;
    Force = other.Force;
    variables = other.variables;
}

ChNodeFEAxyz& ChNodeFEAxyz::operator=(const ChNodeFEAxyz& other) {
    if (&other == this)
        return *this;

    ChNodeFEAbase::operator=(other);
    ChNodeXYZ::operator=(other);

    X0 = other.X0;
    Force = other.Force;
    variables = other.variables;

    return *this;
}

// -----------------------------------------------------------------------------

void ChNodeFEAxyz::SetFixed(bool fixed) {
    variables.SetDisabled(fixed);
}

bool ChNodeFEAxyz::IsFixed() const {
    return variables.IsDisabled();
}

void ChNodeFEAxyz::Relax() {
    X0 = pos;
    SetNoSpeedNoAcceleration();
}

void ChNodeFEAxyz::SetNoSpeedNoAcceleration() {
    pos_dt = VNULL;
    pos_dtdt = VNULL;
}

// -----------------------------------------------------------------------------

void ChNodeFEAxyz::NodeIntStateGather(const unsigned int off_x,
                                      ChState& x,
                                      const unsigned int off_v,
                                      ChStateDelta& v,
                                      double& T) {
    x.segment(off_x, 3) = pos.eigen();
    v.segment(off_v, 3) = pos_dt.eigen();
}

void ChNodeFEAxyz::NodeIntStateScatter(const unsigned int off_x,
                                       const ChState& x,
                                       const unsigned int off_v,
                                       const ChStateDelta& v,
                                       const double T) {
    SetPos(x.segment(off_x, 3));
    SetPos_dt(v.segment(off_v, 3));
}

void ChNodeFEAxyz::NodeIntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) {
    a.segment(off_a, 3) = pos_dtdt.eigen();
}

void ChNodeFEAxyz::NodeIntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) {
    SetPos_dtdt(a.segment(off_a, 3));
}

void ChNodeFEAxyz::NodeIntStateIncrement(const unsigned int off_x,
                                         ChState& x_new,
                                         const ChState& x,
                                         const unsigned int off_v,
                                         const ChStateDelta& Dv) {
    x_new(off_x + 0) = x(off_x + 0) + Dv(off_v + 0);
    x_new(off_x + 1) = x(off_x + 1) + Dv(off_v + 1);
    x_new(off_x + 2) = x(off_x + 2) + Dv(off_v + 2);
}

void ChNodeFEAxyz::NodeIntStateGetIncrement(const unsigned int off_x,
                                            const ChState& x_new,
                                            const ChState& x,
                                            const unsigned int off_v,
                                            ChStateDelta& Dv) {
    Dv(off_v + 0) = x_new(off_x + 0) - x(off_x + 0);
    Dv(off_v + 1) = x_new(off_x + 1) - x(off_x + 1);
    Dv(off_v + 2) = x_new(off_x + 2) - x(off_x + 2);
}

void ChNodeFEAxyz::NodeIntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) {
    R.segment(off, 3) += c * Force.eigen();
}

void ChNodeFEAxyz::NodeIntLoadResidual_Mv(const unsigned int off,
                                          ChVectorDynamic<>& R,
                                          const ChVectorDynamic<>& w,
                                          const double c) {
    R(off + 0) += c * GetMass() * w(off + 0);
    R(off + 1) += c * GetMass() * w(off + 1);
    R(off + 2) += c * GetMass() * w(off + 2);
}

void ChNodeFEAxyz::NodeIntLoadLumpedMass_Md(const unsigned int off,
                                            ChVectorDynamic<>& Md,
                                            double& error,
                                            const double c) {
    Md(off + 0) += c * GetMass();
    Md(off + 1) += c * GetMass();
    Md(off + 2) += c * GetMass();
}

void ChNodeFEAxyz::NodeIntToDescriptor(const unsigned int off_v, const ChStateDelta& v, const ChVectorDynamic<>& R) {
    variables.Get_qb() = v.segment(off_v, 3);
    variables.Get_fb() = R.segment(off_v, 3);
}

void ChNodeFEAxyz::NodeIntFromDescriptor(const unsigned int off_v, ChStateDelta& v) {
    v.segment(off_v, 3) = variables.Get_qb();
}

// -----------------------------------------------------------------------------

void ChNodeFEAxyz::InjectVariables(ChSystemDescriptor& descriptor) {
    descriptor.InsertVariables(&variables);
}

void ChNodeFEAxyz::VariablesFbReset() {
    variables.Get_fb().setZero();
}

void ChNodeFEAxyz::VariablesFbLoadForces(double factor) {
    variables.Get_fb() += factor * Force.eigen();
}

void ChNodeFEAxyz::VariablesQbLoadSpeed() {
    variables.Get_qb() = pos_dt.eigen();
}

void ChNodeFEAxyz::VariablesQbSetSpeed(double step) {
    ChVector<> old_dt = pos_dt;
    SetPos_dt(variables.Get_qb().segment(0, 3));
    if (step) {
        SetPos_dtdt((pos_dt - old_dt) / step);
    }
}

void ChNodeFEAxyz::VariablesFbIncrementMq() {
    variables.Compute_inc_Mb_v(variables.Get_fb(), variables.Get_qb());
}

void ChNodeFEAxyz::VariablesQbIncrementPosition(double step) {
    ChVector<> newspeed = variables.Get_qb().segment(0, 3);

    // ADVANCE POSITION: pos' = pos + dt * vel
    SetPos(GetPos() + newspeed * step);
}

// -----------------------------------------------------------------------------

void ChNodeFEAxyz::ArchiveOut(ChArchiveOut& archive) {
    // version number
    archive.VersionWrite<ChNodeFEAxyz>();
    // serialize parent class
    ChNodeFEAbase::ArchiveOut(archive);
    // serialize parent class
    ChNodeXYZ::ArchiveOut(archive);
    // serialize all member data:
    archive << CHNVP(X0);
    archive << CHNVP(Force);
}

void ChNodeFEAxyz::ArchiveIn(ChArchiveIn& archive) {
    // version number
    /*int version = */ archive.VersionRead<ChNodeFEAxyz>();
    // deserialize parent class
    ChNodeFEAbase::ArchiveIn(archive);
    // serialize parent class
    ChNodeXYZ::ArchiveIn(archive);
    // stream in all member data:
    archive >> CHNVP(X0);
    archive >> CHNVP(Force);
}

}  // end namespace fea
}  // end namespace chrono
