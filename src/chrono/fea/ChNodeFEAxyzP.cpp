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

#include "chrono/fea/ChNodeFEAxyzP.h"

namespace chrono {
namespace fea {

ChNodeFEAxyzP::ChNodeFEAxyzP(ChVector3d initial_pos) : pos(initial_pos), P(0), P_dt(0), F(0) {
    variables.GetMass()(0) = 0;
}

ChNodeFEAxyzP::ChNodeFEAxyzP(const ChNodeFEAxyzP& other) : ChNodeFEAbase(other) {
    pos = other.pos;
    P = other.P;
    P_dt = other.P_dt;
    F = other.F;
    variables = other.variables;
}

// -----------------------------------------------------------------------------

ChNodeFEAxyzP& ChNodeFEAxyzP::operator=(const ChNodeFEAxyzP& other) {
    if (&other == this)
        return *this;

    ChNodeFEAbase::operator=(other);

    pos = other.pos;
    P = other.P;
    P_dt = other.P_dt;
    F = other.F;
    variables = other.variables;
    return *this;
}

// -----------------------------------------------------------------------------

void ChNodeFEAxyzP::SetFixed(bool fixed) {
    variables.SetDisabled(fixed);
}

bool ChNodeFEAxyzP::IsFixed() const {
    return variables.IsDisabled();
}

void ChNodeFEAxyzP::Relax() {
    // no special effect here, just resets scalar field.
    P = 0;
    P_dt = 0;
}

// -----------------------------------------------------------------------------

void ChNodeFEAxyzP::NodeIntStateGather(const unsigned int off_x,
                                       ChState& x,
                                       const unsigned int off_v,
                                       ChStateDelta& v,
                                       double& T) {
    x(off_x) = P;
    v(off_v) = P_dt;
}

void ChNodeFEAxyzP::NodeIntStateScatter(const unsigned int off_x,
                                        const ChState& x,
                                        const unsigned int off_v,
                                        const ChStateDelta& v,
                                        const double T) {
    P = x(off_x);
    P_dt = v(off_v);
}

void ChNodeFEAxyzP::NodeIntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) {
    // a(off_a) = P_dtdt; // NOT NEEDED?
}

void ChNodeFEAxyzP::NodeIntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) {
    // P_dtdt = (a(off_a)); // NOT NEEDED?
}

void ChNodeFEAxyzP::NodeIntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) {
    R(off) += F * c;
}

void ChNodeFEAxyzP::NodeIntLoadResidual_Mv(const unsigned int off,
                                           ChVectorDynamic<>& R,
                                           const ChVectorDynamic<>& w,
                                           const double c) {
    R(off) += c * GetMass() * w(off);
}

void ChNodeFEAxyzP::NodeIntLoadLumpedMass_Md(const unsigned int off,
                                             ChVectorDynamic<>& Md,
                                             double& error,
                                             const double c) {
    Md(off) += c * GetMass();
}

void ChNodeFEAxyzP::NodeIntToDescriptor(const unsigned int off_v, const ChStateDelta& v, const ChVectorDynamic<>& R) {
    variables.State()(0) = v(off_v);
    variables.Force()(0) = R(off_v);
}

void ChNodeFEAxyzP::NodeIntFromDescriptor(const unsigned int off_v, ChStateDelta& v) {
    v(off_v) = variables.State()(0);
}

// -----------------------------------------------------------------------------

void ChNodeFEAxyzP::InjectVariables(ChSystemDescriptor& descriptor) {
    descriptor.InsertVariables(&variables);
}

void ChNodeFEAxyzP::VariablesFbReset() {
    variables.Force()(0) = 0;
}

void ChNodeFEAxyzP::VariablesFbLoadForces(double factor) {
    if (variables.IsDisabled())
        return;
    variables.Force()(0) += F * factor;
}

void ChNodeFEAxyzP::VariablesQbLoadSpeed() {
    if (variables.IsDisabled())
        return;
    // not really a 'speed', just the field derivative (may be used in incremental solver)
    variables.State()(0) = P_dt;
}

void ChNodeFEAxyzP::VariablesQbSetSpeed(double step) {
    if (variables.IsDisabled())
        return;
    // not really a 'speed', just the field derivative (may be used in incremental solver)
    P_dt = variables.State()(0);
}

void ChNodeFEAxyzP::VariablesFbIncrementMq() {
    if (variables.IsDisabled())
        return;
    variables.AddMassTimesVector(variables.Force(), variables.State());
}

void ChNodeFEAxyzP::VariablesQbIncrementPosition(double step) {
    if (variables.IsDisabled())
        return;

    double pseudospeed = variables.State()(0);

    // ADVANCE FIELD: pos' = pos + dt * vel
    P = P + pseudospeed * step;
}

// -----------------------------------------------------------------------------

void ChNodeFEAxyzP::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChNodeFEAxyzP>();
    // serialize parent class
    ChNodeFEAbase::ArchiveOut(archive_out);
    // serialize all member data:
    archive_out << CHNVP(P);
    archive_out << CHNVP(P_dt);
    archive_out << CHNVP(F);
}

void ChNodeFEAxyzP::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChNodeFEAxyzP>();
    // deserialize parent class
    ChNodeFEAbase::ArchiveIn(archive_in);
    // stream in all member data:
    archive_in >> CHNVP(P);
    archive_in >> CHNVP(P_dt);
    archive_in >> CHNVP(F);
}

}  // end namespace fea
}  // end namespace chrono
