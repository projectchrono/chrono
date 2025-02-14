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

#include "ChNodeFEAxyzPP.h"

namespace chrono {
namespace fea {

ChNodeFEAxyzPP::ChNodeFEAxyzPP(ChVector3d initial_pos) : pos(initial_pos), P(0, 0, 0), P_dt(0, 0, 0), F(0, 0, 0) {
    variables.SetNodeMass(0);
}

ChNodeFEAxyzPP::ChNodeFEAxyzPP(const ChNodeFEAxyzPP& other) : ChNodeFEAbase(other), ChNodeXYZ(other) {
    pos = other.pos;
    P = other.P;
    P_dt = other.P_dt;
    F = other.F;
    variables = other.variables;
}

// -----------------------------------------------------------------------------

ChNodeFEAxyzPP& ChNodeFEAxyzPP::operator=(const ChNodeFEAxyzPP& other) {
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

void ChNodeFEAxyzPP::SetFixed(bool fixed) {
    variables.SetDisabled(fixed);
}

bool ChNodeFEAxyzPP::IsFixed() const {
    return variables.IsDisabled();
}

void ChNodeFEAxyzPP::Relax() {
    P = (0, 0, 0);
    P_dt = (0, 0, 0);
}

// -----------------------------------------------------------------------------

void ChNodeFEAxyzPP::NodeIntStateGather(const unsigned int off_x,
                                       ChState& x,
                                       const unsigned int off_v,
                                       ChStateDelta& v,
                                       double& T) {
    x.segment(off_x, 3) = P.eigen();
    v.segment(off_v, 3) = P_dt.eigen();
}

void ChNodeFEAxyzPP::NodeIntStateScatter(const unsigned int off_x,
                                        const ChState& x,
                                        const unsigned int off_v,
                                        const ChStateDelta& v,
                                        const double T) {
    SetFieldVal(x.segment(off_x, 3));
    SetFieldValDt(v.segment(off_v, 3));
}

void ChNodeFEAxyzPP::NodeIntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) {
    // a(off_a) = P_dtdt; // NOT NEEDED?
}

void ChNodeFEAxyzPP::NodeIntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) {
    // P_dtdt = (a(off_a)); // NOT NEEDED?
}

void ChNodeFEAxyzPP::NodeIntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) {
    R.segment(off, 3) += c * F.eigen();
}

void ChNodeFEAxyzPP::NodeIntLoadResidual_Mv(const unsigned int off,
                                           ChVectorDynamic<>& R,
                                           const ChVectorDynamic<>& w,
                                           const double c) {
    R(off + 0) += c * GetMass() * w(off + 0);
    R(off + 1) += c * GetMass() * w(off + 1);
    R(off + 2) += c * GetMass() * w(off + 2);
}

void ChNodeFEAxyzPP::NodeIntLoadLumpedMass_Md(const unsigned int off,
                                             ChVectorDynamic<>& Md,
                                             double& error,
                                             const double c) {
    Md(off + 0) += c * GetMass();
    Md(off + 1) += c * GetMass();
    Md(off + 2) += c * GetMass();
}

void ChNodeFEAxyzPP::NodeIntToDescriptor(const unsigned int off_v, const ChStateDelta& v, const ChVectorDynamic<>& R) {
    variables.State() = v.segment(off_v, 3);
    variables.Force() = R.segment(off_v, 3);
}

void ChNodeFEAxyzPP::NodeIntFromDescriptor(const unsigned int off_v, ChStateDelta& v) {
    v.segment(off_v, 3) = variables.State();
}

// -----------------------------------------------------------------------------

void ChNodeFEAxyzPP::InjectVariables(ChSystemDescriptor& descriptor) {
    descriptor.InsertVariables(&variables);
}

void ChNodeFEAxyzPP::VariablesFbReset() {
    variables.Force().setZero();
}

void ChNodeFEAxyzPP::VariablesFbLoadForces(double factor) {
    if (variables.IsDisabled())
        return;
    variables.Force()(0) += F[0] * factor;
    variables.Force()(1) += F[1] * factor;
    variables.Force()(2) += F[2] * factor;
}

void ChNodeFEAxyzPP::VariablesQbLoadSpeed() {
    if (variables.IsDisabled())
        return;

    variables.State()(0) = P_dt[0];
    variables.State()(1) = P_dt[1];
    variables.State()(2) = P_dt[2];
}

void ChNodeFEAxyzPP::VariablesQbSetSpeed(double step) {
    if (variables.IsDisabled())
        return;
    // not really a 'speed', just the field derivative (may be used in incremental solver)
    P_dt = variables.State();
}

void ChNodeFEAxyzPP::VariablesFbIncrementMq() {
    if (variables.IsDisabled())
        return;
    variables.AddMassTimesVector(variables.Force(), variables.State());
}

void ChNodeFEAxyzPP::VariablesQbIncrementPosition(double step) {
    if (variables.IsDisabled())
        return;

    ChVector3d pseudospeed(variables.State()(0), variables.State()(1), variables.State()(2));

    // ADVANCE FIELD: pos' = pos + dt * vel
    P = P + pseudospeed * step;
}

// -----------------------------------------------------------------------------

void ChNodeFEAxyzPP::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChNodeFEAxyzPP>();
    // serialize parent class
    ChNodeFEAbase::ArchiveOut(archive_out);
    // serialize all member data:
    archive_out << CHNVP(P);
    archive_out << CHNVP(P_dt);
    archive_out << CHNVP(F);
}

void ChNodeFEAxyzPP::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChNodeFEAxyzPP>();
    // deserialize parent class
    ChNodeFEAbase::ArchiveIn(archive_in);
    // stream in all member data:
    archive_in >> CHNVP(P);
    archive_in >> CHNVP(P_dt);
    archive_in >> CHNVP(F);
}

}  // end namespace fea
}  // end namespace chrono
