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

#include "chrono_fea/ChNodeFEAxyzP.h"

namespace chrono {
namespace fea {

ChNodeFEAxyzP::ChNodeFEAxyzP(ChVector<> initial_pos) : pos(initial_pos), P(0), P_dt(0), F(0) {
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

void ChNodeFEAxyzP::NodeIntToDescriptor(const unsigned int off_v, const ChStateDelta& v, const ChVectorDynamic<>& R) {
    variables.Get_qb().PasteClippedMatrix(v, off_v, 0, 1, 1, 0, 0);
    variables.Get_fb().PasteClippedMatrix(R, off_v, 0, 1, 1, 0, 0);
}

void ChNodeFEAxyzP::NodeIntFromDescriptor(const unsigned int off_v, ChStateDelta& v) {
    v.PasteMatrix(variables.Get_qb(), off_v, 0);
}

// -----------------------------------------------------------------------------

void ChNodeFEAxyzP::InjectVariables(ChSystemDescriptor& mdescriptor) {
    mdescriptor.InsertVariables(&variables);
}

void ChNodeFEAxyzP::VariablesFbReset() {
    variables.Get_fb().FillElem(0.0);
}

void ChNodeFEAxyzP::VariablesFbLoadForces(double factor) {
    if (variables.IsDisabled())
        return;
    variables.Get_fb().ElementN(0) += F * factor;
}

void ChNodeFEAxyzP::VariablesQbLoadSpeed() {
    if (variables.IsDisabled())
        return;
    // not really a 'speed', just the field derivative (may be used in incremental solver)
    variables.Get_qb().SetElement(0, 0, P_dt);
}

void ChNodeFEAxyzP::VariablesQbSetSpeed(double step) {
    if (variables.IsDisabled())
        return;
    // not really a 'speed', just the field derivative (may be used in incremental solver)
    P_dt = variables.Get_qb().GetElement(0, 0);
}

void ChNodeFEAxyzP::VariablesFbIncrementMq() {
    if (variables.IsDisabled())
        return;
    variables.Compute_inc_Mb_v(variables.Get_fb(), variables.Get_qb());
}

void ChNodeFEAxyzP::VariablesQbIncrementPosition(double step) {
    if (variables.IsDisabled())
        return;

    double pseudospeed = variables.Get_qb().GetElement(0, 0);

    // ADVANCE FIELD: pos' = pos + dt * vel
    P = P + pseudospeed * step;
}

// -----------------------------------------------------------------------------

void ChNodeFEAxyzP::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChNodeFEAxyzP>();
    // serialize parent class
    ChNodeFEAbase::ArchiveOUT(marchive);
    // serialize all member data:
    marchive << CHNVP(P);
    marchive << CHNVP(P_dt);
    marchive << CHNVP(F);
}

void ChNodeFEAxyzP::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChNodeFEAxyzP>();
    // deserialize parent class
    ChNodeFEAbase::ArchiveIN(marchive);
    // stream in all member data:
    marchive >> CHNVP(P);
    marchive >> CHNVP(P_dt);
    marchive >> CHNVP(F);
}

}  // end namespace fea
}  // end namespace chrono
