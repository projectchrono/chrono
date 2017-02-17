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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#include "chrono/physics/ChShaft.h"
#include "chrono/physics/ChShaftsClutch.h"
#include "chrono/physics/ChSystem.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChShaftsClutch)

ChShaftsClutch::ChShaftsClutch() : maxT(1), minT(-1), modulation(1), torque_react(0) {}

ChShaftsClutch::ChShaftsClutch(const ChShaftsClutch& other) : ChShaftsCouple(other) {
    maxT = other.maxT;
    minT = other.minT;
    modulation = other.modulation;
    torque_react = other.torque_react;
}

bool ChShaftsClutch::Initialize(std::shared_ptr<ChShaft> mshaft1, std::shared_ptr<ChShaft> mshaft2) {
    // parent class initialization
    if (!ChShaftsCouple::Initialize(mshaft1, mshaft2))
        return false;

    ChShaft* mm1 = mshaft1.get();
    ChShaft* mm2 = mshaft2.get();

    constraint.SetVariables(&mm1->Variables(), &mm2->Variables());

    SetSystem(shaft1->GetSystem());

    return true;
}

void ChShaftsClutch::Update(double mytime, bool update_assets) {
    // Inherit time changes of parent class
    ChShaftsCouple::Update(mytime, update_assets);

    // update class data
    // ...
}

void ChShaftsClutch::SetTorqueLimit(double ml, double mu) {
    minT = ml;
    maxT = mu;
}

//// STATE BOOKKEEPING FUNCTIONS

void ChShaftsClutch::IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) {
    L(off_L) = torque_react;
}

void ChShaftsClutch::IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) {
    torque_react = L(off_L);
}

void ChShaftsClutch::IntLoadResidual_CqL(const unsigned int off_L,    // offset in L multipliers
                                         ChVectorDynamic<>& R,        // result: the R residual, R += c*Cq'*L
                                         const ChVectorDynamic<>& L,  // the L vector
                                         const double c               // a scaling factor
                                         ) {
    constraint.MultiplyTandAdd(R, L(off_L) * c);
}

void ChShaftsClutch::IntLoadConstraint_C(const unsigned int off_L,  // offset in Qc residual
                                         ChVectorDynamic<>& Qc,     // result: the Qc residual, Qc += c*C
                                         const double c,            // a scaling factor
                                         bool do_clamp,             // apply clamping to c*C?
                                         double recovery_clamp      // value for min/max clamping of c*C
                                         ) {
    double res = 0;  // no residual anyway! allow drifting...

    double cnstr_violation = c * res;

    if (do_clamp) {
        cnstr_violation = ChMin(ChMax(cnstr_violation, -recovery_clamp), recovery_clamp);
    }

    Qc(off_L) += cnstr_violation;
}

void ChShaftsClutch::IntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) {
    // Might not be the best place to put this, but it works.
    // Update the limits on lagrangian multipliers:
    double dt = c;  // note: not always c=dt, this is true for euler implicit linearized and similar DVI timesteppers,
                    // might be not the case in future
    // double dt = system->GetStep(); // this could be another option.. but with variable-dt timesteppers it
    // should go deeper..
    constraint.SetBoxedMinMax(dt * minT * modulation, dt * maxT * modulation);
}

void ChShaftsClutch::IntToDescriptor(const unsigned int off_v,  // offset in v, R
                                     const ChStateDelta& v,
                                     const ChVectorDynamic<>& R,
                                     const unsigned int off_L,  // offset in L, Qc
                                     const ChVectorDynamic<>& L,
                                     const ChVectorDynamic<>& Qc) {
    constraint.Set_l_i(L(off_L));

    constraint.Set_b_i(Qc(off_L));
}

void ChShaftsClutch::IntFromDescriptor(const unsigned int off_v,  // offset in v
                                       ChStateDelta& v,
                                       const unsigned int off_L,  // offset in L
                                       ChVectorDynamic<>& L) {
    L(off_L) = constraint.Get_l_i();
}

// SOLVER INTERFACES

void ChShaftsClutch::InjectConstraints(ChSystemDescriptor& mdescriptor) {
    // if (!IsActive())
    //	return;

    mdescriptor.InsertConstraint(&constraint);
}

void ChShaftsClutch::ConstraintsBiReset() {
    constraint.Set_b_i(0.);
}

void ChShaftsClutch::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp) {
    // if (!IsActive())
    //	return;

    double res = 0;  // no residual

    constraint.Set_b_i(constraint.Get_b_i() + factor * res);
}

void ChShaftsClutch::ConstraintsBiLoad_Ct(double factor) {
    // if (!IsActive())
    //	return;

    // nothing
}

void ChShaftsClutch::ConstraintsFbLoadForces(double factor) {
    // no forces

    // compute jacobians
    double m_dt = factor;

    constraint.SetBoxedMinMax(m_dt * minT * modulation, m_dt * maxT * modulation);
}

void ChShaftsClutch::ConstraintsLoadJacobians() {
    constraint.Get_Cq_a()->SetElement(0, 0, 1.0);
    constraint.Get_Cq_b()->SetElement(0, 0, -1.0);
}

void ChShaftsClutch::ConstraintsFetch_react(double factor) {
    // From constraints to react vector:
    torque_react = constraint.Get_l_i() * factor;
}

//////// FILE I/O

void ChShaftsClutch::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChShaftsClutch>();

    // serialize parent class
    ChShaftsCouple::ArchiveOUT(marchive);

    // serialize all member data:
    marchive << CHNVP(maxT);
    marchive << CHNVP(minT);
    marchive << CHNVP(modulation);
}

/// Method to allow de serialization of transient data from archives.
void ChShaftsClutch::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChShaftsClutch>();

    // deserialize parent class:
    ChShaftsCouple::ArchiveIN(marchive);

    // deserialize all member data:
    marchive >> CHNVP(maxT);
    marchive >> CHNVP(minT);
    marchive >> CHNVP(modulation);
}

}  // end namespace chrono
