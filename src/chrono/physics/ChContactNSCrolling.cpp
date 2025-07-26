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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#include "chrono/physics/ChContactNSCrolling.h"

namespace chrono {

ChContactNSCrolling::ChContactNSCrolling() {
    Rx.SetRollingConstraintU(&Ru);
    Rx.SetRollingConstraintV(&Rv);
    Rx.SetNormalConstraint(&Nx);
}

ChContactNSCrolling::ChContactNSCrolling(ChContactContainer* contact_container,     // contact container
                                         ChContactable* obj_A,                      // contactable object A
                                         ChContactable* obj_B,                      // contactable object B
                                         const ChCollisionInfo& cinfo,              // data for the collision pair
                                         const ChContactMaterialCompositeNSC& mat,  // composite material
                                         double min_speed                           // minimum speed for rebounce
                                         )
    : ChContactNSC(contact_container, obj_A, obj_B, cinfo, mat, min_speed) {
    Rx.SetRollingConstraintU(&Ru);
    Rx.SetRollingConstraintV(&Rv);
    Rx.SetNormalConstraint(&Nx);

    Reset(obj_A, obj_B, cinfo, mat, min_speed);
}

void ChContactNSCrolling::Reset(ChContactable* obj_A,                      // contactable object A
                                ChContactable* obj_B,                      // contactable object B
                                const ChCollisionInfo& cinfo,              // data for the collision pair
                                const ChContactMaterialCompositeNSC& mat,  // composite material
                                double min_speed                           // minimum speed for rebounce
) {
    // Invoke base class method to reset normal and sliding constraints
    ChContactNSC::Reset(obj_A, obj_B, cinfo, mat, min_speed);

    // Create constraint tuples and set variables
    Rx.SetTuplesFromContactables(objA, objB);
    Ru.SetTuplesFromContactables(objA, objB);
    Rv.SetTuplesFromContactables(objA, objB);

    // Cache composite material properties
    Rx.SetRollingFrictionCoefficient(mat.rolling_friction);
    Rx.SetSpinningFrictionCoefficient(mat.spinning_friction);

    complianceRoll = mat.complianceRoll;
    complianceSpin = mat.complianceSpin;

    // COMPUTE JACOBIANS

    // delegate objA to compute its half of jacobian
    objA->ComputeJacobianForRollingContactPart(p1, contact_plane, Rx.TupleA(), Ru.TupleA(), Rv.TupleA(), false);

    // delegate objB to compute its half of jacobian
    objB->ComputeJacobianForRollingContactPart(p2, contact_plane, Rx.TupleB(), Ru.TupleB(), Rv.TupleB(), true);

    react_torque = VNULL;
}

void ChContactNSCrolling::ContIntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) {
    // base behaviour too
    ChContactNSC::ContIntStateGatherReactions(off_L, L);

    L(off_L + 3) = react_torque.x();
    L(off_L + 4) = react_torque.y();
    L(off_L + 5) = react_torque.z();
}

void ChContactNSCrolling::ContIntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) {
    // base behaviour too
    ChContactNSC::ContIntStateScatterReactions(off_L, L);

    react_torque.x() = L(off_L + 3);
    react_torque.y() = L(off_L + 4);
    react_torque.z() = L(off_L + 5);
}

void ChContactNSCrolling::ContIntLoadResidual_CqL(const unsigned int off_L,
                                                  ChVectorDynamic<>& R,
                                                  const ChVectorDynamic<>& L,
                                                  const double c) {
    // base behaviour too
    ChContactNSC::ContIntLoadResidual_CqL(off_L, R, L, c);

    Rx.AddJacobianTransposedTimesScalarInto(R, L(off_L + 3) * c);
    Ru.AddJacobianTransposedTimesScalarInto(R, L(off_L + 4) * c);
    Rv.AddJacobianTransposedTimesScalarInto(R, L(off_L + 5) * c);
}

void ChContactNSCrolling::ContIntLoadConstraint_C(const unsigned int off_L,
                                                  ChVectorDynamic<>& Qc,
                                                  const double c,
                                                  bool do_clamp,
                                                  double recovery_clamp) {
    // base behaviour too
    ChContactNSC::ContIntLoadConstraint_C(off_L, Qc, c, do_clamp, recovery_clamp);

    // If rolling and spinning compliance, set the cfm terms
    double h = container->GetSystem()->GetStep();

    //// TODO  move to LoadKRMMatrices() the following, and only for !bounced case
    double alpha = dampingf;                    // [R]=alpha*[K]
    double inv_hhpa = 1.0 / (h * (h + alpha));  // 1/(h*(h+a))

    Ru.SetComplianceTerm((inv_hhpa)*complianceRoll);
    Rv.SetComplianceTerm((inv_hhpa)*complianceRoll);
    Rx.SetComplianceTerm((inv_hhpa)*complianceSpin);
}

void ChContactNSCrolling::ContIntToDescriptor(const unsigned int off_L,
                                              const ChVectorDynamic<>& L,
                                              const ChVectorDynamic<>& Qc) {
    // base behaviour too
    ChContactNSC::ContIntToDescriptor(off_L, L, Qc);

    Rx.SetLagrangeMultiplier(L(off_L + 3));
    Ru.SetLagrangeMultiplier(L(off_L + 4));
    Rv.SetLagrangeMultiplier(L(off_L + 5));

    Rx.SetRightHandSide(Qc(off_L + 3));
    Ru.SetRightHandSide(Qc(off_L + 4));
    Rv.SetRightHandSide(Qc(off_L + 5));
}

void ChContactNSCrolling::ContIntFromDescriptor(const unsigned int off_L, ChVectorDynamic<>& L) {
    // base behaviour too
    ChContactNSC::ContIntFromDescriptor(off_L, L);

    L(off_L + 3) = Rx.GetLagrangeMultiplier();
    L(off_L + 4) = Ru.GetLagrangeMultiplier();
    L(off_L + 5) = Rv.GetLagrangeMultiplier();
}

void ChContactNSCrolling::InjectConstraints(ChSystemDescriptor& descriptor) {
    // base behaviour too
    ChContactNSC::InjectConstraints(descriptor);

    descriptor.InsertConstraint(&Rx);
    descriptor.InsertConstraint(&Ru);
    descriptor.InsertConstraint(&Rv);
}

void ChContactNSCrolling::ConstraintsBiReset() {
    // base behaviour too
    ChContactNSC::ConstraintsBiReset();

    Rx.SetRightHandSide(0.);
    Ru.SetRightHandSide(0.);
    Rv.SetRightHandSide(0.);
}

void ChContactNSCrolling::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp) {
    // base behaviour too
    ChContactNSC::ConstraintsBiLoad_C(factor, recovery_clamp, do_clamp);

    // If rolling and spinning compliance, set the cfm terms
    double h = container->GetSystem()->GetStep();

    //// TODO  move to LoadKRMMatrices() the following, and only for !bounced case
    double alpha = dampingf;                    // [R]=alpha*[K]
    double inv_hhpa = 1.0 / (h * (h + alpha));  // 1/(h*(h+a))

    Ru.SetComplianceTerm((inv_hhpa)*complianceRoll);
    Rv.SetComplianceTerm((inv_hhpa)*complianceRoll);
    Rx.SetComplianceTerm((inv_hhpa)*complianceSpin);

    // Assume no residual ever, do not load in C
}

void ChContactNSCrolling::ConstraintsFetch_react(double factor) {
    // base behaviour too
    ChContactNSC::ConstraintsFetch_react(factor);

    // From constraints to react torque:
    react_torque.x() = Rx.GetLagrangeMultiplier() * factor;
    react_torque.y() = Ru.GetLagrangeMultiplier() * factor;
    react_torque.z() = Rv.GetLagrangeMultiplier() * factor;
}

}  // end namespace chrono
