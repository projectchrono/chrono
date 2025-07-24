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

#ifndef CH_CONTACT_NSC_ROLLING_H
#define CH_CONTACT_NSC_ROLLING_H

#include "chrono/physics/ChContactNSC.h"
#include "chrono/solver/constraints_contact/ChConstraintRollingNormal.h"
#include "chrono/solver/constraints_contact/ChConstraintRollingTangential.h"

namespace chrono {

/// Class for non-smooth contact with rolling between two generic ChContactable objects.
/// It inherits ChContactNSC, which has three reaction forces (N,U,V) and adds three rolling reaction torques.
class ChContactNSCrolling : public ChContactNSC {
  protected:
    // The three scalar constraints, to be fed into the system solver
    ChConstraintRollingNormal Rx;      ///< normal rolling constraint
    ChConstraintRollingTangential Ru;  ///< first tangential rolling constraint
    ChConstraintRollingTangential Rv;  ///< second tangential rolling constraint

    ChVector3d react_torque;  ///< constraint reaction torque

    float complianceRoll;
    float complianceSpin;

  public:
    ChContactNSCrolling() {
        Rx.SetRollingConstraintU(&Ru);
        Rx.SetRollingConstraintV(&Rv);
        Rx.SetNormalConstraint(&Nx);
    }

    ChContactNSCrolling(ChContactContainer* contact_container,     ///< contact container
                        ChContactable* obj_A,                      ///< contactable object A
                        ChContactable* obj_B,                      ///< contactable object B
                        const ChCollisionInfo& cinfo,              ///< data for the collision pair
                        const ChContactMaterialCompositeNSC& mat,  ///< composite material
                        double min_speed                           ///< minimum speed for rebounce
                        )
        : ChContactNSC(contact_container, obj_A, obj_B, cinfo, mat, min_speed) {
        Rx.SetRollingConstraintU(&Ru);
        Rx.SetRollingConstraintV(&Rv);
        Rx.SetNormalConstraint(&Nx);

        Reset(obj_A, obj_B, cinfo, mat, min_speed);
    }

    virtual ~ChContactNSCrolling() {}

    /// Reinitialize this contact for reuse.
    virtual void Reset(ChContactable* obj_A,                      ///< contactable object A
                       ChContactable* obj_B,                      ///< contactable object B
                       const ChCollisionInfo& cinfo,              ///< data for the collision pair
                       const ChContactMaterialCompositeNSC& mat,  ///< composite material
                       double min_speed                           ///< minimum speed for rebounce
                       ) override {
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
        objA->ComputeJacobianForRollingContactPart(this->p1, this->contact_plane, Rx.Get_tuple_a(), Ru.Get_tuple_a(),
                                                   Rv.Get_tuple_a(), false);

        // delegate objB to compute its half of jacobian
        objB->ComputeJacobianForRollingContactPart(this->p2, this->contact_plane, Rx.Get_tuple_b(), Ru.Get_tuple_b(),
                                                   Rv.Get_tuple_b(), true);

        react_torque = VNULL;
    }

    /// Get the contact force, if computed, in contact coordinate system
    virtual ChVector3d GetContactTorque() { return react_torque; }

    /// Get the contact rolling friction coefficient
    virtual float GetRollingFriction() { return Rx.GetRollingFrictionCoefficient(); }
    /// Set the contact rolling friction coefficient
    virtual void SetRollingFriction(float mf) { Rx.SetRollingFrictionCoefficient(mf); }

    /// Get the contact spinning friction coefficient
    virtual float GetSpinningFriction() { return Rx.GetSpinningFrictionCoefficient(); }
    /// Set the contact spinning friction coefficient
    virtual void SetSpinningFriction(float mf) { Rx.SetSpinningFrictionCoefficient(mf); }

    /// Access the constraints
    ChConstraint* GetConstraintRu() { return &Ru; }
    ChConstraint* GetConstraintRv() { return &Rv; }
    ChConstraint* GetConstraintRx() { return &Rx; }

    // UPDATING FUNCTIONS

    virtual void ContIntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) override {
        // base behaviour too
        ChContactNSC::ContIntStateGatherReactions(off_L, L);

        L(off_L + 3) = react_torque.x();
        L(off_L + 4) = react_torque.y();
        L(off_L + 5) = react_torque.z();
    }

    virtual void ContIntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) override {
        // base behaviour too
        ChContactNSC::ContIntStateScatterReactions(off_L, L);

        react_torque.x() = L(off_L + 3);
        react_torque.y() = L(off_L + 4);
        react_torque.z() = L(off_L + 5);
    }

    virtual void ContIntLoadResidual_CqL(const unsigned int off_L,
                                         ChVectorDynamic<>& R,
                                         const ChVectorDynamic<>& L,
                                         const double c) override {
        // base behaviour too
        ChContactNSC::ContIntLoadResidual_CqL(off_L, R, L, c);

        Rx.AddJacobianTransposedTimesScalarInto(R, L(off_L + 3) * c);
        Ru.AddJacobianTransposedTimesScalarInto(R, L(off_L + 4) * c);
        Rv.AddJacobianTransposedTimesScalarInto(R, L(off_L + 5) * c);
    }

    virtual void ContIntLoadConstraint_C(const unsigned int off_L,
                                         ChVectorDynamic<>& Qc,
                                         const double c,
                                         bool do_clamp,
                                         double recovery_clamp) override {
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

    // virtual void ContIntLoadResidual_F(ChVectorDynamic<>& R, const double c)  {
    // no force to add - this is NSC, not SMC
    //};

    virtual void ContIntToDescriptor(const unsigned int off_L,
                                     const ChVectorDynamic<>& L,
                                     const ChVectorDynamic<>& Qc) override {
        // base behaviour too
        ChContactNSC::ContIntToDescriptor(off_L, L, Qc);

        Rx.SetLagrangeMultiplier(L(off_L + 3));
        Ru.SetLagrangeMultiplier(L(off_L + 4));
        Rv.SetLagrangeMultiplier(L(off_L + 5));

        Rx.SetRightHandSide(Qc(off_L + 3));
        Ru.SetRightHandSide(Qc(off_L + 4));
        Rv.SetRightHandSide(Qc(off_L + 5));
    }

    virtual void ContIntFromDescriptor(const unsigned int off_L, ChVectorDynamic<>& L) override {
        // base behaviour too
        ChContactNSC::ContIntFromDescriptor(off_L, L);

        L(off_L + 3) = Rx.GetLagrangeMultiplier();
        L(off_L + 4) = Ru.GetLagrangeMultiplier();
        L(off_L + 5) = Rv.GetLagrangeMultiplier();
    }

    virtual void InjectConstraints(ChSystemDescriptor& descriptor) override {
        // base behaviour too
        ChContactNSC::InjectConstraints(descriptor);

        descriptor.InsertConstraint(&Rx);
        descriptor.InsertConstraint(&Ru);
        descriptor.InsertConstraint(&Rv);
    }

    virtual void ConstraintsBiReset() override {
        // base behaviour too
        ChContactNSC::ConstraintsBiReset();

        Rx.SetRightHandSide(0.);
        Ru.SetRightHandSide(0.);
        Rv.SetRightHandSide(0.);
    }

    virtual void ConstraintsBiLoad_C(double factor = 1., double recovery_clamp = 0.1, bool do_clamp = false) override {
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

    virtual void ConstraintsFetch_react(double factor) override {
        // base behaviour too
        ChContactNSC::ConstraintsFetch_react(factor);

        // From constraints to react torque:
        react_torque.x() = Rx.GetLagrangeMultiplier() * factor;
        react_torque.y() = Ru.GetLagrangeMultiplier() * factor;
        react_torque.z() = Rv.GetLagrangeMultiplier() * factor;
    }
};

}  // end namespace chrono

#endif
