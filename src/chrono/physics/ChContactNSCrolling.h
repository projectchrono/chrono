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
#include "chrono/solver/ChConstraintTwoTuplesRollingN.h"
#include "chrono/solver/ChConstraintTwoTuplesRollingT.h"

namespace chrono {

/// Class for non-smooth contact between two generic ChContactable objects.
/// It inherits ChContactNSC, that has three reaction forces (N,U,V), but also adds
/// three rolling reaction torques.
/// This means that it requires about twice the memory required by the ChContactNSC.
/// Ta and Tb are of ChContactable sub classes.

template <class Ta, class Tb>
class ChContactNSCrolling : public ChContactNSC<Ta, Tb> {
  public:
    typedef typename ChContactTuple<Ta, Tb>::typecarr_a typecarr_a;
    typedef typename ChContactTuple<Ta, Tb>::typecarr_b typecarr_b;

  protected:
    // The three scalar constraints, to be feed into the
    // system solver. They contain jacobians data and special functions.
    ChConstraintTwoTuplesRollingN<typecarr_a, typecarr_b> Rx;
    ChConstraintTwoTuplesRollingT<typecarr_a, typecarr_b> Ru;
    ChConstraintTwoTuplesRollingT<typecarr_a, typecarr_b> Rv;

    ChVector3d react_torque;

    float complianceRoll;
    float complianceSpin;

  public:
    ChContactNSCrolling() {
        Rx.SetRollingConstraintU(&this->Ru);
        Rx.SetRollingConstraintV(&this->Rv);
        Rx.SetNormalConstraint(&this->Nx);
    }

    ChContactNSCrolling(ChContactContainer* contact_container,     ///< contact container
                        Ta* obj_A,                                 ///< contactable object A
                        Tb* obj_B,                                 ///< contactable object B
                        const ChCollisionInfo& cinfo,              ///< data for the collision pair
                        const ChContactMaterialCompositeNSC& mat,  ///< composite material
                        double min_speed                           ///< minimum speed for rebounce
                        )
        : ChContactNSC<Ta, Tb>(contact_container, obj_A, obj_B, cinfo, mat, min_speed) {
        Rx.SetRollingConstraintU(&this->Ru);
        Rx.SetRollingConstraintV(&this->Rv);
        Rx.SetNormalConstraint(&this->Nx);

        Reset(obj_A, obj_B, cinfo, mat, min_speed);
    }

    virtual ~ChContactNSCrolling() {}

    /// Reinitialize this contact for reuse.
    virtual void Reset(Ta* obj_A,                                 ///< contactable object A
                       Tb* obj_B,                                 ///< contactable object B
                       const ChCollisionInfo& cinfo,              ///< data for the collision pair
                       const ChContactMaterialCompositeNSC& mat,  ///< composite material
                       double min_speed                           ///< minimum speed for rebounce
                       ) override {
        // Invoke base class method to reset normal and sliding constraints
        ChContactNSC<Ta, Tb>::Reset(obj_A, obj_B, cinfo, mat, min_speed);

        Rx.Get_tuple_a().SetVariables(*this->objA);
        Rx.Get_tuple_b().SetVariables(*this->objB);
        Ru.Get_tuple_a().SetVariables(*this->objA);
        Ru.Get_tuple_b().SetVariables(*this->objB);
        Rv.Get_tuple_a().SetVariables(*this->objA);
        Rv.Get_tuple_b().SetVariables(*this->objB);

        // Cache composite material properties
        Rx.SetRollingFrictionCoefficient(mat.rolling_friction);
        Rx.SetSpinningFrictionCoefficient(mat.spinning_friction);

        this->complianceRoll = mat.complianceRoll;
        this->complianceSpin = mat.complianceSpin;

        // COMPUTE JACOBIANS

        // delegate objA to compute its half of jacobian
        this->objA->ComputeJacobianForRollingContactPart(this->p1, this->contact_plane, Rx.Get_tuple_a(),
                                                         Ru.Get_tuple_a(), Rv.Get_tuple_a(), false);

        // delegate objB to compute its half of jacobian
        this->objB->ComputeJacobianForRollingContactPart(this->p2, this->contact_plane, Rx.Get_tuple_b(),
                                                         Ru.Get_tuple_b(), Rv.Get_tuple_b(), true);

        this->react_torque = VNULL;
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
        ChContactNSC<Ta, Tb>::ContIntStateGatherReactions(off_L, L);

        L(off_L + 3) = react_torque.x();
        L(off_L + 4) = react_torque.y();
        L(off_L + 5) = react_torque.z();
    }

    virtual void ContIntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) override {
        // base behaviour too
        ChContactNSC<Ta, Tb>::ContIntStateScatterReactions(off_L, L);

        react_torque.x() = L(off_L + 3);
        react_torque.y() = L(off_L + 4);
        react_torque.z() = L(off_L + 5);
    }

    virtual void ContIntLoadResidual_CqL(const unsigned int off_L,
                                         ChVectorDynamic<>& R,
                                         const ChVectorDynamic<>& L,
                                         const double c) override {
        // base behaviour too
        ChContactNSC<Ta, Tb>::ContIntLoadResidual_CqL(off_L, R, L, c);

        this->Rx.AddJacobianTransposedTimesScalarInto(R, L(off_L + 3) * c);
        this->Ru.AddJacobianTransposedTimesScalarInto(R, L(off_L + 4) * c);
        this->Rv.AddJacobianTransposedTimesScalarInto(R, L(off_L + 5) * c);
    }

    virtual void ContIntLoadConstraint_C(const unsigned int off_L,
                                         ChVectorDynamic<>& Qc,
                                         const double c,
                                         bool do_clamp,
                                         double recovery_clamp) override {
        // base behaviour too
        ChContactNSC<Ta, Tb>::ContIntLoadConstraint_C(off_L, Qc, c, do_clamp, recovery_clamp);

        // If rolling and spinning compliance, set the cfm terms
        double h = this->container->GetSystem()->GetStep();

        //// TODO  move to LoadKRMMatrices() the following, and only for !bounced case
        double alpha = this->dampingf;              // [R]=alpha*[K]
        double inv_hhpa = 1.0 / (h * (h + alpha));  // 1/(h*(h+a))

        this->Ru.SetComplianceTerm((inv_hhpa) * this->complianceRoll);
        this->Rv.SetComplianceTerm((inv_hhpa) * this->complianceRoll);
        this->Rx.SetComplianceTerm((inv_hhpa) * this->complianceSpin);
    }

    // virtual void ContIntLoadResidual_F(ChVectorDynamic<>& R, const double c)  {
    // no force to add - this is NSC, not SMC
    //};

    virtual void ContIntToDescriptor(const unsigned int off_L,
                                     const ChVectorDynamic<>& L,
                                     const ChVectorDynamic<>& Qc) override {
        // base behaviour too
        ChContactNSC<Ta, Tb>::ContIntToDescriptor(off_L, L, Qc);

        Rx.SetLagrangeMultiplier(L(off_L + 3));
        Ru.SetLagrangeMultiplier(L(off_L + 4));
        Rv.SetLagrangeMultiplier(L(off_L + 5));

        Rx.SetRightHandSide(Qc(off_L + 3));
        Ru.SetRightHandSide(Qc(off_L + 4));
        Rv.SetRightHandSide(Qc(off_L + 5));
    }

    virtual void ContIntFromDescriptor(const unsigned int off_L, ChVectorDynamic<>& L) override {
        // base behaviour too
        ChContactNSC<Ta, Tb>::ContIntFromDescriptor(off_L, L);

        L(off_L + 3) = Rx.GetLagrangeMultiplier();
        L(off_L + 4) = Ru.GetLagrangeMultiplier();
        L(off_L + 5) = Rv.GetLagrangeMultiplier();
    }

    virtual void InjectConstraints(ChSystemDescriptor& descriptor) override {
        // base behaviour too
        ChContactNSC<Ta, Tb>::InjectConstraints(descriptor);

        descriptor.InsertConstraint(&Rx);
        descriptor.InsertConstraint(&Ru);
        descriptor.InsertConstraint(&Rv);
    }

    virtual void ConstraintsBiReset() override {
        // base behaviour too
        ChContactNSC<Ta, Tb>::ConstraintsBiReset();

        Rx.SetRightHandSide(0.);
        Ru.SetRightHandSide(0.);
        Rv.SetRightHandSide(0.);
    }

    virtual void ConstraintsBiLoad_C(double factor = 1., double recovery_clamp = 0.1, bool do_clamp = false) override {
        // base behaviour too
        ChContactNSC<Ta, Tb>::ConstraintsBiLoad_C(factor, recovery_clamp, do_clamp);

        // If rolling and spinning compliance, set the cfm terms
        double h = this->container->GetSystem()->GetStep();

        //// TODO  move to LoadKRMMatrices() the following, and only for !bounced case
        double alpha = this->dampingf;              // [R]=alpha*[K]
        double inv_hhpa = 1.0 / (h * (h + alpha));  // 1/(h*(h+a))

        this->Ru.SetComplianceTerm((inv_hhpa) * this->complianceRoll);
        this->Rv.SetComplianceTerm((inv_hhpa) * this->complianceRoll);
        this->Rx.SetComplianceTerm((inv_hhpa) * this->complianceSpin);

        // Assume no residual ever, do not load in C
    }

    virtual void ConstraintsFetch_react(double factor) override {
        // base behaviour too
        ChContactNSC<Ta, Tb>::ConstraintsFetch_react(factor);

        // From constraints to react torque:
        react_torque.x() = Rx.GetLagrangeMultiplier() * factor;
        react_torque.y() = Ru.GetLagrangeMultiplier() * factor;
        react_torque.z() = Rv.GetLagrangeMultiplier() * factor;
    }
};

}  // end namespace chrono

#endif
