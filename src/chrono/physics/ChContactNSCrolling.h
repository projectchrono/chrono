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
#include "chrono/solver/ChConstraintRollingNormal.h"
#include "chrono/solver/ChConstraintRollingTangential.h"

namespace chrono {

/// Class for non-smooth contact with rolling between two generic ChContactable objects.
/// It inherits ChContactNSC, which has three reaction forces (N,U,V) and adds three rolling reaction torques.
class ChApi ChContactNSCrolling : public ChContactNSC {
  public:
    ChContactNSCrolling();

    ChContactNSCrolling(ChContactContainer* contact_container,     ///< contact container
                        ChContactable* obj_A,                      ///< contactable object A
                        ChContactable* obj_B,                      ///< contactable object B
                        const ChCollisionInfo& cinfo,              ///< data for the collision pair
                        const ChContactMaterialCompositeNSC& mat,  ///< composite material
                        double min_speed                           ///< minimum speed for rebounce
    );

    virtual ~ChContactNSCrolling() {}

    /// Reinitialize this contact for reuse.
    virtual void Reset(ChContactable* obj_A,                      ///< contactable object A
                       ChContactable* obj_B,                      ///< contactable object B
                       const ChCollisionInfo& cinfo,              ///< data for the collision pair
                       const ChContactMaterialCompositeNSC& mat,  ///< composite material
                       double min_speed                           ///< minimum speed for rebounce
                       ) override;

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

    virtual void ContIntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) override;
    virtual void ContIntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) override;
    virtual void ContIntLoadResidual_CqL(const unsigned int off_L,
                                         ChVectorDynamic<>& R,
                                         const ChVectorDynamic<>& L,
                                         const double c) override;
    virtual void ContIntLoadConstraint_C(const unsigned int off_L,
                                         ChVectorDynamic<>& Qc,
                                         const double c,
                                         bool do_clamp,
                                         double recovery_clamp) override;
    // virtual void ContIntLoadResidual_F(ChVectorDynamic<>& R, const double c)  {} // no force to add
    virtual void ContIntToDescriptor(const unsigned int off_L,
                                     const ChVectorDynamic<>& L,
                                     const ChVectorDynamic<>& Qc) override;
    virtual void ContIntFromDescriptor(const unsigned int off_L, ChVectorDynamic<>& L) override;
    virtual void InjectConstraints(ChSystemDescriptor& descriptor) override;
    virtual void ConstraintsBiReset() override;
    virtual void ConstraintsBiLoad_C(double factor = 1., double recovery_clamp = 0.1, bool do_clamp = false) override;
    virtual void ConstraintsFetch_react(double factor) override;

  protected:
    // The three scalar constraints, to be fed into the system solver
    ChConstraintRollingNormal Rx;      ///< normal rolling constraint
    ChConstraintRollingTangential Ru;  ///< first tangential rolling constraint
    ChConstraintRollingTangential Rv;  ///< second tangential rolling constraint

    ChVector3d react_torque;  ///< constraint reaction torque

    float complianceRoll;
    float complianceSpin;
};

}  // end namespace chrono

#endif
