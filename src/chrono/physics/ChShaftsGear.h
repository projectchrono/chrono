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

#ifndef CHSHAFTSGEAR_H
#define CHSHAFTSGEAR_H

#include "chrono/physics/ChShaftsCouple.h"
#include "chrono/solver/ChConstraintTwoGeneric.h"

namespace chrono {

/// Class for defining a 'transmission ratio' (a 1D gear) between two one-degree-of-freedom parts.
/// Note that this really simple constraint does not provide a way to transmit a reaction force to the truss, if this is
/// needed, just use the ChShaftsPlanetary with a fixed carrier shaft, or the ChShaftGearbox.
class ChApi ChShaftsGear : public ChShaftsCouple {
  public:
    ChShaftsGear();
    ChShaftsGear(const ChShaftsGear& other);
    ~ChShaftsGear() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChShaftsGear* Clone() const override { return new ChShaftsGear(*this); }

    /// Number of scalar constraints
    virtual unsigned int GetNumConstraintsBilateral() override { return 1; }

    /// Set the transmission ratio t, as in w2=t*w1, or t=w2/w1 , or  t*w1 - w2 = 0.
    /// For example, t=1 for a rigid joint; t=-0.5 for representing
    /// a couple of spur gears with teeth z1=20 & z2=40; t=0.1 for
    /// a gear with inner teeth (or epicycloidal reducer), etc.
    void SetTransmissionRatio(double t) { ratio = t; }

    /// Get the transmission ratio t, as in w2=t*w1, or t=w2/w1.
    double GetTransmissionRatio() const { return ratio; }

    /// Enable phase drift avoidance (default: true).
    /// If true, phasing is always tracked and the constraint is satisfied also at the position level.
    /// If false, microslipping can accumulate (as in friction wheels).
    void AvoidPhaseDrift(bool avoid) { avoid_phase_drift = avoid; }

    /// Get the reaction torque exchanged between the two shafts, considered as applied to the 1st axis.
    double GetReaction1() const override { return ratio * torque_react; }

    /// Get the reaction torque exchanged between the two shafts, considered as applied to the 2nd axis.
    double GetReaction2() const override { return -torque_react; }

    /// Return current constraint violation.
    double GetConstraintViolation() const { return violation; }

    /// Initialize this shafts gear, given two shafts to join.
    /// Both shafts must belong to the same ChSystem.
    bool Initialize(std::shared_ptr<ChShaft> shaft_1,  ///< first  shaft to join
                    std::shared_ptr<ChShaft> shaft_2   ///< second shaft to join
                    ) override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  private:
    double ratio;                       ///< transmission ratio t, as in w2=t*w1, or t=w2/w1
    double violation;                   ///< constraint violation
    double torque_react;                ///< reaction torque
    ChConstraintTwoGeneric constraint;  ///< used as an interface to the solver
    bool avoid_phase_drift;
    double phase1;
    double phase2;

    virtual void Update(double time, bool update_assets) override;

    virtual void IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) override;
    virtual void IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) override;
    virtual void IntLoadResidual_CqL(const unsigned int off_L,
                                     ChVectorDynamic<>& R,
                                     const ChVectorDynamic<>& L,
                                     const double c) override;
    virtual void IntLoadConstraint_C(const unsigned int off,
                                     ChVectorDynamic<>& Qc,
                                     const double c,
                                     bool do_clamp,
                                     double recovery_clamp) override;
    virtual void IntToDescriptor(const unsigned int off_v,
                                 const ChStateDelta& v,
                                 const ChVectorDynamic<>& R,
                                 const unsigned int off_L,
                                 const ChVectorDynamic<>& L,
                                 const ChVectorDynamic<>& Qc) override;
    virtual void IntFromDescriptor(const unsigned int off_v,
                                   ChStateDelta& v,
                                   const unsigned int off_L,
                                   ChVectorDynamic<>& L) override;

    virtual void InjectConstraints(ChSystemDescriptor& descriptor) override;
    virtual void LoadConstraintJacobians() override;

    virtual void ConstraintsBiReset() override;
    virtual void ConstraintsBiLoad_C(double factor = 1, double recovery_clamp = 0.1, bool do_clamp = false) override;
    virtual void ConstraintsFetch_react(double factor = 1) override;
};

CH_CLASS_VERSION(ChShaftsGear, 0)

}  // end namespace chrono

#endif
