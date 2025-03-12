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
// Authors: Alessandro Tasora
// =============================================================================

#ifndef CHSHAFTSFREEWHEEL_H
#define CHSHAFTSFREEWHEEL_H

#include "chrono/physics/ChShaftsCouple.h"
#include "chrono/solver/ChConstraintTwoGenericBoxed.h"

namespace chrono {

/// Class for defining a 'freewheel' (a 1D model of a ratchet wheel) between two one-degree-of-freedom parts.
class ChApi ChShaftsFreewheel : public ChShaftsCouple {
  public:
    // By default this is a ratcheting clutch with 24 teeth: use SetRatchetingModeTeeth()
    // or SetRatchetingModeStep() or SetJammingMode() to change this.

    ChShaftsFreewheel();
    ChShaftsFreewheel(const ChShaftsFreewheel& other);
    ~ChShaftsFreewheel() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChShaftsFreewheel* Clone() const override { return new ChShaftsFreewheel(*this); }

    /// Number of scalar constraints
    virtual unsigned int GetNumConstraintsBilateral() override { return 1; }

    /// Set if you want the clutch to work in "ratcheting mode", here defined via an
    /// angular step in [rad].
    /// When in ratcheting mode, there is a periodic "backward backlash", like in bicycle
    /// freewheels.
    void SetRatchetingModeStep(double mt);

    /// Set if you want the clutch to work in "ratcheting mode", here defined via a
    /// number of teeth.
    /// When in ratcheting mode, there is a periodic "backward backlash", like in bicycle
    /// freewheels.
    void SetRatchetingModeTeeth(int n_teeth);

    /// Set if you want the clutch to work in "jamming mode" like a sprag clutch,
    /// based on friction with pawls and jamming. This is like a ratcheting clutch with infinite teeth,
    /// hence a perfect unidirectional clutch without backward backlash, but
    /// instead of using SetRatchetingModeStep(0), that can't work well because of bad
    /// conditioning, just use this.
    void SetJammingMode();

    /// Get the angular step [rad], if a "ratcheting mode" freewheel (it is zero if in "jamming mode").
    double GetRatchetingStep() const { return step; }

    /// True if the clutch is working in "jamming mode" like a sprag clutch.
    /// This represents a perfect unidirectional clutch without backward backlash.
    bool IsModeJamming() const { return jamming_mode; }

    /// True if the clutch is working in "ratcheting mode".
    /// This implies some periodic backward backlash.
    bool IsModeRatcheting() { return !jamming_mode; }

    /// Shaft 1 free to rotate forward respect to shaft 2.
    /// Viceversa, backward rotation of 1 respect to 2 is prevented and will generate a torque.
    void SetFreeForward();

    /// Shaft 1 free to rotate backward respect to shaft 2.
    /// Viceversa, forward rotation of 1 respect to 2 is prevented and will generate a torque.
    void SetFreeBackward();

    /// If true, shaft 1 is free to rotate forward respect to shaft 2 (default). Viceversa, if false.
    /// Use SetFreeForward() or SetFreeBackward() to change this mode.
    bool IsFreeForward() const { return free_forward; }

    /// If in "ratcheting mode", this is the phase of the first tooth respect to zero rotation of shaft 1. Default 0.
    void SetPhase(double p) { phase = p; }

    /// If in "ratcheting mode", this is the phase of the first tooth respect to zero rotation of shaft 1.
    double GetPhase() const { return this->phase; }

    /// Get the reaction torque exchanged between the two shafts,
    /// considered as applied to the 1st axis.
    double GetReaction1() const override { return torque_react; }

    /// Get the reaction torque exchanged between the two shafts,
    /// considered as applied to the 2nd axis.
    double GetReaction2() const override { return -torque_react; }

    /// Return current constraint violation
    double GetConstraintViolation() const { return violation; }

    /// Return current max relative rotation
    double GetMaxReachedRelativeRotation() const { return alpha_max; }

    /// Return the current teeth vane in ratcheting freewheel (returns zero if in jamming mode).
    double GetCurrentTeethVane() const;

    /// Use this function after gear creation, to initialize it, given
    /// two shafts to join.
    /// Each shaft must belong to the same ChSystem.
    /// By default this is a ratcheting clutch with 24 teeth: use SetRatchetingModeTeeth()
    /// or SetRatchetingModeStep() or SetJammingMode() to change this.
    bool Initialize(std::shared_ptr<ChShaft> shaft_1,  ///< first  shaft to join
                    std::shared_ptr<ChShaft> shaft_2   ///< second shaft to join
                    ) override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  private:
    double step;                             ///< angular step, for ratcheting
    double violation;                        ///< constraint violation
    double alpha_max;                        ///< record max rotation, must be monotone
    double torque_react;                     ///< reaction torque
    ChConstraintTwoGenericBoxed constraint;  ///< used as an interface to the solver
    bool jamming_mode;
    double phase;

    /// default true: shaft 1 free to rotate forward respect to shaft 2. False: 1 free backward respect to 2
    bool free_forward;

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

CH_CLASS_VERSION(ChShaftsFreewheel, 0)

}  // end namespace chrono

#endif
