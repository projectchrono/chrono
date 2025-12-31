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

#ifndef CHSHAFTSCLUTCH_H
#define CHSHAFTSCLUTCH_H

#include "chrono/physics/ChShaftsCouple.h"
#include "chrono/solver/ChConstraintTwoGenericBoxed.h"

namespace chrono {

/// Class for defining a clutch or a brake (1D model) between two one-degree-of-freedom parts.
class ChApi ChShaftsClutch : public ChShaftsCouple {
  public:
    ChShaftsClutch();
    ChShaftsClutch(const ChShaftsClutch& other);
    ~ChShaftsClutch() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChShaftsClutch* Clone() const override { return new ChShaftsClutch(*this); }

    /// Number of scalar constraints, for statistical reasons
    virtual unsigned int GetNumConstraintsBilateral() override { return 1; }

    /// Disable this element (disable constraints).
    void SetDisabled(bool val) { active = !val; }

    /// Set the transmissible torque limit (the maximum torque that the clutch can transmit between the two shafts).
    /// You can specify two values for backward/forward directions: usually these are equal (ex. -100,100) in most
    /// commercial clutches, but if you define (0,100), for instance, you can create a so called freewheel or
    /// overrunning clutch that works only in one direction.
    void SetTorqueLimit(double ml, double mu);

    /// Set the transmissible torque limit (the maximum torque that
    /// the clutch can transmit between the two shafts), for both
    /// forward and backward direction.
    void SetTorqueLimit(double ml) { SetTorqueLimit(-fabs(ml), fabs(ml)); }

    /// Get the torque limit for forward rotation.
    double GetTorqueLimitF() const { return maxT; }

    /// Get the torque limit for backward rotation.
    double GetTorqueLimitB() const { return minT; }

    /// Get the torque limit (when this is a clutch with symmetric forw/backw limits).
    double GetTorqueLimit() const { return maxT; }

    /// Set the user modulation of the torque (or brake, if you use it between a fixed shaft and a free shaft).
    /// The modulation must range from 0 (switched off) to 1 (max torque). Default is 1, when clutch is created.
    /// You can update this during integration loop to simulate the pedal pushing by the driver.
    void SetModulation(double mm) { modulation = std::max(std::min(mm, 1.0), 0.0); }

    /// Get the the user modulation.
    double GetModulation() const { return modulation; }

    /// Get the actual angle slippage of the clutch, in terms of phase of shaft 1 with respect to 2.
    double GetSlippage() const { return GetRelativePos(); }

    /// Get the actual slippage speed of the clutch, in terms of speed of shaft 1 with respect to 2.
    double GetSlippageDt() const { return GetRelativePosDt(); }

    /// Get the actual slippage acceleration of the clutch, in terms of accel. of shaft 1 with respect to 2.
    double GetSlippageDt2() const { return GetRelativePosDt2(); }

    /// Get the reaction torque exchanged between the two shafts, considered as applied to the 1st axis.
    virtual double GetReaction1() const override { return torque_react; }

    /// Get the reaction torque exchanged between the two shafts, considered as applied to the 2nd axis.
    virtual double GetReaction2() const override { return -torque_react; }

    /// Initialize this shafts clutch, given two shafts to join.
    /// Both shafts must belong to the same ChSystem.
    bool Initialize(std::shared_ptr<ChShaft> shaft_1,  ///< first  shaft to join
                    std::shared_ptr<ChShaft> shaft_2   ///< second shaft to join
                    ) override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  private:
    bool active;

    double maxT;                             ///< clutch max transmissible torque (for forward direction
    double minT;                             ///< clutch min transmissible torque (for backward direction)
    double modulation;                       ///< 0...1  (default 1).
    double torque_react;                     ///< reaction torque
    ChConstraintTwoGenericBoxed constraint;  ///< used as an interface to the solver

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
    virtual void IntLoadConstraint_Ct(const unsigned int off, ChVectorDynamic<>& Qc, const double c) override {}
    virtual void IntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) override;
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
    virtual void ConstraintsBiLoad_Ct(double factor = 1) override;
    virtual void ConstraintsFbLoadForces(double factor = 1) override;
    virtual void ConstraintsFetch_react(double factor = 1) override;
};

CH_CLASS_VERSION(ChShaftsClutch, 0)

}  // end namespace chrono

#endif
