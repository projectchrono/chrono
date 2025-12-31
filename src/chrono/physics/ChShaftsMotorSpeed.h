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

#ifndef CH_SHAFTS_MOTOR_SPEED_H
#define CH_SHAFTS_MOTOR_SPEED_H

#include "chrono/physics/ChShaftsMotor.h"
#include "chrono/solver/ChConstraintTwoGeneric.h"
#include "chrono/solver/ChVariablesGeneric.h"
#include "chrono/functions/ChFunction.h"

namespace chrono {

/// Motor to enforce the relative speed w(t) between two shafts, using a rheonomic constraint.
/// The relative speed represents an anglular velocity for rotational motor and a linear speed for linear motors.
/// Note: no compliance is allowed, so if the actuator hits an undeformable  obstacle it hits a pathological situation
/// and the solver result can be unstable/unpredictable. Think at it as a servo drive with "infinitely stiff" control.
/// This type of motor is very easy to use, stable and efficient, and should be used if the 'infinitely stiff' control
/// assumption  is a good approximation of what you simulate (e.g., very good and reactive controllers). By default it
/// is initialized with constant angular speed: df/dt = 1. Use SetSpeedFunction() to change to other speed functions.
class ChApi ChShaftsMotorSpeed : public ChShaftsMotor {
  public:
    ChShaftsMotorSpeed();
    ChShaftsMotorSpeed(const ChShaftsMotorSpeed& other);
    ~ChShaftsMotorSpeed() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChShaftsMotorSpeed* Clone() const override { return new ChShaftsMotorSpeed(*this); }

    /// Sets the angular speed function s(t).
    /// Best if C0 continuous, otherwise it gives spikes in accelerations.
    void SetSpeedFunction(const std::shared_ptr<ChFunction> function) { motor_function = function; }

    /// Gets the speed function s(t).
    std::shared_ptr<ChFunction> GetSpeedFunction() const { return motor_function; }

    /// Set initial offset (default: 0).
    void SetOffset(double offset) { rot_offset = offset; }

    /// Get initial offset.
    double GetOffset() const { return rot_offset; }

    /// Enable relative position drift avoidance (default: true).
    /// If true, the constraint is satisfied also at the position level, by integrating the velocity in a separate
    /// auxiliary state.
    void AvoidDrift(bool avoid) { avoid_drift = avoid; }

    /// Initialize this motor, given two shafts to join.
    /// The first shaft is the 'output' shaft of the motor, the second is the 'truss', often fixed and not rotating.
    /// The torque is applied to the output shaft, while the truss shafts gets the same torque but with opposite sign.
    /// Both shafts must belong to the same ChSystem.
    bool Initialize(std::shared_ptr<ChShaft> shaft_1,  ///< first shaft to join (motor output shaft)
                    std::shared_ptr<ChShaft> shaft_2   ///< second shaft to join (motor truss)
                    ) override;

    /// Get the current motor torque between shaft2 and shaft1, expressed as applied to shaft1
    virtual double GetMotorLoad() const override { return motor_load; }

    ChVariablesGeneric& Variables() { return variable; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  private:
    std::shared_ptr<ChFunction> motor_function;
    double rot_offset;

    ChVariablesGeneric variable;

    double aux_dt;
    double aux_dtdt;

    bool avoid_drift;

    double motor_load;
    ChConstraintTwoGeneric constraint;  ///< used as an interface to the solver

    virtual unsigned int GetNumCoordsPosLevel() override { return 1; }
    virtual unsigned int GetNumConstraintsBilateral() override { return 1; }

    virtual void Update(double time, bool update_assets) override;

    virtual void IntStateGather(const unsigned int off_x,
                                ChState& x,
                                const unsigned int off_v,
                                ChStateDelta& v,
                                double& T) override;
    virtual void IntStateScatter(const unsigned int off_x,
                                 const ChState& x,
                                 const unsigned int off_v,
                                 const ChStateDelta& v,
                                 const double T,
                                 bool full_update) override;
    virtual void IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) override;
    virtual void IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) override;
    virtual void IntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) override;
    virtual void IntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) override;
    virtual void IntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) override;
    virtual void IntLoadResidual_Mv(const unsigned int off,
                                    ChVectorDynamic<>& R,
                                    const ChVectorDynamic<>& w,
                                    const double c) override;
    virtual void IntLoadLumpedMass_Md(const unsigned int off,
                                      ChVectorDynamic<>& Md,
                                      double& err,
                                      const double c) override;
    virtual void IntLoadResidual_CqL(const unsigned int off_L,
                                     ChVectorDynamic<>& R,
                                     const ChVectorDynamic<>& L,
                                     const double c) override;
    virtual void IntLoadConstraint_C(const unsigned int off,
                                     ChVectorDynamic<>& Qc,
                                     const double c,
                                     bool do_clamp,
                                     double recovery_clamp) override;
    virtual void IntLoadConstraint_Ct(const unsigned int off, ChVectorDynamic<>& Qc, const double c) override;
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

    virtual void InjectVariables(ChSystemDescriptor& descriptor) override;
    virtual void InjectConstraints(ChSystemDescriptor& descriptor) override;
    virtual void LoadConstraintJacobians() override;

    virtual void ConstraintsBiReset() override;
    virtual void ConstraintsBiLoad_C(double factor = 1, double recovery_clamp = 0.1, bool do_clamp = false) override;
    virtual void ConstraintsBiLoad_Ct(double factor = 1) override;
    virtual void ConstraintsFetch_react(double factor = 1) override;
    virtual void VariablesFbReset() override;
    virtual void VariablesFbLoadForces(double factor = 1) override;
    virtual void VariablesQbLoadSpeed() override;
    virtual void VariablesFbIncrementMq() override;
    virtual void VariablesQbSetSpeed(double step = 0) override;
};

CH_CLASS_VERSION(ChShaftsMotorSpeed, 0)

}  // end namespace chrono

#endif
