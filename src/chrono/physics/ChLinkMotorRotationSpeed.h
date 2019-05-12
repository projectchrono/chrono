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

#ifndef CHLINKMOTORROTATIONSPEED_H
#define CHLINKMOTORROTATIONSPEED_H

#include "chrono/physics/ChLinkMotorRotation.h"
#include "chrono/solver/ChVariablesGeneric.h"

namespace chrono {

/// A motor that enforces the angular speed w(t) between two frames on two bodies, using a rheonomic constraint.
/// Note: no compliance is allowed, so if the actuator hits an undeformable obstacle it hits a pathological
/// situation and the solver result can be unstable/unpredictable.
/// Think at it as a servo drive with "infinitely stiff" control.
/// This type of motor is very easy to use, stable and efficient, and should be used if the 'infinitely stiff'
/// control assumption is a good approximation of what you simulate (e.g., very good and reactive controllers).
/// By default it is initialized with constant angular speed: df/dt = 1.
/// Use SetSpeedFunction() to change to other speed functions.

class ChApi ChLinkMotorRotationSpeed : public ChLinkMotorRotation {
  public:
    ChLinkMotorRotationSpeed();
    ChLinkMotorRotationSpeed(const ChLinkMotorRotationSpeed& other);
    virtual ~ChLinkMotorRotationSpeed();

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkMotorRotationSpeed* Clone() const override { return new ChLinkMotorRotationSpeed(*this); }

    /// Set the angular speed function of time w(t).
    /// To prevent acceleration pikes, this function should be C0 continuous.
    void SetSpeedFunction(const std::shared_ptr<ChFunction> function) { SetMotorFunction(function); }

    /// Get the angular speed function w(t).
    std::shared_ptr<ChFunction> GetSpeedFunction() const { return GetMotorFunction(); }

    /// Get initial offset, in [rad]. By default = 0.
    void SetAngleOffset(double mo) { rot_offset = mo; }

    /// Get initial offset, in [rad].
    double GetAngleOffset() { return rot_offset; }

    /// Set if the constraint must avoid angular drift. If true, it
    /// means that the constraint is satisfied also at the rotation level,
    /// by integrating the velocity in a separate auxiliary state. Default, true.
    void SetAvoidAngleDrift(bool mb) { this->avoid_angle_drift = mb; }

    /// Set if the constraint is in "avoid angle drift" mode.
    bool GetAvoidAngleDrift() { return this->avoid_angle_drift; }

    /// Get the current actuator reaction torque [Nm]
    virtual double GetMotorTorque() const override { return -this->react_torque.z(); }

    void Update(double mytime, bool update_assets) override;

    //
    // STATE FUNCTIONS
    //

    virtual int GetDOF() override { return 1; }

    ChVariablesGeneric& Variables() { return variable; }

    virtual void IntStateGather(const unsigned int off_x,
                                ChState& x,
                                const unsigned int off_v,
                                ChStateDelta& v,
                                double& T) override;
    virtual void IntStateScatter(const unsigned int off_x,
                                 const ChState& x,
                                 const unsigned int off_v,
                                 const ChStateDelta& v,
                                 const double T) override;
    virtual void IntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) override;
    virtual void IntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) override;
    virtual void IntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) override;
    virtual void IntLoadResidual_Mv(const unsigned int off,
                                    ChVectorDynamic<>& R,
                                    const ChVectorDynamic<>& w,
                                    const double c) override;
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

    virtual void IntLoadConstraint_Ct(const unsigned int off, ChVectorDynamic<>& Qc, const double c) override;

    //
    // SOLVER INTERFACE (OLD)
    //

    virtual void VariablesFbReset() override;
    virtual void VariablesFbLoadForces(double factor = 1) override;
    virtual void VariablesQbLoadSpeed() override;
    virtual void VariablesFbIncrementMq() override;
    virtual void VariablesQbSetSpeed(double step = 0) override;
    virtual void InjectVariables(ChSystemDescriptor& mdescriptor) override;

    virtual void ConstraintsBiLoad_Ct(double factor = 1) override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;

  private:
    double rot_offset;

    ChVariablesGeneric variable;

    double aux_dt;  // used for integrating speed, = angle
    double aux_dtdt;

    bool avoid_angle_drift;
};

CH_CLASS_VERSION(ChLinkMotorRotationSpeed, 0)

}  // end namespace chrono

#endif
