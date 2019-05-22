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

#ifndef CHLINKMOTORLINEARSPEED_H
#define CHLINKMOTORLINEARSPEED_H

#include "chrono/physics/ChLinkMotorLinear.h"
#include "chrono/solver/ChVariablesGeneric.h"

namespace chrono {

/// A linear motor that enforces the speed v(t) between two frames on two bodies, using a rheonomic constraint.
/// Note: no compliance is allowed, so if the actuator hits an undeformable obstacle it hits a pathological
/// situation and the solver result can be unstable/unpredictable.
/// Think at it as a servo drive with "infinitely stiff" control.
/// This type of motor is very easy to use, stable and efficient, and should be used if the 'infinitely stiff'
/// control assumption is a good approximation of what you simulate (e.g., very good and reactive controllers).
/// By default it is initialized with constant speed: df/dt= 1.
/// Use SetSpeedFunction() to change to other speed functions.

class ChApi ChLinkMotorLinearSpeed : public ChLinkMotorLinear {
  public:
    ChLinkMotorLinearSpeed();
    ChLinkMotorLinearSpeed(const ChLinkMotorLinearSpeed& other);
    virtual ~ChLinkMotorLinearSpeed();

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkMotorLinearSpeed* Clone() const override { return new ChLinkMotorLinearSpeed(*this); }

    /// Set the speed function of time v(t).
    /// To prevent acceleration pikes, this function should be C0 continuous.
    void SetSpeedFunction(const std::shared_ptr<ChFunction> function) { SetMotorFunction(function); }

    /// Get the speed function v(t).
    std::shared_ptr<ChFunction> GetSpeedFunction() const { return GetMotorFunction(); }

    /// Get initial offset, by default = 0.
    void SetMotionOffset(double mo) { pos_offset = mo; }

    /// Get initial offset.
    double GetMotionOffset() { return pos_offset; }

    /// Set if the constraint must avoid position drift. If true, it
    /// means that the constraint is satisfied also at the position level,
    /// by integrating the velocity in a separate auxiliary state. Default, true.
    void SetAvoidPositionDrift(bool mb) { this->avoid_position_drift = mb; }

    /// Set if the constraint is in "avoid position drift" mode.
    bool GetAvoidPositionDrift() { return this->avoid_position_drift; }

    /// Get the current actuator reaction force [N]
    virtual double GetMotorForce() const override { return -this->react_force.x(); }

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
    double pos_offset;

    ChVariablesGeneric variable;

    double aux_dt;  // used for integrating speed, = pos
    double aux_dtdt;

    bool avoid_position_drift;
};

CH_CLASS_VERSION(ChLinkMotorLinearSpeed, 0)

}  // end namespace chrono

#endif
