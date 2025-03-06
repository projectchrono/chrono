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

#ifndef CH_SHAFTS_MOTOR_LOAD_H
#define CH_SHAFTS_MOTOR_LOAD_H

#include "chrono/functions/ChFunction.h"
#include "chrono/physics/ChShaftsMotor.h"

namespace chrono {

/// Motor to apply a load between two shafts.
/// Similar to the simplier ChShaftsAppliedTorque, this class specifies the load through a function of time.
/// Differently from the ChShaftsMotorPosition and ChShaftsMotorSpeed, this does not enforce precise motion via
/// constraint. Application examples:
/// - mimic a PID controlled system with some feedback
/// - force that is updated by a cosimulation
/// - force from a human-in-the-loop setpoint
class ChApi ChShaftsMotorLoad : public ChShaftsMotor {
  public:
    ChShaftsMotorLoad();
    ChShaftsMotorLoad(const ChShaftsMotorLoad& other);
    ~ChShaftsMotorLoad() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChShaftsMotorLoad* Clone() const override { return new ChShaftsMotorLoad(*this); }

    /// Set the load function L(t).
    /// This function outputs a torque for a rotational motor and a force for a linear motor.
    void SetLoadFunction(const std::shared_ptr<ChFunction> mf) { motor_function = mf; }

    /// Get the load function L(t).
    std::shared_ptr<ChFunction> GetLoadFunction() const { return motor_function; }

    /// Get the current actuator reaction torque in Nm.
    virtual double GetMotorLoad() const override { return this->motor_function->GetVal(this->GetChTime()); }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  private:
    std::shared_ptr<ChFunction> motor_function;

    virtual void Update(double time, bool update_assets) override;
    virtual void IntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) override;
    virtual void VariablesFbLoadForces(double factor) override;
};

CH_CLASS_VERSION(ChShaftsMotorLoad, 0)

}  // end namespace chrono

#endif
