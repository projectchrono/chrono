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

#ifndef CHSHAFTSMOTORTORQUE_H
#define CHSHAFTSMOTORTORQUE_H

#include "chrono/functions/ChFunction.h"
#include "chrono/physics/ChShaftsMotor.h"

namespace chrono {

/// Motor to apply a torque between two shafts.
/// Similar to the simplier ChShaftsAppliedTorque, this class specifies the torque through a function of time.
/// Differently from the ChShaftsMotorAngle and ChShaftsMotorSpeed, this does not enforce precise motion via constraint.
/// Application examples:
/// - mimic a PID controlled system with some feedback
/// - force that is updated by a cosimulation
/// - force from a human-in-the-loop setpoint
class ChApi ChShaftsMotorTorque : public ChShaftsMotor {
  public:
    ChShaftsMotorTorque();
    ChShaftsMotorTorque(const ChShaftsMotorTorque& other);
    ~ChShaftsMotorTorque() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChShaftsMotorTorque* Clone() const override { return new ChShaftsMotorTorque(*this); }

    /// Set the torque function T(t), in Nm.
    void SetTorqueFunction(const std::shared_ptr<ChFunction> mf) { f_torque = mf; }

    /// Get the torque function F(t).
    std::shared_ptr<ChFunction> GetTorqueFunction() { return f_torque; }

    /// Get the current actuator reaction torque in Nm.
    virtual double GetMotorTorque() const override { return this->f_torque->GetVal(this->GetChTime()); }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  private:
    std::shared_ptr<ChFunction> f_torque;

    virtual void Update(double mytime, bool update_assets = true) override;
    virtual void IntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) override;
    virtual void VariablesFbLoadForces(double factor) override;
};

CH_CLASS_VERSION(ChShaftsMotorTorque, 0)

}  // end namespace chrono

#endif
