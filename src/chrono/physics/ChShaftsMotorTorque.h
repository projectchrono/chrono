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

#include "chrono/motion_functions/ChFunction.h"
#include "chrono/physics/ChShaftsMotor.h"

namespace chrono {

/// A motor that applies a torque between two ChShaft shafts, a bit
/// like the simplier ChShaftsTorque, with some differences, ex. here
/// torque can be a ChFunction.
/// Differently from the ChLinkMotorRotationAngle and
/// ChLinkMotorRotationSpeed, this does not enforce precise
/// motion via constraint.
/// Example of application:
/// - mimic a PID controlled system with
///   some feedback (that is up to you too implement)
/// - force that is updated by a cosimulation
/// - force from a man-in-the-loop setpoint
/// Use SetTorqueFunction() to change to other force function (by
/// default is no force), possibly introduce some custom ChFunction
/// of yours that is updated at each time step.

class ChApi ChShaftsMotorTorque : public ChShaftsMotorBase {
  private:
    std::shared_ptr<ChFunction> f_torque;

  public:
    ChShaftsMotorTorque();
    ChShaftsMotorTorque(const ChShaftsMotorTorque& other);
    ~ChShaftsMotorTorque() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChShaftsMotorTorque* Clone() const override { return new ChShaftsMotorTorque(*this); }

    /// Sets the torque function T(t). In [Nm]. It is a function of time.
    void SetTorqueFunction(const std::shared_ptr<ChFunction> mf) { f_torque = mf; }

    /// Gets the torque function F(t).
    std::shared_ptr<ChFunction> GetTorqueFunction() { return f_torque; }

    /// Get the current actuator reaction torque [Nm]
    virtual double GetMotorTorque() const override { return this->f_torque->Get_y(this->GetChTime()); }

    /// Update all auxiliary data
    virtual void Update(double mytime, bool update_assets = true) override;

    // (override/implement interfaces for global state vectors, see ChPhysicsItem for comments.)
    virtual void IntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) override;

    // Old...
    virtual void VariablesFbLoadForces(double factor) override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;
};

CH_CLASS_VERSION(ChShaftsMotorTorque, 0)

}  // end namespace chrono

#endif
