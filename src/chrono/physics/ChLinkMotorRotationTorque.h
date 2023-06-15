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

#ifndef CHLINKMOTORROTATIONTORQUE_H
#define CHLINKMOTORROTATIONTORQUE_H

#include "chrono/physics/ChLinkMotorRotation.h"

namespace chrono {

/// A motor that applies a torque between two frames on two bodies.
/// Differently from the ChLinkMotorRotationAngle and ChLinkMotorRotationSpeed,
/// this does not enforce precise motion via constraint.
/// Example of application:
/// - mimic a PID controlled system with some feedback (user-defined)
/// - force that is updated by a cosimulation
/// - force from a human-in-the-loop setpoint
/// Use SetTorqueFunction() to change to other torque function (default zero torque).

class ChApi ChLinkMotorRotationTorque : public ChLinkMotorRotation {
  public:
    ChLinkMotorRotationTorque();
    ChLinkMotorRotationTorque(const ChLinkMotorRotationTorque& other);
    virtual ~ChLinkMotorRotationTorque();

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkMotorRotationTorque* Clone() const override { return new ChLinkMotorRotationTorque(*this); }

    /// Set the torque function of time T(t).
    void SetTorqueFunction(const std::shared_ptr<ChFunction> function) { SetMotorFunction(function); }

    /// Gets the torque function F(t).
    std::shared_ptr<ChFunction> GetTorqueFunction() const { return GetMotorFunction(); }

    /// Get the current actuator reaction torque.
    virtual double GetMotorTorque() const override { return m_func->Get_y(GetChTime()); }

    void Update(double mytime, bool update_assets) override;

    //
    // STATE FUNCTIONS
    //
    virtual void IntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) override;

    //
    // SOLVER INTERFACE (OLD)
    //
    virtual void ConstraintsFbLoadForces(double factor = 1) override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;
};

CH_CLASS_VERSION(ChLinkMotorRotationTorque, 0)

}  // end namespace chrono

#endif
