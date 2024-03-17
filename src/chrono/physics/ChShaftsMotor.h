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

#ifndef CHSHAFTSMOTOR_H
#define CHSHAFTSMOTOR_H

#include "chrono/physics/ChShaftsCouple.h"
#include "chrono/solver/ChConstraintTwoGeneric.h"

namespace chrono {

/// Base class for all motors between two 1D elements of ChShaft class.
class ChApi ChShaftsMotor : public ChShaftsCouple {
  public:
    ChShaftsMotor() {}
    ChShaftsMotor(const ChShaftsMotor& other) {}
    virtual ~ChShaftsMotor() {}

    /// Get the actual angle rotation [rad] of the motor, in terms of phase of shaft 1 respect to 2.
    virtual double GetMotorAngle() const { return (shaft1->GetPos() - shaft2->GetPos()); }

    /// Get the actual speed [rad/s] of the motor, in terms of speed of shaft 1 respect to 2.
    virtual double GetMotorAngleDer() const { return (shaft1->GetPosDer() - shaft2->GetPosDer()); }

    /// Get the actual acceleration [rad/s^2] of the motor, in terms of accel. of shaft 1 respect to 2.
    virtual double GetMotorAngleDer2() const { return (shaft1->GetPosDer2() - shaft2->GetPosDer2()); }

    /// In case of multi-turns, gets the current actuator number of (integer) rotations:
    virtual int GetMotorNumTurns() const { return int(GetMotorAngle() / CH_C_2PI); }

    /// In case of multi-turns, gets the current actuator rotation angle [rad], in periodic -PI..+PI.
    virtual double GetMotorAngleWrapped() const { return fmod(GetMotorAngle(), CH_C_2PI); }

    /// Get the current motor torque between shaft2 and shaft1, expressed as applied to shaft1
    virtual double GetMotorTorque() const = 0;

    /// Get the reaction torque exchanged between the two shafts,
    /// considered as applied to the 1st axis.
    virtual double GetTorqueReactionOn1() const override { return (GetMotorTorque()); }

    /// Get the reaction torque exchanged between the two shafts,
    /// considered as applied to the 2nd axis.
    virtual double GetTorqueReactionOn2() const override { return -(GetMotorTorque()); }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;
};

CH_CLASS_VERSION(ChShaftsMotor, 0)

}  // end namespace chrono

#endif
