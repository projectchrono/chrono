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

    /// Get the position of the motor, as position of shaft 1 with respect to 2.
    /// This is an angle for rotational motors or a displacement for linear motors.
    virtual double GetMotorPos() const { return (shaft1->GetPos() - shaft2->GetPos()); }

    /// Get the motor speed, as speed of shaft 1 with respect to 2.
    virtual double GetMotorPosDt() const { return (shaft1->GetPosDt() - shaft2->GetPosDt()); }

    /// Get the motor acceleration, as accelerations of shaft 1 with respect to 2.
    virtual double GetMotorPosDt2() const { return (shaft1->GetPosDt2() - shaft2->GetPosDt2()); }

    /// Get number of full rotations for this shafts rotational motor.
    /// This function is meaningful only for a motor connecting two rotational shafts.
    virtual int GetMotorNumTurns() const { return int(GetMotorPos() / CH_2PI); }

    /// Get the motor angle in the range [-pi, +pi].
    /// This function is meaningful only for a motor connecting two rotational shafts.
    virtual double GetMotorAngleWrapped() const { return fmod(GetMotorPos(), CH_2PI); }

    /// Get the current motor load between shaft2 and shaft1, expressed as applied to shaft1.
    /// This represetns a torque for a rotational motor and a force for a linear motor.
    virtual double GetMotorLoad() const = 0;

    /// Get the reaction torque exchanged between the two shafts,
    /// considered as applied to the 1st axis.
    virtual double GetReaction1() const override { return (GetMotorLoad()); }

    /// Get the reaction torque exchanged between the two shafts,
    /// considered as applied to the 2nd axis.
    virtual double GetReaction2() const override { return -(GetMotorLoad()); }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;
};

CH_CLASS_VERSION(ChShaftsMotor, 0)

}  // end namespace chrono

#endif
