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

#ifndef CHLINKMOTORROTATION_H
#define CHLINKMOTORROTATION_H

#include "chrono/physics/ChLinkMotor.h"

namespace chrono {

/// Base class for all rotational "motor" constraints between
/// two frames on two bodies. Motors of this type assume that
/// the spindle is directed along Z direction of the master frame.
/// Look for children classes for specialized behaviors,
/// for example chrono::ChLinkMotorRotationAngle

class ChApi ChLinkMotorRotation : public ChLinkMotor {
  public:
    /// Type of spindle constraint.
    enum class SpindleConstraint { FREE, REVOLUTE, CYLINDRICAL, OLDHAM };

    ChLinkMotorRotation();
    ChLinkMotorRotation(const ChLinkMotorRotation& other);
    virtual ~ChLinkMotorRotation();

    /// Sets which movements (of frame 1 respect to frame 2) are constrained.
    /// By default, acts as bearing, like a revolute joint.
    /// Note that the Z direction is the actuated one, and is never affected by this option.
    void SetSpindleConstraint(const SpindleConstraint mconstraint);

    /// Sets which movements (of frame 1 respect to frame 2) are constrained.
    /// By default, acts as bearing, like a revolute joint.
    /// Note that the Z direction is the actuated one, and is never affected by this option.
    void SetSpindleConstraint(bool mc_x, bool mc_y, bool mc_z, bool mc_rx, bool mc_ry);

    /// Get the motor rotation angle [rad].
    /// The value takes into account also multiple turns, so it is not limited to any angle range.
    /// Refer to GetMotorAngleWrapped() to get the angle in the range [-PI ... +PI].
    virtual double GetMotorAngle() const { return mrot; }

    /// Get the number of complete turns of the motor.
    virtual int GetMotorNumTurns() const { return std::floor(mrot / CH_2PI); }

    /// Get the motor rotation angle [rad] in the range [-PI..+PI].
    /// To retrieve the complete angle value, use GetMotorAngle().
    virtual double GetMotorAngleWrapped() const { return fmod(mrot, CH_2PI); }

    /// Get the current actuator speed [rad/s].
    virtual double GetMotorAngleDt() const { return mrot_dt; }

    /// Get the current actuator acceleration [rad/s^2].
    virtual double GetMotorAngleDt2() const { return mrot_dtdt; }

    /// Get the current actuator reaction torque.
    virtual double GetMotorTorque() const = 0;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

    /// Return a string describing the specified motor spindle constraint type.
    static std::string GetSpindleTypeString(SpindleConstraint type);

  protected:
    // aux data for optimization
    double mrot;
    double mrot_dt;
    double mrot_dtdt;

    virtual void Update(double time, bool update_assets) override;
};

CH_CLASS_VERSION(ChLinkMotorRotation, 0)

}  // end namespace chrono

#endif
