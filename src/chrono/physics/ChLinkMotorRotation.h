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
    /// Type of guide constraint
    enum class SpindleConstraint { FREE, REVOLUTE, CYLINDRICAL, OLDHAM };

    ChLinkMotorRotation();
    ChLinkMotorRotation(const ChLinkMotorRotation& other);
    virtual ~ChLinkMotorRotation();

    /// Sets which movements (of frame 1 respect to frame 2) are constrained.
    /// By default, acts as bearing, like a revolute joint.
    /// Note that the Z direction is the motorized one, and is never affected by
    /// this option.
    void SetSpindleConstraint(const SpindleConstraint mconstraint);

    /// Sets which movements (of frame 1 respect to frame 2) are constrained.
    /// By default, acts as bearing, like a revolute joint.
    /// Note that the Z direction is the motorized one, and is never affected by
    /// this option.
    void SetSpindleConstraint(bool mc_x, bool mc_y, bool mc_z, bool mc_rx, bool mc_ry);

    /// Get the current actuator rotation [rad], including error etc.
    /// This rotation keeps track of multiple turns, so it is not limited in periodic -PI..+PI,
    /// and rotation accumulates indefinitely. Use GetMotorRotTurns() and GetMotorRotPeriodic() otherwise.
    virtual double GetMotorRot() const { return mrot; }

    /// In case of multi-turns, gets the current actuator number of (integer) rotations,
    virtual int GetMotorRotTurns() const { return int(mrot / CH_C_2PI); }

    /// In case of multi-turns, gets the current actuator rotation angle [rad], in periodic -PI..+PI.
    virtual double GetMotorRotPeriodic() const { return fmod(mrot, CH_C_2PI); }

    /// Get the current actuator speed [rad/s], including error etc.
    virtual double GetMotorRot_dt() const { return mrot_dt; }

    /// Get the current actuator acceleration [rad/s^2], including error etc.
    virtual double GetMotorRot_dtdt() const { return mrot_dtdt; }

    /// Get the current actuator reaction torque [Nm]
    virtual double GetMotorTorque() const = 0;

    void Update(double mytime, bool update_assets) override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;

  protected:
    // aux data for optimization
    double mrot;
    double mrot_dt;
    double mrot_dtdt;
};

CH_CLASS_VERSION(ChLinkMotorRotation, 0)

}  // end namespace chrono

#endif
