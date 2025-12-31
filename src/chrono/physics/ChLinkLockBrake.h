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

#ifndef CHLINKBRAKE_H
#define CHLINKBRAKE_H

#include "chrono/physics/ChLinkLock.h"

namespace chrono {

/// Link representing a brake between two rigid bodies,
/// including the sticking effect.
/// It could be used to represent also linear brakes.
/// This constraint can behave also as a clutch.
///
///  ***OBSOLETE***: consider using a ChLinkMotorRotation and add a ChShaftsClutch between shafts
class ChApi ChLinkLockBrake : public ChLinkLock {
  public:
    enum class Mode { ROTATION, TRANSLATEX };

    ChLinkLockBrake();
    ChLinkLockBrake(const ChLinkLockBrake& other);
    virtual ~ChLinkLockBrake() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkLockBrake* Clone() const override { return new ChLinkLockBrake(*this); }

    virtual void UpdateForces(double time) override;

    virtual void SetDisabled(bool mdis) override;

    double GetBrakeTorque() const { return brake_torque; }

    void SetBrakeTorque(double mset) { brake_torque = mset; }

    double GetStickingCoeff() const { return stick_ratio; }

    /// Set the sticking coefficient, as a ratio respect to the brake torque.
    /// If less than 1, the brake will not have any sticking effect.
    void SetStickingCoeff(double mset) { stick_ratio = mset; }

    Mode GetBrakeMode() const { return brake_mode; }

    void SetBrakeMode(Mode mmode);

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  protected:
    double brake_torque;  ///< applied torque.
    double stick_ratio;  ///< static sticking torque = stick ratio * brake torque (if <1, sticking effect is turned off)

    Mode brake_mode;  ///< default works as traditional rotating brake, but can also be linear, on x

    int last_dir;     ///< 0= clockwise, 1= anticlockw.  -- internal
    bool must_stick;  ///< if true, change DOF mask to add link -- internal
};

CH_CLASS_VERSION(ChLinkLockBrake, 0)

}  // end namespace chrono

#endif
