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

#ifndef CHLINKGEAR_H
#define CHLINKGEAR_H

#include "chrono/physics/ChLinkLock.h"

namespace chrono {

/// Gear link between two rigid bodies. This can
/// also be used to represent spur and bevel gears, and
/// it correctly handles the direction of transmitted force
/// given the teeth pressure angle.

class ChApi ChLinkLockGear : public ChLinkLock {
  protected:
    double tau;       ///< transmission coeff.
    double alpha;     ///< inclination of action line
    double beta;      ///< helix angle
    double phase;     ///< mounting phase angle
    bool checkphase;  ///< keep gear always on phase
    bool epicyclic;   ///< epiciclyc (gear 1 is internal to gear2)  if true.

    double a1;  ///< auxiliary
    double a2;  ///< auxiliary
    double r1;  ///< auxiliary
    double r2;  ///< auxiliary

    ChVector3d contact_pt;

    ChFrame<double> local_shaft1;  ///< shaft1 pos & dir (as Z axis), relative to body1
    ChFrame<double> local_shaft2;  ///< shaft2 pos & dir (as Z axis), relative to body2

  public:
    ChLinkLockGear();
    ChLinkLockGear(const ChLinkLockGear& other);
    virtual ~ChLinkLockGear() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkLockGear* Clone() const override { return new ChLinkLockGear(*this); }

    // Updates motion laws, marker positions, etc.
    virtual void UpdateTime(double time) override;

    /// Get the transmission ratio. Its value is assumed always positive,
    /// both for inner and outer gears (so use GetEpicyclic() to distinguish)
    double GetTransmissionRatio() const { return tau; }

    /// Set the transmission ratio. Its value is assumed always positive,
    /// both for inner and outer gears (so use SetEpicyclic() to distinguish)
    void SetTransmissionRatio(double mset) { tau = fabs(mset); }

    /// Set the transmission ratio given the number of teeth (or radius) of 1st gear
    /// and the number of teeth (or radius) of 2nd gear
    void SetTransmissionRatio(double mz1, double mz2) { tau = fabs(mz1 / mz2); }

    /// Get the pressure angle (usually 20 deg for typical gears)
    double GetPressureAngle() const { return alpha; }

    /// Set the pressure angle (usually 20 deg for typical gears)
    void SetPressureAngle(double mset) { alpha = mset; }

    /// Get the angle of teeth in bevel gears (0 deg for spur gears)
    double GetPitchAngle() const { return beta; }

    /// Set the angle of teeth in bevel gears (0 deg for spur gears)
    void SetPitchAngle(double mset) { beta = mset; }

    /// Get the initial phase of rotation of gear A respect to gear B
    double GetPhase() const { return phase; }

    /// Set the initial phase of rotation of gear A respect to gear B
    void SetPhase(double mset) { phase = mset; }

    /// If true, the bigger wheel has inner (internal) teeth
    bool GetEpicyclic() const { return epicyclic; }

    /// If true, the bigger wheel has inner (internal) teeth
    void SetEpicyclic(bool mset) { epicyclic = mset; }

    /// If true, enforce check on exact phase between gears
    /// (otherwise after many simulation steps the phasing
    /// may be affected by numerical error accumulation).
    /// By default, it is turned off.
    /// Note that, to ensure the correct phasing during the many
    /// rotations, an algorithm will use the a1 and a2 total rotation
    /// values, which might be affected by loss of numerical precision
    /// after few thousands of revolutions, so this is NOT suited to
    /// real-time simulators which must run for many hours.
    void SetEnforcePhase(bool mset) { checkphase = mset; }

    bool GetEnforcePhase() const { return checkphase; }

    /// Get total rotation of 1st gear, respect to interaxis, in radians
    double GetRotation1() const { return a1; }

    /// Get total rotation of 2nd gear, respect to interaxis, in radians
    double GetRotation2() const { return a2; }

    /// Reset the total rotations of a1 and a2.
    void ResetRotations() { a1 = a2 = 0; }

    /// Get radius of 1st gear (depends on axis position and t.ratio)
    double GetRadius1() const { return r1; }

    /// Get radius of 2nd gear (depends on axis position and t.ratio)
    double GetRadius2() const { return r2; }

    /// Get shaft position and direction, for 1st gear, in body1-relative reference.
    /// The shaft direction is the Z axis of that frame.
    const ChFrame<double>& GetFrameShaft1() const { return local_shaft1; }

    /// Set shaft position and direction, for 1st gear, in body1-relative reference.
    /// The shaft direction is the Z axis of that frame.
    /// Note that the origin of shaft position may be automatically shifted along
    /// shaft direction in order to have both wheels on same plane (for spur gears) -
    /// same sphere (for bevel gears).
    void SetFrameShaft1(ChFrame<double> mf) { local_shaft1 = mf; }

    /// Get shaft position and direction, for 2nd gear, in body2-relative reference.
    /// The shaft direction is the Z axis of that frame.
    const ChFrame<double>& GetFrameShaft2() const { return local_shaft2; }

    /// Set shaft position and direction, for 2nd gear, in body2-relative reference.
    /// The shaft direction is the Z axis of that frame.
    void SetFrameShaft2(ChFrame<double> mf) { local_shaft2 = mf; }

    /// Get shaft direction, for 1st gear, in absolute reference
    ChVector3d GetDirShaft1() const;

    /// Get shaft direction, for 2nd gear, in absolute reference
    ChVector3d GetDirShaft2() const;

    /// Get shaft position, for 1st gear, in absolute reference
    ChVector3d GetPosShaft1() const;

    /// Get shaft position, for 2nd gear, in absolute reference
    ChVector3d GetPosShaft2() const;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;
};

CH_CLASS_VERSION(ChLinkLockGear, 0)

}  // end namespace chrono

#endif
