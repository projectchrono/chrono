// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
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

class ChApi ChLinkGear : public ChLinkLock {

    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChLinkGear)

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

    ChVector<> contact_pt;

    ChFrame<double> local_shaft1;  ///< shaft1 pos & dir (as Z axis), relative to body1
    ChFrame<double> local_shaft2;  ///< shaft2 pos & dir (as Z axis), relative to body2

  public:
    ChLinkGear();
    ChLinkGear(const ChLinkGear& other);
    virtual ~ChLinkGear() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkGear* Clone() const override { return new ChLinkGear(*this); }

    // Updates motion laws, marker positions, etc.
    virtual void UpdateTime(double mytime) override;

    /// Get the transmission ratio. Its value is assumed always positive,
    /// both for inner and outer gears (so use Get_epicyclic() to distinguish)
    double Get_tau() const { return tau; }
    /// Set the transmission ratio. Its value is assumed always positive,
    /// both for inner and outer gears (so use Set_epicyclic() to distinguish)
    void Set_tau(double mset) { tau = fabs(mset); }
    /// Set the transmission ratio given the number of teeths (or radius) of 1st gear
    /// and the number of teeths (or radius) of 2nd gear
    void Set_tau(double mz1, double mz2) { tau = fabs(mz1 / mz2); }

    /// Get the pressure angle (usually 20° for typical gears)
    double Get_alpha() const { return alpha; }
    /// Set the pressure angle (usually 20° for typical gears)
    void Set_alpha(double mset) { alpha = mset; }

    /// Get the angle of teeth in bevel gears (0° for spur gears)
    double Get_beta() const { return beta; }
    /// Set the angle of teeth in bevel gears (0° for spur gears)
    void Set_beta(double mset) { beta = mset; }

    /// Get the initial phase of rotation of gear A respect to gear B
    double Get_phase() const { return phase; }
    /// Set the initial phase of rotation of gear A respect to gear B
    void Set_phase(double mset) { phase = mset; }

    /// If true, the bigger wheel has inner (internal) teeth
    bool Get_epicyclic() const { return epicyclic; }
    /// If true, the bigger wheel has inner (internal) teeth
    void Set_epicyclic(bool mset) { epicyclic = mset; }

    /// If true, enforce check on exact phase between gears
    /// (otherwise after many simulation steps the phasing
    /// may be affected by numerical error accumulation).
    /// By default, it is turned off.
    /// Note that, to ensure the correct phasing during the many
    /// rotations, an algorithm will use the a1 and a2 total rotation
    /// values, which might be affected by loss of numerical precision
    /// after few thousands of revolutions, so this is NOT suited to
    /// real-time simulators which must run for many hours.
    void Set_checkphase(bool mset) { checkphase = mset; }
    bool Get_checkphase() const { return checkphase; }

    /// Get total rotation of 1st gear, respect to interaxis, in radians
    double Get_a1() const { return a1; }
    /// Get total rotation of 1st gear, respect to interaxis, in radians
    double Get_a2() const { return a2; }
    /// Reset the total rotations of a1 and a2.
    void Reset_a1a2() { a1 = a2 = 0; }

    /// Get radius of 1st gear (depends on axis position and t.ratio)
    double Get_r1() const { return r1; }
    /// Get radius of 2nd gear (depends on axis position and t.ratio)
    double Get_r2() const { return r2; }

    /// Get shaft position and direction, for 1st gear, in body1-relative reference.
    /// The shaft direction is the Z axis of that frame.
    const ChFrame<double>& Get_local_shaft1() const { return local_shaft1; }
    /// Set shaft position and direction, for 1st gear, in body1-relative reference.
    /// The shaft direction is the Z axis of that frame.
    /// Note that the origin of shaft position may be automatically shifted along
    /// shaft direction in order to have both wheels on same plane (for spur gears) -
    /// same sphere (for bevel gears).
    void Set_local_shaft1(ChFrame<double> mf) { local_shaft1 = mf; }

    /// Get shaft position and direction, for 2nd gear, in body2-relative reference.
    /// The shaft direction is the Z axis of that frame.
    const ChFrame<double>& Get_local_shaft2() const { return local_shaft2; }
    /// Set shaft position and direction, for 2nd gear, in body2-relative reference.
    /// The shaft direction is the Z axis of that frame.
    void Set_local_shaft2(ChFrame<double> mf) { local_shaft2 = mf; }

    /// Get shaft direction, for 1st gear, in absolute reference
    ChVector<> Get_shaft_dir1() const;
    /// Get shaft direction, for 2nd gear, in absolute reference
    ChVector<> Get_shaft_dir2() const;

    /// Get shaft position, for 1st gear, in absolute reference
    ChVector<> Get_shaft_pos1() const;
    /// Get shaft position, for 2nd gear, in absolute reference
    ChVector<> Get_shaft_pos2() const;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;
};

CH_CLASS_VERSION(ChLinkGear,0)

}  // end namespace chrono

#endif
