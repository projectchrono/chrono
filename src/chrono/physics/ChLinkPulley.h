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

#ifndef CHLINKPULLEY_H
#define CHLINKPULLEY_H

#include "chrono/physics/ChLinkLock.h"

namespace chrono {

/// Class to create pulleys on two rigid bodies, connected by a belt.
/// The two bodies must be already connected to a truss by other
/// links, for example two revolute joints (ChLinkLockRevolute), because
/// this link constraints only the rotation.

class ChApi ChLinkPulley : public ChLinkLock {

    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChLinkPulley)

  protected:
    double tau;       ///< transmission coeff.
    double r1;        ///< radius of pulley in body1
    double r2;        ///< radius of pulley in body2
    double phase;     ///< mounting phase angle
    bool checkphase;  ///< keep pulleys always on phase

    double a1;  ///< auxiliary
    double a2;  ///< auxiliary

    double shaft_dist;  ///< distance between shafts

    ChVector<> belt_up1;   ///< upper segment of belt - end on body1.
    ChVector<> belt_up2;   ///< upper segment of belt - end on body2.
    ChVector<> belt_low1;  ///< lower segment of belt - end on body1.
    ChVector<> belt_low2;  ///< lower segment of belt - end on body2.

    ChFrame<double> local_shaft1;  ///< shaft1 pos & dir (as Z axis), relative to body1
    ChFrame<double> local_shaft2;  ///< shaft2 pos & dir (as Z axis), relative to body2

  public:
    ChLinkPulley();
    ChLinkPulley(const ChLinkPulley& other);
    virtual ~ChLinkPulley() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkPulley* Clone() const override { return new ChLinkPulley(*this); }

    /// Updates motion laws, marker positions, etc.
    virtual void UpdateTime(double mytime) override;

    /// Set radius of 1st pulley.
    void Set_r1(double mr);

    /// Set radius of 2nd pulley.
    void Set_r2(double mr);

    /// Get radius of 1st pulley.
    double Get_r1() const { return r1; }
    /// Get radius of 2nd pulley.
    double Get_r2() const { return r2; }

    /// Get the transmission ratio. Its value is assumed always positive.
    double Get_tau() const { return tau; }

    /// Get the initial phase of rotation of pulley A.
    double Get_phase() const { return phase; }
    /// Set the initial phase of rotation of pulley A.
    void Set_phase(double mset) { phase = mset; }

    /// If true, enforce check on exact phase between pulleys
    /// (otherwise after many simulation steps the phasing
    /// may be affected by numerical error accumulation).
    ///  By default, it is turned off, but for the simulation of
    /// synchro belts, this should be better turned on.
    ///  Note that, to ensure the correct phasing during the many
    /// rotations, an algorithm will use the a1 and a2 total rotation
    /// values, which might be affected by loss of numerical precision
    /// after few thousands of revolutions, so this is NOT suited to
    /// real-time simulators which must run for many hours.
    void Set_checkphase(bool mset) { checkphase = mset; }
    bool Get_checkphase() const { return checkphase; };

    /// Get total rotation of 1st pulley, respect to interaxis, in radians
    double Get_a1() const { return a1; }
    /// Get total rotation of 1st pulley, respect to interaxis, in radians
    double Get_a2() const { return a2; }
    /// Reset the total rotations of a1 and a2.
    void Reset_a1a2() { a1 = a2 = 0; }

    /// Get shaft position and direction, for 1st pulley, in body1-relative reference.
    /// The shaft direction is the Z axis of that frame.
    const ChFrame<double>& Get_local_shaft1() const { return local_shaft1; }
    /// Set shaft position and direction, for 1st pulley, in body1-relative reference.
    /// The shaft direction is the Z axis of that frame.  It should be parallel to shaft 2.
    /// Note that the origin of shaft position will be automatically shifted along
    /// shaft direction in order to have both pulleys on same plane.
    void Set_local_shaft1(ChFrame<double> mf) { local_shaft1 = mf; }

    /// Get shaft position and direction, for 2nd pulley, in body2-relative reference.
    /// The shaft direction is the Z axis of that frame.
    const ChFrame<double>& Get_local_shaft2() const { return local_shaft2; }
    /// Set shaft position and direction, for 2nd pulley, in body2-relative reference.
    /// The shaft direction is the Z axis of that frame.  It should be parallel to shaft 1.
    void Set_local_shaft2(ChFrame<double> mf) { local_shaft2 = mf; }

    /// Get shaft direction, for 1st pulley, in absolute reference
    ChVector<> Get_shaft_dir1();
    /// Get shaft direction, for 2nd pulley, in absolute reference
    ChVector<> Get_shaft_dir2();

    /// Get shaft position, for 1st pulley, in absolute reference
    ChVector<> Get_shaft_pos1();
    /// Get shaft position, for 2nd pulley, in absolute reference
    ChVector<> Get_shaft_pos2();

    /// Get the endpoint of belt, on pulley of body1, for the 'upper' segment,
    /// in absolute coordinates.
    ChVector<> Get_belt_up1() const { return belt_up1; }
    /// Get the endpoint of belt, on pulley of body2, for the 'upper' segment,
    /// in absolute coordinates.
    ChVector<> Get_belt_up2() const { return belt_up2; }
    /// Get the endpoint of belt, on pulley of body1, for the 'lower' segment,
    /// in absolute coordinates.
    ChVector<> Get_belt_low1() const { return belt_low1; }
    /// Get the endpoint of belt, on pulley of body1, for the 'lower' segment,
    /// in absolute coordinates.
    ChVector<> Get_belt_low2() const { return belt_low2; }

    /// Return distance between the two axes.
    double GetShaftsDistance() const { return shaft_dist; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;
};

CH_CLASS_VERSION(ChLinkPulley,0)

}  // end namespace chrono

#endif
