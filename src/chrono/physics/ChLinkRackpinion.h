//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//
// File Author: A. Tasora

#ifndef CHLINKRACKPINION_H
#define CHLINKRACKPINION_H

#include "physics/ChLinkMate.h"

namespace chrono {

///
/// Rack-pinion link between two body frames.
/// It correctly handles the direction of transmitted force
/// given the teeth pressure angle.
///

class ChApi ChLinkRackpinion : public ChLinkMateGeneric {
    CH_RTTI(ChLinkRackpinion, ChLinkMateGeneric);

  protected:
    double R;        // primitive radius of the pinion
    double alpha;    // inclination of action line
    double beta;     // helix angle
    double phase;    // mounting phase angle
    int checkphase;  // keep gear always on phase

    double a1;  // auxiliary

    ChVector<> contact_pt;

    ChFrame<double> local_pinion;  // pinion shaft pos & dir (frame Z axis), relative to body1
    ChFrame<double> local_rack;    // rack direction (frame X axis), relative to body2

  public:
    // builders and destroyers
    ChLinkRackpinion();
    virtual ~ChLinkRackpinion();
    virtual void Copy(ChLinkRackpinion* source);
    virtual ChLink* new_Duplicate();  // always return base link class pointer

    // UPDATING FUNCTIONS - "gear" custom implementations

    // Updates aux frames positions
    virtual void UpdateTime(double mytime);

    // data get/set

    /// Get the primitive radius of the pinion.
    double GetPinionRadius() { return R; };
    /// Set the primitive radius of the pinion.
    void SetPinionRadius(double mR) { R = mR; };

    /// Get the pressure angle (usually 20� for typical gears)
    double GetAlpha() { return alpha; };
    /// Set the pressure angle (usually 20� for typical gears)
    void SetAlpha(double mset) { alpha = mset; }

    /// Get the angle of teeth in bevel gears (0� for spur gears)
    double GetBeta() { return beta; };
    /// Set the angle of teeth in bevel gears (0� for spur gears)
    void SetBeta(double mset) { beta = mset; }

    /// Get the initial phase of rotation of pinion respect to rack
    double GetPhase() { return phase; };
    /// Set the initial phase of rotation of pinion respect to rack
    void SetPhase(double mset) { phase = mset; }

    /// If true, enforce check on exact phase between gears
    /// (otherwise after many simulation steps the phasing
    /// may be affected by numerical error accumulation).
    /// By default, it is turned off.
    /// Note that, to ensure the correct phasing during the many
    /// rotations, an algorithm will update an accumulator with total rotation
    /// values, which might be affected by loss of numerical precision
    /// after few thousands of revolutions; keep in mind this if you do
    /// real-time simulations which must run for many hours.
    void SetCheckphase(int mset) { checkphase = mset; }
    int GetCheckphase() { return checkphase; };

    /// Get total rotation of 1st gear, respect to interaxis, in radians
    double Get_a1() { return a1; };
    /// Reset the total rotations of a1 and a2.
    void Reset_a1() { a1 = 0; };

    /// Set pinion shaft position and direction, in body1-relative reference.
    /// The shaft direction is the Z axis of that frame.
    void SetPinionFrame(ChFrame<double> mf) { local_pinion = mf; }
    /// Get pinion shaft position and direction, in body1-relative reference.
    /// The shaft direction is the Z axis of that frame.
    ChFrame<double> GetPinionFrame() { return local_pinion; }

    /// Set rack position and direction, in body2-relative reference.
    /// The rack direction is the X axis of that frame.
    void SetRackFrame(ChFrame<double> mf) { local_rack = mf; }
    /// Get rack position and direction, in body2-relative reference.
    /// The rack direction is the X axis of that frame.
    ChFrame<double> GetRackFrame() { return local_rack; }

    /// Get pinion shaft direction in absolute reference
    ChVector<> GetAbsPinionDir();
    /// Get pinion position in absolute reference
    ChVector<> GetAbsPinionPos();

    /// Get rack direction in absolute reference
    ChVector<> GetAbsRackDir();
    /// Get rack position in absolute reference
    ChVector<> GetAbsRackPos();

    //
    // SERIALIZATION
    //

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive);

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive);
};

//////////////////////////////////////////////////////
//////////////////////////////////////////////////////

}  // END_OF_NAMESPACE____

#endif
