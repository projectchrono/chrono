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

#ifndef CHLINKBRAKE_H
#define CHLINKBRAKE_H

#include "chrono/physics/ChLinkLock.h"

namespace chrono {

/// Link representing a brake between two rigid bodies,
/// including the sticking effect.
/// It could be used to represent also linear brakes.
/// This constraint can behave also as a clutch.
///  ***OBSOLETE*** better add ChLinkEngine in ENG_MODE_TO_POWERTRAIN_SHAFT mode, and add a ChShaftsClutch between
///  shafts

class ChApi ChLinkBrake : public ChLinkLock {

    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChLinkBrake)

  protected:
    double brake_torque;  ///< applied torque.
    double stick_ratio;  ///< static sticking torque = stick ratio * brake torque (if <1, sticking effect is turned off)

    int brake_mode;  ///< default works as traditional rotating brake, but can also be linear, on x

    enum eChBrmode { BRAKE_ROTATION = 0, BRAKE_TRANSLATEX };

    int last_dir;     ///< 0= clockwise, 1= anticlockw.  -- internal
    bool must_stick;  ///< if true, change DOF mask to add link -- internal

  public:
    ChLinkBrake();
    ChLinkBrake(const ChLinkBrake& other);
    virtual ~ChLinkBrake() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkBrake* Clone() const override { return new ChLinkBrake(*this); }

    virtual void UpdateTime(double time) override;
    virtual void UpdateForces(double mytime) override;

    virtual void SetDisabled(bool mdis) override;

    double Get_brake_torque() { return brake_torque; };
    void Set_brake_torque(double mset) { brake_torque = mset; }
    double Get_stick_ratio() { return stick_ratio; };
    void Set_stick_ratio(double mset) { stick_ratio = mset; }
    int Get_brake_mode() { return brake_mode; };
    void Set_brake_mode(int mmode);

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;
};

CH_CLASS_VERSION(ChLinkBrake,0)

}  // end namespace chrono

#endif
