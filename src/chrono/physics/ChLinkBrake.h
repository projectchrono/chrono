//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010-2011 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHLINKBRAKE_H
#define CHLINKBRAKE_H

///////////////////////////////////////////////////
//
//   ChLink.h
//
//
//   Classes for brakes and clutches between
//   two rotating bodies.
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "physics/ChLinkLock.h"

namespace chrono {

///
/// Link representing a brake between two rigid bodies,
/// including the sticking effect.
/// It could be used to represent also linear brakes.
/// This constraint can behave also as a clutch.
///  ***OBSOLETE*** better add ChLinkEngine in ENG_MODE_TO_POWERTRAIN_SHAFT mode, and add a ChShaftsClutch between
///  shafts

class ChApi ChLinkBrake : public ChLinkLock {
    CH_RTTI(ChLinkBrake, ChLinkLock);

  protected:
    double brake_torque;  // applied torque.
    double stick_ratio;   // static sticking torque = stick ratio * brake torque
                          // (if <1, sticking effect is turned off)

    int brake_mode;  // default works as traditional rotating brake, but can also be linear, on x

    enum eChBrmode { BRAKE_ROTATION = 0, BRAKE_TRANSLATEX };

    int last_dir;    // 0= clockwise, 1= anticlockw.  -- internal
    int must_stick;  // if true, change DOF mask to add link -- internal
  public:
    // builders and destroyers
    ChLinkBrake();
    virtual ~ChLinkBrake();
    virtual void Copy(ChLinkBrake* source);
    virtual ChLink* new_Duplicate();  // always return base link class pointer

    // UPDATING FUNCTION  -apply braking force-
    virtual void UpdateTime(double time);
    virtual void UpdateForces(double mytime);

    virtual void SetDisabled(bool mdis);

    // data get/set
    double Get_brake_torque() { return brake_torque; };
    void Set_brake_torque(double mset) { brake_torque = mset; }
    double Get_stick_ratio() { return stick_ratio; };
    void Set_stick_ratio(double mset) { stick_ratio = mset; }
    int Get_brake_mode() { return brake_mode; };
    void Set_brake_mode(int mmode);

    // STREAMING
    virtual void StreamIN(ChStreamInBinary& mstream);
    virtual void StreamOUT(ChStreamOutBinary& mstream);
};

//////////////////////////////////////////////////////
//////////////////////////////////////////////////////

}  // END_OF_NAMESPACE____

#endif
