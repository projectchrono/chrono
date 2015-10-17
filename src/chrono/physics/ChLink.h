//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHLINK_H
#define CHLINK_H

#include "physics/ChLinkBase.h"
#include "physics/ChBodyFrame.h"

namespace chrono {

// Forward references
class ChSystem;

///
/// Base class for joints betwenn two ChBodyFrame objects.
///
///  Links are objects which can be created to constrain two rigid
/// bodies (i.e. objects from the ChBody class) in 3D space, like with
/// revolute joints, guides, etc.
///
///  Note that there are many specializations of this base class,
/// for example the ChLinkEngine class inherits this base class and
/// implements specific functions to represent an engine between two
/// bodies, etc. etc. (In fact, this base ChLink class does basically
/// _nothing_ unless it is specialized by some child class).
///

class ChApi ChLink : public ChLinkBase {
    CH_RTTI(ChLink, ChLinkBase);

  protected:
    //
    // DATA
    //

    ChBodyFrame* Body1;
    ChBodyFrame* Body2;

    Vector react_force;   // store the xyz reactions, expressed in local coordinate system of link;
    Vector react_torque;  // store the torque reactions, expressed in local coordinate system of link;

  public:
    //
    // CONSTRUCTORS
    //
    ChLink();
    virtual ~ChLink();
    virtual void Copy(ChLink* source);
    virtual ChLink* new_Duplicate() = 0;

  public:
    //
    // FUNCTIONS
    //

    /// Get the type identifier of this link. Use if you don't want to use RTTI for performance.
    virtual int GetType() { return LNK_BASE; }

    /// Get the number of free degrees of freedom left by this link, between two bodies.
    virtual int GetLeftDOF() { return 6 - GetDOC(); }
    /// Get the number of scalar variables affected by constraints in this link
    virtual int GetNumCoords() { return 12; }

    /// Get the constrained body '1', the 'slave' body.
    ChBodyFrame* GetBody1() { return Body1; }
    /// Get the constrained body '2', the 'master' body.
    ChBodyFrame* GetBody2() { return Body2; }

    /// Get the link coordinate system, expressed relative to Body2 (the 'master'
    /// body). This represents the 'main' reference of the link: reaction forces
    /// and reaction torques are expressed in this coordinate system.
    /// By default is in the origin of Body2, but child classes should implement this.
    virtual ChCoordsys<> GetLinkRelativeCoords() { return CSYSNORM; }

    /// Get the link coordinate system in absolute reference.
    /// This represents the 'main' reference of the link: reaction forces
    /// and reaction torques are expressed in this coordinate system.
    /// Child classes should implement this.
    virtual ChCoordsys<> GetLinkAbsoluteCoords() { return GetLinkRelativeCoords() >> Body2->GetCoord(); }

    /// To get reaction force, expressed in link coordinate system:
    virtual ChVector<> Get_react_force() { return react_force; }
    /// To get reaction torque,  expressed in link coordinate system:
    virtual ChVector<> Get_react_torque() { return react_torque; }

    /// If some constraint is redundant, return to normal state  //***OBSOLETE***
    virtual int RestoreRedundant() { return 0; };  ///< \return number of changed constraints

    //
    // UPDATING FUNCTIONS
    //

    /// Given new time, current body state, updates
    /// time-dependant stuff in link state, for example
    /// motion laws, moving markers, etc.
    /// (Default: do nothing but setting new time.)
    virtual void UpdateTime(double mytime);

    // -----------COMPLETE UPDATE.
    // sequence:
    //			UpdateTime;

    /// This is an important function, which is called by the
    /// owner ChSystem at least once per integration step.
    /// It may update all auxiliary data of the link, such as
    /// matrices if any, etc.
    /// The inherited classes, for example the ChLinkMask, often
    /// implement specialized versions of this Update(time) function,
    /// because they might need to update inner states, forces, springs, etc.
    /// This base version, by default, simply updates the time.
    virtual void Update(double mytime, bool update_assets = true);

    /// As above, but with current time
    virtual void Update(bool update_assets = true);

    /// Called from a foreign software (i.e. a plugin, a CAD appl.), if any, to report
    /// that time has changed. Most often you can leave this unimplemented.
    virtual void UpdatedExternalTime(double prevtime, double time){};

    //
    // SERIALIZATION
    //

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive);

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive);
};

}  // END_OF_NAMESPACE____

#endif
