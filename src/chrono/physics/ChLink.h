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

#ifndef CHLINK_H
#define CHLINK_H

#include "chrono/physics/ChBodyFrame.h"
#include "chrono/physics/ChLinkBase.h"

namespace chrono {

// Forward references
class ChSystem;

/// Base class for joints between two ChBodyFrame objects.
///
/// Links (joints) are objects which can be created to constrain two rigid bodies
/// (i.e. objects from the ChBody class) in 3D space.
///
/// Note that there are many specializations of this base class. In fact, this base
/// ChLink class does basically nothing, unless it is specialized by some child class.
class ChApi ChLink : public ChLinkBase {

  protected:
    ChBodyFrame* Body1;       ///< first connected body
    ChBodyFrame* Body2;       ///< second connected body
    ChVector<> react_force;   ///< store the xyz reactions, expressed in local coordinate system of link;
    ChVector<> react_torque;  ///< store the torque reactions, expressed in local coordinate system of link;

  public:
    ChLink() : Body1(NULL), Body2(NULL), react_force(VNULL), react_torque(VNULL) {}
    ChLink(const ChLink& other);
    virtual ~ChLink() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLink* Clone() const override { return new ChLink(*this); }

    /// Get the number of free degrees of freedom left by this link, between two bodies.
    int GetLeftDOF() { return 6 - GetDOC(); }

    /// Get the number of scalar variables affected by constraints in this link
    virtual int GetNumCoords() override { return 12; }

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
    virtual ChCoordsys<> GetLinkAbsoluteCoords() override { return GetLinkRelativeCoords() >> Body2->GetCoord(); }

    /// To get reaction force, expressed in link coordinate system:
    virtual ChVector<> Get_react_force() override { return react_force; }
    /// To get reaction torque,  expressed in link coordinate system:
    virtual ChVector<> Get_react_torque() override { return react_torque; }

    /// If some constraint is redundant, return to normal state  //***OBSOLETE***
    virtual int RestoreRedundant() { return 0; }

    //
    // UPDATING FUNCTIONS
    //

    /// Given new time, current body state, update time-dependent quantities in link state,
    /// for example motion laws, moving markers, etc.
    /// Default: do nothing except setting new time.
    virtual void UpdateTime(double mytime);

    /// This is an important function, which is called by the
    /// owner ChSystem at least once per integration step.
    /// It may update all auxiliary data of the link, such as
    /// matrices if any, etc.
    /// The inherited classes, for example the ChLinkMask, often
    /// implement specialized versions of this Update(time) function,
    /// because they might need to update inner states, forces, springs, etc.
    /// This base version, by default, simply updates the time.
    virtual void Update(double mytime, bool update_assets = true) override;

    /// As above, but with current time
    virtual void Update(bool update_assets = true) override;

    /// Called from a external package (i.e. a plugin, a CAD app.) to report
    /// that time has changed. Most often you can leave this unimplemented.
    virtual void UpdatedExternalTime(double prevtime, double time) {}

    //
    // SERIALIZATION
    //

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;
};

CH_CLASS_VERSION(ChLink,0)

}  // end namespace chrono

#endif
