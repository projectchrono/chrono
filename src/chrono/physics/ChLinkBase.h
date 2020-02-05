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

#ifndef CHLINKBASE_H
#define CHLINKBASE_H

#include "chrono/core/ChLog.h"
#include "chrono/physics/ChPhysicsItem.h"

namespace chrono {

/// Base class for all types of constraints that act like mechanical joints ('links') in 3D space.
class ChApi ChLinkBase : public ChPhysicsItem {

  protected:
    bool disabled;  ///< all constraints of link disabled because of user needs
    bool valid;     ///< link data is valid
    bool broken;    ///< link is broken because of excessive pulling/pushing.

  public:
    ChLinkBase() : broken(false), valid(true), disabled(false) {}
    ChLinkBase(const ChLinkBase& other);
    virtual ~ChLinkBase() {}

    /// Tells if the link data is currently valid.
    /// (i.e. pointers to other items are correct)
    bool IsValid() { return valid; }
    /// Set the status of link validity
    void SetValid(bool mon) { valid = mon; }

    /// Tells if all constraints of this link are currently turned on or off by the user.
    bool IsDisabled() { return disabled; }
    /// User can use this to enable/disable all the constraint of the link as desired.
    virtual void SetDisabled(bool mdis) { disabled = mdis; }

    /// Tells if the link is broken, for excess of pulling/pushing.
    bool IsBroken() { return broken; }
    /// Set the 'broken' status vof this link.
    virtual void SetBroken(bool mon) { broken = mon; }

    /// An important function!
    /// Tells if the link is currently active, in general,
    /// that is tells if it must be included into the system solver or not.
    /// This method cumulates the effect of various flags (so a link may
    /// be not active either because disabled, or broken, or not valid)
    bool IsActive() { return (valid && !disabled && !broken); }

    /// Get the number of scalar variables affected by constraints in this link
    virtual int GetNumCoords() = 0;

    /// Get the link coordinate system in absolute reference.
    /// This represents the 'main' reference of the link: reaction forces
    /// and reaction torques are expressed in this coordinate system.
    /// Child classes should implement this.
    virtual ChCoordsys<> GetLinkAbsoluteCoords() { return CSYSNORM; }

    /// Get the master coordinate system for the assets, in absolute reference.
    /// (should be implemented by children classes)
    virtual ChFrame<> GetAssetsFrame(unsigned int nclone = 0) override { return ChFrame<>(GetLinkAbsoluteCoords()); }

    /// To get reaction force, expressed in link coordinate system:
    virtual ChVector<> Get_react_force() { return VNULL; }
    /// To get reaction torque,  expressed in link coordinate system:
    virtual ChVector<> Get_react_torque() { return VNULL; }
    // (Note, functions above might fit better in a specialized subclass, but here for easier GUI interface)

    /// Tells if this link requires that the connected ChBody objects
    /// must be waken if they are sleeping. By default =true, i.e. always keep awaken, but
    /// child classes might return false for optimizing sleeping, in case no time-dependant.
    virtual bool IsRequiringWaking() { return true; }

    //
    // SERIALIZATION
    //

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;
};

CH_CLASS_VERSION(ChLinkBase,0)

}  // end namespace

#endif
