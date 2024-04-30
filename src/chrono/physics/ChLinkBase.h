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

#include "chrono/physics/ChPhysicsItem.h"

namespace chrono {

/// Base class for all types of constraints in the 3D space.
class ChApi ChLinkBase : public ChPhysicsItem {
  public:
    ChLinkBase() : disabled(false), valid(true), broken(false) {}
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

    /// Return true if the link is currently active and thereofre included into the system solver.
    /// This method cumulates the effect of various flags (so a link may be inactive either because disabled, or broken,
    /// or not valid)
    virtual bool IsActive() const override { return (valid && !disabled && !broken); }

    /// Get the number of scalar variables affected by constraints in this link
    virtual unsigned int GetNumAffectedCoords() = 0;

    /// Get the current constraint violations.
    virtual ChVectorDynamic<> GetConstraintViolation() const { return ChVectorDynamic<>(); }

    /// Tells if this link requires that the connected ChBody objects
    /// must be waken if they are sleeping. By default =true, i.e. always keep awaken, but
    /// child classes might return false for optimizing sleeping, in case no time-dependant.
    virtual bool IsRequiringWaking() { return true; }

    /// Get the link frame 1, on the 1st connected object, expressed in the absolute frame.
    virtual ChFramed GetFrame1Abs() const = 0;

    /// Get the link frame 2, on the 2nd connected object, expressed in the absolute frame.
    virtual ChFramed GetFrame2Abs() const = 0;

    /// Get the reaction force and torque on the 1st connected object, expressed in the link frame 1.
    virtual ChWrenchd GetReaction1() const = 0;

    /// Get the reaction force and torque on the 2nd connected object, expressed in the link frame 2.
    virtual ChWrenchd GetReaction2() const = 0;

    /// Get the reference frame (expressed in and relative to the absolute frame) of the visual model.
    /// For a ChLink, the default implementation returns the link coordinate frame 1.
    virtual ChFrame<> GetVisualModelFrame(unsigned int nclone = 0) const override { return GetFrame1Abs(); }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  protected:
    bool disabled;  ///< all constraints of link disabled because of user needs
    bool valid;     ///< link data is valid
    bool broken;    ///< link is broken because of excessive pulling/pushing.
};

CH_CLASS_VERSION(ChLinkBase, 0)

}  // namespace chrono

#endif
