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
  public:
    ChLink() : m_body1(NULL), m_body2(NULL), react_force(VNULL), react_torque(VNULL) {}
    ChLink(const ChLink& other);
    virtual ~ChLink() {}

    /// Get the number of scalar variables affected by constraints in this link
    virtual unsigned int GetNumAffectedCoords() override { return 12; }

    /// Get the constrained body 1.
    ChBodyFrame* GetBody1() const { return m_body1; }

    /// Get the constrained body 2.
    ChBodyFrame* GetBody2() const { return m_body2; }

    /// Get the link frame 1, relative to body 1.
    virtual ChFramed GetFrame1Rel() const = 0;

    /// Get the link frame 2, relative to body 2.
    virtual ChFramed GetFrame2Rel() const = 0;

    /// Get reaction force, expressed on link frame 2.
    virtual ChVector3d GetReactForce2() const { return react_force; }

    /// Get reaction torque, expressed on link frame 2.
    virtual ChVector3d GetReactTorque2() const { return react_torque; }

    /// Get reaction force, expressed on link frame 1.
    ChVector3d GetReactForce1() const;

    /// Get reaction torque, expressed on link frame 1.
    ChVector3d GetReactTorque1() const;

    /// Get the link frame 2, relative to the absolute frame.
    ChFramed GetFrame2Abs() const { return GetFrame2Rel() >> *m_body2; }

    /// Get the link frame 1, relative to the absolute frame.
    ChFramed GetFrame1Abs() const { return GetFrame1Rel() >> *m_body1; }

    /// Get reaction force, expressed on body 2.
    ChVector3d GetReactForceBody2() const;

    /// Get reaction torque, expressed on body 2 frame.
    ChVector3d GetReactTorqueBody2() const;

    /// Get reaction force, expressed on body 1.
    ChVector3d GetReactForceBody1() const;

    /// Get reaction torque, expressed on body 1 frame.
    ChVector3d GetReactTorqueBody1() const;

    // UPDATING FUNCTIONS

    /// Given new time, current body state, update time-dependent quantities in link state,
    /// for example motion laws, moving markers, etc.
    /// Default: do nothing except setting new time.
    virtual void UpdateTime(double mytime);

    /// This function, called by the owner ChSystem at least once per integration step,
    /// updates any auxiliary data of the link (e.g. internal states, forces, Jacobian matrices).
    /// This base version, by default, simply updates the time.
    virtual void Update(double mytime, bool update_assets = true) override;

    /// As above, but with current time
    virtual void Update(bool update_assets = true) override;

    /// Called from a external package (i.e. a plugin, a CAD app.) to report that time has changed.
    virtual void UpdatedExternalTime(double prevtime, double time) {}

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  protected:
    ChBodyFrame* m_body1;     ///< first connected body
    ChBodyFrame* m_body2;     ///< second connected body
    ChVector3d react_force;   ///< store the xyz reactions, expressed in local coordinate system of link;
    ChVector3d react_torque;  ///< store the torque reactions, expressed in local coordinate system of link;

  private:
    /// [INTERNAL USE ONLY] Reaction force on the link frame 2.
    virtual ChVector3d GetReactForce() const override { return GetReactForce2(); }

    /// [INTERNAL USE ONLY] Reaction torque on the link frame 2.
    virtual ChVector3d GetReactTorque() const override { return GetReactTorque2(); }

    /// Get the link coordinate system in absolute reference.
    /// This represents the 'main' reference of the link: reaction forces and reaction torques are expressed in this
    /// coordinate system. Child classes should implement this.
    virtual ChFramed GetFrameAbs() const override { return GetFrame2Abs(); }
};

CH_CLASS_VERSION(ChLink, 0)

}  // end namespace chrono

#endif
