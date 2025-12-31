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
/// ChLink objects can constrain the motion in the 3D space of ChBodyFrame objects, most often rigid bodies
/// (ChBody and derived).
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

    /// Get the link frame 1, on body 1, expressed in the absolute frame.
    virtual ChFramed GetFrame1Abs() const override;

    /// Get the link frame 2, on body 2, expressed in the absolute frame.
    virtual ChFramed GetFrame2Abs() const override;

    // The default ChLink implementation assumes that react_force and react_torque represent the reaction wrench on the
    // 2nd body, expressed in the link frame 2. A derived class may interpret react_force and react_torque differently,
    // in which case it must override GetReaction1() and GetReaction2().

    /// Get the reaction force and torque on the 1st body, expressed in the link frame 1.
    virtual ChWrenchd GetReaction1() const override;

    /// Get the reaction force and torque on the 2nd body object, expressed in the link frame 2.
    virtual ChWrenchd GetReaction2() const override;

    // UPDATING FUNCTIONS

    /// This function, called by the owner ChSystem at least once per integration step,
    /// updates any auxiliary data of the link (e.g. internal states, forces, Jacobian matrices).
    /// This base version, by default, simply updates the time.
    virtual void Update(double time, bool update_assets) override;

    /// Called from a external package (i.e. a plugin, a CAD app.) to report that time has changed.
    virtual void UpdatedExternalTime(double prevtime, double time) {}

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  protected:
    ChBodyFrame* m_body1;     ///< first connected body
    ChBodyFrame* m_body2;     ///< second connected body
    ChVector3d react_force;   ///< xyz reactions, expressed in local coordinate system of link;
    ChVector3d react_torque;  ///< torque reactions, expressed in local coordinate system of link;
};

CH_CLASS_VERSION(ChLink, 0)

}  // end namespace chrono

#endif
