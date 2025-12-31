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

#ifndef CHLINKCLEARANCE_H
#define CHLINKCLEARANCE_H

#include "chrono/physics/ChLinkLock.h"

namespace chrono {

/// A class for the custom fast simulation of revolute joints with clearance.
/// ***OBSOLETE***

class ChApi ChLinkLockClearance : public ChLinkLockLock {
  protected:
    double clearance;      ///< distance offset
    double c_friction;     ///< friction coeff.
    double c_restitution;  ///< restitution coeff.

    double diameter;  ///< radius of shaft (in case of circular shaft)

    ChVector3d contact_F_abs;  ///< [internal]
    ChVector3d contact_V_abs;  ///< [internal]

  public:
    // builders and destroyers
    ChLinkLockClearance();
    ChLinkLockClearance(const ChLinkLockClearance& other);
    ~ChLinkLockClearance() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkLockClearance* Clone() const override { return new ChLinkLockClearance(*this); }

    // Updates marker positions, etc.
    virtual void UpdateTime(double time) override;

    // Updates forces
    virtual void UpdateForces(double time) override;

    // data get/set
    double GetClearance() const { return clearance; }
    void SetClearance(double mset);
    double GetFriction() const { return c_friction; }
    void SetFriction(double mset) { c_friction = mset; }
    double GetRestitution() const { return c_restitution; }
    void SetRestitution(double mset);
    double GetDiameter() const { return diameter; }
    void SetDiameter(double mset) { diameter = mset; }

    double GetEccentricity() const;            // distance between the two shafts
    double GetAxisAngularLocation() const;     // phase of center of shaft, respect to hole
    double GetRotationAngle() const;           // rotation of shafti in hole (relative)
    ChVector3d GetContactPosAbs() const;       // absolute contact point
    ChVector3d GetContactNormalAbs() const;    // absolute normal to contact
    ChVector3d GetContactForceAbs() const;     // absolute force in contact
    double GetContactForceNormal() const;      // normal part of force
    double GetContactForceTangential() const;  // tangent part of force
    double GetContactSpeedTangential() const;  // tangent part of speed

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;
};

CH_CLASS_VERSION(ChLinkLockClearance, 0)

}  // end namespace chrono

#endif
