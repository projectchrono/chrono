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
// Authors: Alessandro Tasora
// =============================================================================

#ifndef CHLINKMOTORLINEAR_H
#define CHLINKMOTORLINEAR_H

#include "chrono/physics/ChLinkMotor.h"

namespace chrono {

/// Base class for all linear "motor" constraints between
/// two frames on two bodies. Motors of this type assume that
/// the actuator is directed along X direction of the master frame.
/// Look for children classes for specialized behaviors, for example
/// ex. chrono::ChLinkMotorLinearPosition

class ChApi ChLinkMotorLinear : public ChLinkMotor {
  public:
    /// Type of guide constraint
    enum class GuideConstraint { FREE, PRISMATIC, SPHERICAL };

    ChLinkMotorLinear();
    ChLinkMotorLinear(const ChLinkMotorLinear& other);
    virtual ~ChLinkMotorLinear();

    /// "Virtual" copy constructor (covariant return type).
    // virtual ChLinkMotorLinear* Clone() const override { return new ChLinkMotorLinear(*this); }

    /// Sets which movements (of frame 1 respect to frame 2) are constrained.
    /// By default, acts as a pure prismatic guide.
    /// Note that the x direction is the motorized one, and is never affected by
    /// this option.
    void SetGuideConstraint(const GuideConstraint mconstraint);

    /// Sets which movements (of frame 1 respect to frame 2) are constrained.
    /// By default, acts as a pure prismatic guide.
    /// Note that the x direction is the motorized one, and is never affected by
    /// this option.
    void SetGuideConstraint(bool mc_y, bool mc_z, bool mc_rx, bool mc_ry, bool mc_rz);

    /// Get the current actuator displacement [m], including error etc.
    virtual double GetMotorPos() const { return mpos; }
    /// Get the current actuator speed [m/s], including error etc.
    virtual double GetMotorPos_dt() const { return mpos_dt; }
    /// Get the current actuator acceleration [m/s^2], including error etc.
    virtual double GetMotorPos_dtdt() const { return mpos_dtdt; }
    /// Get the current actuator reaction force [N]
    virtual double GetMotorForce() const = 0;

    void Update(double mytime, bool update_assets) override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;

  protected:
    // aux data for optimization
    double mpos;
    double mpos_dt;
    double mpos_dtdt;
};

CH_CLASS_VERSION(ChLinkMotorLinear, 0)

}  // end namespace chrono

#endif
