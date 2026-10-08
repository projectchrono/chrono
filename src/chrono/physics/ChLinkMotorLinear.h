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

#ifndef CH_LINK_MOTOR_LINEAR_H
#define CH_LINK_MOTOR_LINEAR_H

#include "chrono/physics/ChLinkMotor.h"

namespace chrono {

/// Base class for all linear "motor" constraints between two frames on two bodies.
/// Motors of this type assume that the actuator is directed along Z direction of the master frame. Look for children
/// classes for specialized behaviors, for example e.g. chrono::ChLinkMotorLinearPosition
class ChApi ChLinkMotorLinear : public ChLinkMotor {
  public:
    /// Type of guide constraint.
    enum class GuideConstraint { FREE, PRISMATIC, SPHERICAL };

    ChLinkMotorLinear();
    ChLinkMotorLinear(const ChLinkMotorLinear& other);
    virtual ~ChLinkMotorLinear();

    /// Sets which movements (of frame 1 respect to frame 2) are constrained.
    /// By default, acts as a pure prismatic guide.
    /// Note that the Z direction is the motorized one, and is never affected by this option.
    void SetGuideConstraint(const GuideConstraint constraint);

    /// Sets which movements (of frame 1 respect to frame 2) are constrained.
    /// By default, acts as a pure prismatic guide.
    /// Note that the Z direction is the motorized one, and is never affected by this option.
    void SetGuideConstraint(bool cx, bool cy, bool crx, bool cry, bool crz);

    /// Get the current actuator displacement.
    virtual double GetMotorPos() const { return mpos; }

    /// Get the current actuator velocity.
    virtual double GetMotorPosDt() const { return mpos_dt; }

    /// Get the current actuator acceleration.
    virtual double GetMotorPosDt2() const { return mpos_dtdt; }

    /// Get the current actuator reaction force.
    virtual double GetMotorForce() const = 0;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

    /// Return a string describing the specified motor guide constraint type.
    static std::string GetGuideTypeString(GuideConstraint type);

  protected:
    // aux data for optimization
    double mpos;
    double mpos_dt;
    double mpos_dtdt;

    int m_actuated_idx;  ///< row index of the actuated constraint (Z axis)

    virtual void Update(double time, UpdateFlags update_flags) override;
};

CH_CLASS_VERSION(ChLinkMotorLinear, 0)

}  // end namespace chrono

#endif
