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

#ifndef CHLINKMOTORLINEARPOSITION_H
#define CHLINKMOTORLINEARPOSITION_H

#include "chrono/physics/ChLinkMotorLinear.h"

namespace chrono {

/// A linear motor that enforces the position z(t) between two frames on two bodies, using a rheonomic constraint.
/// The z(t) position of frame 1 sliding on Z axis of frame 2, is imposed via an exact function of time f(t),
/// and an optional offset:
///    z(t) = f(t) + offset
/// Note: no compliance is allowed, so if the actuator hits an undeformable obstacle it hits a pathological
/// situation and the solver result can be unstable/unpredictable.
/// Think at it as a servo drive with "infinitely stiff" control.
/// This type of motor is very easy to use, stable and efficient, and should be used if the 'infinitely stiff'
/// control assumption is a good approximation of what you simulate (e.g., very good and reactive controllers).
/// By default it is initialized with linear ramp: df/dt= 1.
/// Use SetMotionFunction() to change to other motion functions.
class ChApi ChLinkMotorLinearPosition : public ChLinkMotorLinear {
  public:
    ChLinkMotorLinearPosition();
    ChLinkMotorLinearPosition(const ChLinkMotorLinearPosition& other);
    virtual ~ChLinkMotorLinearPosition();

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkMotorLinearPosition* Clone() const override { return new ChLinkMotorLinearPosition(*this); }

    /// Set the position function of time p(t).
    /// This function should be C0 continuous and, to prevent acceleration spikes,
    /// it should ideally be C1 continuous.
    void SetMotionFunction(const std::shared_ptr<ChFunction> function) { SetMotorFunction(function); }

    /// Get the position function p(t).
    std::shared_ptr<ChFunction> GetMotionFunction() const { return GetMotorFunction(); }

    /// Get initial offset for f(t)=0. Position on Z of the two axes
    /// will be z(t) = f(t) + offset.
    /// By default, offset = 0
    void SetMotionOffset(double mo) { pos_offset = mo; }

    /// Get initial offset for f(t)=0.
    double GetMotionOffset() { return pos_offset; }

    /// Get the current actuator reaction force, as applied to slider
    virtual double GetMotorForce() const override { return -this->react_force.z(); }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  private:
    double pos_offset;

    virtual void Update(double time, bool update_assets) override;
    virtual void IntLoadConstraint_Ct(const unsigned int off, ChVectorDynamic<>& Qc, const double c) override;
    virtual void ConstraintsBiLoad_Ct(double factor = 1) override;
};

CH_CLASS_VERSION(ChLinkMotorLinearPosition, 0)

}  // end namespace chrono

#endif
