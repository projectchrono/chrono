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

#ifndef CHLINKMOTORROTATIONANGLE_H
#define CHLINKMOTORROTATIONANGLE_H

#include "chrono/physics/ChLinkMotorRotation.h"

namespace chrono {

/// A motor that enforces the rotation angle between two frames on two bodies, using a rheonomic constraint.
/// The angle of frame A rotating on Z axis of frame B, is imposed via an exact function of time f(t) and an optional
/// angle offset: r(t) = f(t) + offset.
/// Note: no compliance is allowed, so if the actuator hits a fixed, rigid obstacle it reaches a pathological
/// situation were solver results are incorrect. As such, this models a servo drive with "infinitely stiff" control.
class ChApi ChLinkMotorRotationAngle : public ChLinkMotorRotation {
  public:
    ChLinkMotorRotationAngle();
    ChLinkMotorRotationAngle(const ChLinkMotorRotationAngle& other);
    virtual ~ChLinkMotorRotationAngle();

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkMotorRotationAngle* Clone() const override { return new ChLinkMotorRotationAngle(*this); }

    /// Set the rotation angle function of time (default: zero constant function).
    /// This function should be C0 continuous and, to prevent acceleration spikes, ideally C1 continuous.
    void SetAngleFunction(const std::shared_ptr<ChFunction> function) { SetMotorFunction(function); }

    /// Get the rotation angle function f(t).
    std::shared_ptr<ChFunction> GetAngleFunction() const { return GetMotorFunction(); }

    /// Set the initial angle offset, in [rad] (default: 0).
    /// Rotation on Z of the two axes will be r(t) = f(t) + offset.
    void SetAngleOffset(double mo) { rot_offset = mo; }

    /// Get initial angle offset, in [rad].
    double GetAngleOffset() { return rot_offset; }

    /// Get the current actuator reaction torque.
    virtual double GetMotorTorque() const override { return -this->react_torque.z(); }

    /// Add the current stiffness K matrix in encapsulated ChKRMBlock item(s), if any.
    /// The K matrix is loaded with scaling value Kfactor.
    virtual void LoadKRMMatrices(double Kfactor, double Rfactor, double Mfactor) override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  private:
    double rot_offset;

    virtual void Update(double time, UpdateFlags update_flags) override;
    virtual void IntLoadConstraint_Ct(const unsigned int off, ChVectorDynamic<>& Qc, const double c) override;
    virtual void ConstraintsBiLoad_Ct(double factor = 1) override;
};

CH_CLASS_VERSION(ChLinkMotorRotationAngle, 0)

}  // end namespace chrono

#endif
