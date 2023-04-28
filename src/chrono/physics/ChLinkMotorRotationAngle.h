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

/// A motor that enforces the rotation angle r(t) between two frames on two bodies, using a rheonomic constraint.
/// The r(t) angle of frame A rotating on Z axis of frame B, is imposed via an exact function of time f(t),
/// and an optional angle offset:
///    r(t) = f(t) + offset
/// Note: no compliance is allowed, so if the actuator hits an undeformable obstacle it hits a pathological
/// situation and the solver result can be unstable/unpredictable.
/// Think at it as a servo drive with "infinitely stiff" control.
/// This type of motor is very easy to use, stable and efficient, and should be used if the 'infinitely stiff'
/// control assumption  is a good approximation of what you simulate (e.g., very good and reactive controllers).
/// By default it is initialized with linear ramp: df/dt= 1.
/// Use SetAngleFunction() to change to other motion functions.

class ChApi ChLinkMotorRotationAngle : public ChLinkMotorRotation {
  public:
    ChLinkMotorRotationAngle();
    ChLinkMotorRotationAngle(const ChLinkMotorRotationAngle& other);
    virtual ~ChLinkMotorRotationAngle();

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkMotorRotationAngle* Clone() const override { return new ChLinkMotorRotationAngle(*this); }

    /// Set the rotation angle function of time a(t).
    /// This function should be C0 continuous and, to prevent acceleration spikes,
    /// it should ideally be C1 continuous.
    void SetAngleFunction(const std::shared_ptr<ChFunction> function) { SetMotorFunction(function); }

    /// Get the rotation angle function f(t).
    std::shared_ptr<ChFunction> GetAngleFunction() const { return GetMotorFunction(); }

    /// Get initial angle offset for f(t)=0, in [rad]. Rotation on Z of the two axes
    /// will be r(t) = f(t) + offset.
    /// By default, offset = 0
    void SetAngleOffset(double mo) { rot_offset = mo; }

    /// Get initial offset for f(t)=0, in [rad]
    double GetAngleOffset() { return rot_offset; }

    /// Get the current actuator reaction torque [Nm]
    virtual double GetMotorTorque() const override { return -this->react_torque.z(); }

    void Update(double mytime, bool update_assets) override;

    //
    // STATE FUNCTIONS
    //
    virtual void IntLoadConstraint_Ct(const unsigned int off, ChVectorDynamic<>& Qc, const double c) override;

    //
    // SOLVER INTERFACE (OLD)
    //
    virtual void ConstraintsBiLoad_Ct(double factor = 1) override;

    /// Add the current stiffness K matrix in encapsulated ChKblock item(s), if any.
    /// The K matrices are load with scaling values Kfactor.
    virtual void KRMmatricesLoad(double Kfactor, double Rfactor, double Mfactor) override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;

  private:
    double rot_offset;
};

CH_CLASS_VERSION(ChLinkMotorRotationAngle, 0)

}  // end namespace chrono

#endif
