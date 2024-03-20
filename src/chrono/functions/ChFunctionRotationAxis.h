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

#ifndef CHFUNCTIONROTATION_AXIS_H
#define CHFUNCTIONROTATION_AXIS_H

#include "chrono/functions/ChFunctionBase.h"
#include "chrono/functions/ChFunctionRotation.h"

namespace chrono {

/// @addtogroup chrono_functions
/// @{

/// A rotation function q=f(s) where q(s) is defined with axis V and angle alpha, assuming fixed axis of rotation V
/// and a angle of rotation alpha about that axis, expressed with a ChFunction object  alpha=alpha(s).
/// By default, the axis is the Z axis, and rotation is constant zero rotation.

class ChApi ChFunctionRotationAxis : public ChFunctionRotation {
  public:
    ChFunctionRotationAxis();
    ChFunctionRotationAxis(const ChFunctionRotationAxis& other);
    virtual ~ChFunctionRotationAxis();

    /// "Virtual" copy constructor.
    virtual ChFunctionRotationAxis* Clone() const override { return new ChFunctionRotationAxis(*this); }

    /// Set the angle(s) function expressing the rotation about the fixed axis of rotation. Angle assumed in radians.
    void SetFunctionAngle(std::shared_ptr<ChFunction> mx) { this->fangle = mx; }
    /// Get the angle(s) function expressing the rotation about the fixed axis of rotation. Angle assumed in radians.
    std::shared_ptr<ChFunction> GetFunctionAngle() { return this->fangle; }

    /// Set the fixed axis of rotation. It must be unit length. Default Z vector.
    void SetAxis(const ChVector3d& mv) { this->axis = mv; }
    /// Get the fixed axis of rotation.
    ChVector3d GetAxis() { return this->axis; }

    /// Return the rotation as a quaternion, function of s, as q=f(s).
    virtual ChQuaternion<> GetQuat(double s) const override;

    /// Return the derivative of the rotation function, at s, expressed as angular velocity w in local frame.
    virtual ChVector3d GetAngVel(double s) const override;

    /// Return the derivative of the rotation function, at s, expressed as angular acceleration in local frame.
    virtual ChVector3d GetAngAcc(double s) const override;

    /// Method to allow serialization of transient data to archives
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  private:
    std::shared_ptr<ChFunction> fangle;
    ChVector3d axis;
};

/// @} chrono_functions

CH_CLASS_VERSION(ChFunctionRotationAxis, 0)

}  // end namespace chrono

#endif
