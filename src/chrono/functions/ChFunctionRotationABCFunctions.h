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

#ifndef CHFUNCTIONROTATION_ABCFUNCTIONS_H
#define CHFUNCTIONROTATION_ABCFUNCTIONS_H

#include "chrono/core/ChRotation.h"

#include "chrono/functions/ChFunctionBase.h"
#include "chrono/functions/ChFunctionRotation.h"

namespace chrono {

/// @addtogroup chrono_functions
/// @{

/// A rotation function q=f(s) where q(s) is defined with three ChFunction objects, each per
/// an an angle in an intrinsic triplets of angles (e.g. Euler angles, Cardan angles, etc).
/// By default, rotation is constant zero rotation.
/// By default, uses RotRepresentation::CARDAN_ANGLES_XYZ (sequence: X-Y'-Z'' intrinsic).
class ChApi ChFunctionRotationABCFunctions : public ChFunctionRotation {
  public:
    ChFunctionRotationABCFunctions();
    ChFunctionRotationABCFunctions(const ChFunctionRotationABCFunctions& other);
    virtual ~ChFunctionRotationABCFunctions();

    /// "Virtual" copy constructor.
    virtual ChFunctionRotationABCFunctions* Clone() const override { return new ChFunctionRotationABCFunctions(*this); }

    /// Set the function A=A(s) for the rotation angle (in radians) about the first axis.
    /// Default: constant 0 function.
    void SetFunctionAngleA(std::shared_ptr<ChFunction> angle_function) { angleA = angle_function; }

    /// Get the function A=A(s) for the rotation angle (in radians) about the first axis.
    std::shared_ptr<ChFunction> GetFunctionAngleA() const { return angleA; }

    /// Set the function B(s) for the rotation angle (in radians) about the second axis.
    /// Default: constant 0 function.
    void SetFunctionAngleB(std::shared_ptr<ChFunction> angle_function) { angleB = angle_function; }

    /// Get the function B(s) for the rotation angle (in radians) about the second axis.
    std::shared_ptr<ChFunction> GetFunctionAngleB() const { return angleB; }

    /// Set the function C(s) for the rotation angle (in radians) about the third axis.
    /// Default: constant 0 function.
    void SetFunctionAngleC(std::shared_ptr<ChFunction> angle_function) { angleC = angle_function; }

    /// Get the function C(s) for the rotation angle (in radians) about the third axis.
    std::shared_ptr<ChFunction> GetFunctionAngleC() const { return angleC; }

    /// Set the angle set for rotation representation.
    /// This can be one of the supported Euler angle sets.
    void SetRotationRepresentation(const RotRepresentation rot_rep);

    /// Get the angle set for rotation representation.
    RotRepresentation GetRotationRepresentation() const { return angleset; }

    /// Return the rotation as a quaternion, function of s, as q=f(s).
    virtual ChQuaternion<> GetQuat(double s) const override;

    /// Method to allow serialization of transient data to archives
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  private:
    RotRepresentation angleset;

    std::shared_ptr<ChFunction> angleA;
    std::shared_ptr<ChFunction> angleB;
    std::shared_ptr<ChFunction> angleC;
};

/// @} chrono_functions

CH_CLASS_VERSION(ChFunctionRotationABCFunctions, 0)

}  // end namespace chrono

#endif
