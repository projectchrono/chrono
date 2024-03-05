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

#ifndef CHFUNCTIONROTATION_H
#define CHFUNCTIONROTATION_H

#include <memory.h>
#include <cfloat>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <list>

#include "chrono/core/ChApiCE.h"
#include "chrono/core/ChFrame.h"
#include "chrono/core/ChQuaternion.h"

namespace chrono {

/// @addtogroup chrono_functions
/// @{

/// Interface base class for scalar->quaternion functions of the type:
///
///    q= f(s)
///
/// where q is a unit quaternion (i.e. a rotation in 3D) and s is a scalar (ex. time)
/// Inherited classes must override at least the GetQuat() method.

class ChApi ChFunctionRotation {
  public:
    ChFunctionRotation() {}
    ChFunctionRotation(const ChFunctionRotation& other) {}
    virtual ~ChFunctionRotation() {}

    /// "Virtual" copy constructor.
    virtual ChFunctionRotation* Clone() const = 0;

    /// Return the rotation as a quaternion, function of s, as q=f(s).
    virtual ChQuaternion<> GetQuat(double s) const = 0;

    /// Return the angular velocity in local frame.
    /// Default implementation computes a numerical differentiation.
    /// Inherited classes may override this method with a more efficient implementation (e.g. analytical solution).
    virtual ChVector3d GetAngVel(double s) const;

    /// Return the angular acceleration in local frame.
    /// Default implementation computes a numerical differentiation.
    /// Inherited classes may override this method with a more efficient implementation (e.g. analytical solution).
    virtual ChVector3d GetAngAcc(double s) const;

    /// Update could be implemented by children classes, ex. to launch callbacks
    virtual void Update(double t) {}

    /// Method to allow serialization of transient data to archives
    virtual void ArchiveOut(ChArchiveOut& archive_out);

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in);
};

/// @} chrono_functions

CH_CLASS_VERSION(ChFunctionRotation, 0)

}  // end namespace chrono

#endif
