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
#include "chrono/core/ChMath.h"
#include "chrono/core/ChQuaternion.h"

namespace chrono {

/// @addtogroup chrono_functions
/// @{

/// Interface base class for scalar->quaternion functions of the type:
///
///    q= f(s)
///
/// where q is a unit quaternion (i.e. a rotation in 3D) and s is a scalar (ex. time) 
/// Classes inherited from ChFunctionRotation are often
/// used to set time-dependent rotation, for example to set
/// the imposed alignment of a rigid body in space. 
/// Inherited classes must override at least the Get_q() method, 
/// in order to represent more complex functions.

class ChApi ChFunctionRotation {

  public:
    ChFunctionRotation() {}
    ChFunctionRotation(const ChFunctionRotation& other) {}
    virtual ~ChFunctionRotation() {}

    /// "Virtual" copy constructor.
    virtual ChFunctionRotation* Clone() const = 0;

    // THE MOST IMPORTANT MEMBER FUNCTIONS
    // At least Get_p() should be overridden by derived classes.

    /// Return the rotation as a quaternion, function of s, as q=f(s).
    virtual ChQuaternion<> Get_q(double s) const = 0;

    /// Return the derivative of the rotation function, at s, expressed as angular velocity w in local frame.
    /// Note that inherited classes may also avoid overriding this method,
    /// because this base method already provide a general-purpose numerical differentiation
    /// to get w only from the Get_p() function. (however, if the analytical derivative
    /// is known, it may better to implement a custom method).
	virtual ChVector<> Get_w_loc(double s) const;

    /// Return the derivative of the rotation function, at s, expressed as angular acceleration in local frame.
    /// Note that inherited classes may also avoid overriding this method,
    /// because this base method already provide a general-purpose numerical differentiation
    /// to get angular acceleration only from the Get_q() function. (however, if the analytical derivative
    /// is known, it may be better to implement a custom method).
	virtual ChVector<> Get_a_loc(double s) const;

    /// Return an estimate of the domain of the function argument.
    /// (ex. can be used for automatic zooming in a GUI, or for computing the bounding box)
    virtual void Estimate_s_domain(double& smin, double& smax) const {
        smin = 0.0;
        smax = 1.0;
    }

    /// Update could be implemented by children classes, ex. to launch callbacks
    virtual void Update(const double t) {}
 
    /// Method to allow serialization of transient data to archives
    virtual void ArchiveOut(ChArchiveOut& marchive);

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive);
};

/// @} chrono_functions

CH_CLASS_VERSION(ChFunctionRotation, 0)

}  // end namespace chrono

#endif
