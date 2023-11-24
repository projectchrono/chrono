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

#ifndef CHFUNCTIONPOSITION_H
#define CHFUNCTIONPOSITION_H

#include <memory.h>
#include <cfloat>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <list>

#include "chrono/core/ChApiCE.h"
#include "chrono/core/ChMath.h"
#include "chrono/core/ChVector.h"

namespace chrono {

/// @addtogroup chrono_functions
/// @{

/// Interface base class for scalar->vector functions of the type:
///
///    p= f(s)
///
/// where p is a 3D vector (ex. a position) and s is a scalar (ex. time)
/// Classes inherited from ChFunctionPosition are often
/// used to set time-dependent positions, for example to set
/// the imposed trajectory of a rigid body in space.
/// Inherited classes must override at least the Get_p() method,
/// in order to represent more complex functions.

class ChApi ChFunctionPosition {
  public:
    ChFunctionPosition() {}
    ChFunctionPosition(const ChFunctionPosition& other) {}
    virtual ~ChFunctionPosition() {}

    /// "Virtual" copy constructor.
    virtual ChFunctionPosition* Clone() const = 0;

    // THE MOST IMPORTANT MEMBER FUNCTIONS
    // At least Get_p() should be overridden by derived classes.

    /// Return the p value of the function, at s, as p=f(s).
    virtual ChVector<> Get_p(double s) const = 0;

    /// Return the dp/ds derivative of the function, at s.
    /// Note that inherited classes may also avoid overriding this method,
    /// because this base method already provide a general-purpose numerical differentiation
    /// to get dp/dt only from the Get_p() function. (however, if the analytical derivative
    /// is known, it may better to implement a custom method).
    virtual ChVector<> Get_p_ds(double s) const { return ((Get_p(s + BDF_STEP_LOW) - Get_p(s)) / BDF_STEP_LOW); }

    /// Return the ddp/dsds double derivative of the function, at s.
    /// Note that inherited classes may also avoid overriding this method,
    /// because this base method already provide a general-purpose numerical differentiation
    /// to get ddp/dsds only from the Get_p() function. (however, if the analytical derivative
    /// is known, it may be better to implement a custom method).
    virtual ChVector<> Get_p_dsds(double s) const {
        return ((Get_p_ds(s + BDF_STEP_LOW) - Get_p_ds(s)) / BDF_STEP_LOW);
    };

    /// Return an estimate of the domain of the function argument.
    /// (ex. can be used for automatic zooming in a GUI, or for computing the bounding box)
    virtual void Estimate_s_domain(double& smin, double& smax) const {
        smin = 0.0;
        smax = 1.0;
    }

    /// Return an estimate of the range of the function value. By default it samples the function N times,
    /// but children classes migh implement a more efficient closed form solution.
    /// (ex. can be used for automatic zooming in a GUI)
    virtual void Estimate_boundingbox(ChVector<>& pmin, ChVector<>& pmax) const;

    /// Update could be implemented by children classes, ex. to launch callbacks
    virtual void Update(double t) {}

    /// Method to allow serialization of transient data to archives
    virtual void ArchiveOut(ChArchiveOut& marchive);

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive);
};

/// @} chrono_functions

CH_CLASS_VERSION(ChFunctionPosition, 0)

}  // end namespace chrono

#endif
