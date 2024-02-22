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
#include "chrono/core/ChFrame.h"
#include "chrono/core/ChVector3.h"

namespace chrono {

/// @addtogroup chrono_functions
/// @{

/// Interface base class for scalar->vector functions
///
///    p = f(s)
///
/// where p is a 3D vector (ex. a position) and s is a scalar (ex. time)
/// Inherited classes must override at least the GetVal() method.

class ChApi ChFunctionPosition {
  public:
    ChFunctionPosition() {}
    ChFunctionPosition(const ChFunctionPosition& other) {}
    virtual ~ChFunctionPosition() {}

    /// "Virtual" copy constructor.
    virtual ChFunctionPosition* Clone() const = 0;

    /// Return the value of the function, at \a s.
    virtual ChVector3d GetVal(double s) const = 0;

    /// Return the first derivative of the function.
    /// Default implementation computes a numerical differentiation.
    /// Inherited classes may override this method with a more efficient implementation (e.g. analytical solution).
    virtual ChVector3d GetDer(double s) const;

    /// Return the second derivative of the function.
    /// Default implementation computes a numerical differentiation.
    /// Inherited classes may override this method with a more efficient implementation (e.g. analytical solution).
    virtual ChVector3d GetDer2(double s) const;

    /// Set the perturbation value used for numerical differentiation (default: 1e-7).
    void SetNumericDiffPerturbation(double pert) { m_der_perturbation = pert; }

    /// Get the perturbation value used for numerical differentiation.
    double GetNumericDiffPerturbation() const { return m_der_perturbation; }

    /// Update could be implemented by children classes, ex. to launch callbacks
    virtual void Update(double t) {}

    /// Method to allow serialization of transient data to archives
    virtual void ArchiveOut(ChArchiveOut& archive_out);

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in);

  protected:
    double m_der_perturbation;  ///< perturbation value used for numerical differentiation
};

/// @} chrono_functions

CH_CLASS_VERSION(ChFunctionPosition, 0)

}  // end namespace chrono

#endif
