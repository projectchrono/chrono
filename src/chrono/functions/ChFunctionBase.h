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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#ifndef CHFUNCT_BASE_H
#define CHFUNCT_BASE_H

#include "chrono/core/ChApiCE.h"
#include "chrono/core/ChMatrix.h"
#include "chrono/serialization/ChArchive.h"

namespace chrono {

/// @addtogroup chrono_functions
/// @{

/// Interface base class for scalar functions.
///
/// Base class for all Chrono scalar functions.
/// The GetVal() and Clone() methods must be implemented by derived classes.
class ChApi ChFunction {
  public:
    /// Enumeration of function types.
    enum class Type {
        BSPLINE,
        CONSTANT,
        CONSTACC,
        CONSTJERK,
        CUSTOM,
        CYCLOIDAL,
        DERIVATIVE,
        FILLET3,
        INTEGRAL,
        INTERP,
        LAMBDA,
        MIRROR,
        OPERATOR,
        POLY,
        POLY23,
        POLY345,
        RAMP,
        REPEAT,
        SEQUENCE,
        SINE,
        SINE_STEP
    };

    ChFunction() {}

    ChFunction(const ChFunction& other) {}

    virtual ~ChFunction() {}

    /// "Virtual" copy constructor.
    virtual ChFunction* Clone() const = 0;

    /// Return the unique function type identifier.
    virtual Type GetType() const { return ChFunction::Type::CUSTOM; }

    /// Return the function output for input \a x.
    /// Must be overridden by specialized classes.
    virtual double GetVal(double x) const = 0;

    /// Return the first derivative of the function.
    /// Default implementation computes a numerical differentiation.
    /// Inherited classes may override this method with a more efficient implementation (e.g. analytical solution).
    virtual double GetDer(double x) const;

    /// Return the second derivative of the function.
    /// Default implementation computes a numerical differentiation.
    /// Inherited classes may override this method with a more efficient implementation (e.g. analytical solution).
    virtual double GetDer2(double x) const;

    /// Return the third derivative of the function.
    /// Default implementation computes a numerical differentiation.
    /// Inherited classes may override this method with a more efficient implementation (e.g. analytical solution).
    virtual double GetDer3(double x) const;

    /// Return the Nth derivative of the function (up to 3rd derivative).
    /// Alias for other GetDerX functions.
    virtual double GetDerN(double x, int der_order) const;

    /// Return the weight of the function.
    /// (useful for applications where you need to mix different weighted ChFunctions)
    virtual double GetWeight(double x) const { return 1.0; }

    /// Update the function at the provided value of its argument.
    virtual void Update(double x) {}

    /// Estimate the maximum of the function (or its \a der_order derivative) in the range [\a xmin, \a xmax].
    /// The estimate is based on function evaluation at equally-spaced points.  
    virtual double GetMax(double xmin, double xmax, double sampling_step, int der_order) const;

    /// Estimate the minimum of the function (or its \a der_order derivative) in the range [\a xmin, \a xmax].
    /// The estimate is based on function evaluation at equally-spaced points.  
    virtual double GetMin(double xmin, double xmax, double sampling_step, int der_order) const;

    /// Estimate the mean of the function (or its \a der_order derivative) in the range [\a xmin, \a xmax].
    /// The estimate is based on function evaluation at equally-spaced points.
    virtual double GetMean(double xmin, double xmax, double sampling_step, int der_order) const;

    /// Estimate the squared mean of the function (or its \a der_order derivative) in the range [\a xmin, \a xmax].
    /// The estimate is based on function evaluation at equally-spaced points.
    virtual double GetSquaredMean(double xmin, double xmax, double sampling_step, int der_order) const;

    /// Estimate the integral of the function (or its \a der_order derivative) over the range [\a xmin, \a xmax].
    /// The estimate is based on function evaluation at equally-spaced points.
    virtual double GetIntegral(double xmin, double xmax, double sampling_step, int der_order) const;

    /// Compute the positive acceleration coefficient.
    /// (derived classes should override this).
    virtual double GetPositiveAccelerationCoeff() const { return 0.0; }

    /// Compute the negative acceleration coefficient.
    /// (derived classes should override this).
    virtual double GetNegativeAccelerationCoeff() const { return 0.0; }

    /// Compute the velocity coefficient.
    /// (derived classes must override this).
    virtual double GetVelocityCoefficient() const { return 0.0; }

    /// Store X-Y pairs to an ASCII File.
    /// Values are separated by \a delimiter (default=',').
    /// The function is sampled \a samples times, from \a xmin to \a xmax.
    virtual void OutputToASCIIFile(std::ostream& file, double xmin, double xmax, int samples, char delimiter);

    /// Sample function on given interval [\a xmin, \a xmax], up to \a derN derivative (0 being the function output
    /// itself). Store interval x=[xmin:step:xmax] and function evaluations as columns into matrix.
    virtual ChMatrixDynamic<> SampleUpToDerN(double xmin, double xmax, double step, int derN = 0);

    /// Alias operator of the GetVal function.
    double operator()(double arg) const { return GetVal(arg); }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out);

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in);
};

/// @} chrono_functions

CH_CLASS_VERSION(ChFunction, 0)

}  // end namespace chrono

#endif
