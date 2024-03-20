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
// Authors: Dario Fusai
// =============================================================================

#ifndef CHFUNCT_BSPLINE_H
#define CHFUNCT_BSPLINE_H

#include "chrono/functions/ChFunctionBase.h"
#include "chrono/geometry/ChBasisToolsBSpline.h"

namespace chrono {

/// @addtogroup chrono_functions
/// @{

/// B-Spline motion function
class ChApi ChFunctionBSpline : public ChFunction {
  public:
    /// Generic univariate B-Spline of order \a p, approximating given control points.
    /// If knot vector is not provided, initialize it as equally spaced and clamped at both ends.
    ChFunctionBSpline(int p,                             ///< order
                      const ChVectorDynamic<>& cpoints,  ///< control points
                      ChVectorDynamic<>* knots = 0       ///< knot vector
    );

    /// Univariate B-Spline of order p, exactly interpolating given waypoints and derivatives:
    /// internally solve linear problem A * b = c to find proper constrained control points
    /// (eg. useful to pass through given points with assigned velocities, at specific times).
    /// If knot vector is not provided, initialize it as equally spaced and clamped at both ends.
    ChFunctionBSpline(
        int p,                                 ///< order
        const ChVectorDynamic<>& x_interp,     ///< parameters (eg. times) at which to perform interpolation
        const ChVectorDynamic<>& y_dN_interp,  ///< output value to interpolate, Nth derivative: y(x)^(Nth)
        const ChVectorDynamic<int>&
            der_order,                ///< derivative order of given interpolation output (0: pos, 1: vel, 2: acc, ...)
        ChVectorDynamic<>* knots = 0  ///< knot vector
    );

    ChFunctionBSpline(const ChFunctionBSpline& other);

    ~ChFunctionBSpline() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChFunctionBSpline* Clone() const override { return new ChFunctionBSpline(*this); }

    /// Setup internal data of B-Spline.
    virtual void Setup(int p, ChVectorDynamic<> cpoints, ChVectorDynamic<>* knots);

    /// Get type of ChFunction.
    virtual Type GetType() const override { return ChFunction::Type::BSPLINE; }

    /// Get B-Spline order.
    int GetOrder() const { return m_p; }

    /// Get B-Spline control points.
    ChVectorDynamic<> GetControlPoints() const { return m_cpoints; }

    /// Get B-Spline averaged control points abscissae (Greville abscissae) as in [De Boor, chapt. XI] averaging
    /// formula. Useful, in univariate B-Spline functions, to get csi(x) abscissae associated to control points y(x),
    /// ie. b_i = [cpointx_i, cpointy_i] = [csi(x)_i, y(x)_i]
    ChVectorDynamic<> GetControlPointsAbscissae() const;

    /// Get B-Spline knots.
    ChVectorDynamic<> GetKnots() const { return m_knots; }

    /// Get B-Spline internal tool to evaluate basis functions.
    std::shared_ptr<ChBasisToolsBSpline> GetBasisTool() { return m_basis_tool; }

    /// B-Spline: y(x) = = SUM_i Ni,p(x) b_i
    virtual double GetVal(double x) const override;

    /// B-Spline 1st derivative: y(x)' = = SUM_i Ni,p(x)' b_i
    virtual double GetDer(double x) const override;

    /// B-Spline 2nd derivative: y(x)'' = = SUM_i Ni,p(x)'' b_i
    virtual double GetDer2(double x) const override;

    /// B-Spline 3rd derivative: y(x)''' = = SUM_i Ni,p(x)''' b_i
    virtual double GetDer3(double x) const override;

    /// Recompute B-Spline control points to exactly interpolate given waypoints and derivatives
    /// (eg. satisfy position, velocity, acceleration constraints).
    virtual void ApplyInterpolationConstraints(
        int p,                                 ///< order
        const ChVectorDynamic<>& x_interp,     ///< parameters (eg. times) at which perform interpolation
        const ChVectorDynamic<>& y_dN_interp,  ///< output value to interpolate, Nth derivative: y(x)^(Nth)
        const ChVectorDynamic<int>
            der_order,                ///< derivative order of given interpolation output (0: pos, 1: vel, 2: acc, ...)
        ChVectorDynamic<>* knots = 0  ///< knot vector
    );

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  private:
    int m_p;                                            ///< B-Spline order
    ChVectorDynamic<> m_cpoints;                        ///< B-Spline control points
    ChVectorDynamic<> m_knots;                          ///< B-Spline knots
    int m_n;                                            ///< number of knots
    std::shared_ptr<ChBasisToolsBSpline> m_basis_tool;  ///< internal tool to evaluate B-Spline basis functions
};

/// @} chrono_functions

}  // end namespace chrono

#endif
