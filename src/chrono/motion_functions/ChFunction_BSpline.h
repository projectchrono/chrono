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

#include "chrono/motion_functions/ChFunction_Base.h"
#include "chrono/geometry/ChBasisToolsBspline.h"

namespace chrono {

/// @addtogroup chrono_functions
/// @{

/// B-Spline motion function
class ChApi ChFunction_BSpline : public ChFunction {
public:
    /// Generic univariate B-Spline of order p, approximating given control points.
    /// If knot vector is not provided, initialize it as equally spaced and clamped at both ends.
    ChFunction_BSpline(
        int p,                              ///< order
        const ChVectorDynamic<>& cpoints,   ///< control points
        ChVectorDynamic<>* knots = 0        ///< knot vector
    );

    /// Univariate B-Spline of order p, exactly interpolating given waypoints and derivatives:
    /// internally solve linear problem A * b = c to find proper constrained control points
    /// (eg. useful to pass through given points with assigned velocities, at specific times).
    /// If knot vector is not provided, initialize it as equally spaced and clamped at both ends. 
    ChFunction_BSpline(
        int p,                                  ///< order
        const ChVectorDynamic<>& x_interp,      ///< parameters (eg. times) at which perform interpolation
        const ChVectorDynamic<>& y_dN_interp,   ///< output value to interpolate, Nth derivative: y(x)^(Nth)
        const ChVectorDynamic<int>& der_order,  ///< derivative order of given interpolation output (0: pos, 1: vel, 2: acc, ...)
        ChVectorDynamic<>* knots = 0            ///< knot vector
    );

    ChFunction_BSpline(const ChFunction_BSpline& other);

    ~ChFunction_BSpline() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChFunction_BSpline* Clone() const override { return new ChFunction_BSpline(*this); }

    /// Setup internal data of B-Spline.
    virtual void Setup_Data(int p, ChVectorDynamic<> cpoints, ChVectorDynamic<>* knots);

    /// Get type of ChFunction.
    virtual FunctionType Get_Type() const override { return FUNCT_BSPLINE; }

    /// Get B-Spline order.
    int Get_Order() const { return m_p; }

    /// Get B-Spline control points.
    ChVectorDynamic<> Get_Control_Points() const { return m_cpoints; }

    /// Get B-Spline knots.
    ChVectorDynamic<> Get_Knots() const { return m_knots; }

    /// Get B-Spline averaged control points abscissae (Greville abscissae) as in [De Boor, chapt. XI] averaging formula.
    /// Useful, in univariate B-Spline functions, to get csi(x) abscissae associated to control points y(x),
    /// ie. b_i = [cpointx_i, cpointy_i] = [csi(x)_i, y(x)_i]
    ChVectorDynamic<> Get_Control_Points_Abscissae() const;

    /// Get B-Spline internal tool to evaluate basis functions.
    std::shared_ptr<geometry::ChBasisToolsBspline> Get_Basis_Tool() { return m_basis_tool; }

    /// B-Spline: y(x) = = SUM_i Ni,p(x) b_i
    virtual double Get_y(double x) const override;

    /// B-Spline 1st derivative: y(x)' = = SUM_i Ni,p(x)' b_i
    virtual double Get_y_dx(double x) const override;

    /// B-Spline 2nd derivative: y(x)'' = = SUM_i Ni,p(x)'' b_i
    virtual double Get_y_dxdx(double x) const override;

    /// B-Spline 3rd derivative: y(x)''' = = SUM_i Ni,p(x)''' b_i
    virtual double Get_y_dxdxdx(double x) const override;

    /// Recompute B-Spline control points to exactly interpolate given waypoints and derivatives
    /// (eg. satisfy position, velocity, acceleration constraints)
    virtual void Recompute_Constrained(
        int p,                                  ///< order
        const ChVectorDynamic<>& x_interp,      ///< parameters (eg. times) at which perform interpolation
        const ChVectorDynamic<>& y_dN_interp,   ///< output value to interpolate, Nth derivative: y(x)^(Nth)
        const ChVectorDynamic<int> der_order,   ///< derivative order of given interpolation output (0: pos, 1: vel, 2: acc, ...)
        ChVectorDynamic<>* knots = 0            ///< knot vector
    );

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;

private:
    int m_p;                                                        ///< B-Spline order
    ChVectorDynamic<> m_cpoints;                                    ///< B-Spline control points
    ChVectorDynamic<> m_knots;                                      ///< B-Spline knots
    int m_n;                                                        ///< number of knots
    std::shared_ptr<geometry::ChBasisToolsBspline> m_basis_tool;    ///< internal tool to evaluate B-Spline basis functions
};

/// @} chrono_functions

}  // end namespace chrono

#endif
