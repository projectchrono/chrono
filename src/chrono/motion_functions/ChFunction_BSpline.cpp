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

#include "chrono/motion_functions/ChFunction_BSpline.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChFunction_BSpline)


ChFunction_BSpline::ChFunction_BSpline(
    int p,                              ///< order
    const ChVectorDynamic<>& cpoints,   ///< conrol points
    ChVectorDynamic<>* knots            ///< knot vector
) {
    if (p < 1)
        throw ChException("ChFunction_BSpline::Setup_Data() requires order >= 1.");

    if (cpoints.size() < p + 1)
        throw ChException("ChFunction_BSpline::Setup_Data() requires size(cpoints) >= order + 1.");

    if (knots && (size_t)knots->size() != (cpoints.size() + p) + 1)
        throw ChException("ChFunction_BSpline::Setup_Data() requires size(knots) = size(cpoints) + order + 1.");

    Setup_Data(p, cpoints, knots);
}

ChFunction_BSpline::ChFunction_BSpline(
    int p,                                  ///< order
    const ChVectorDynamic<>& x_interp,      ///< parameters (eg. times) at which perform interpolation
    const ChVectorDynamic<>& y_dN_interp,   ///< output value to interpolate, Nth derivative: y(x)^(Nth)
    const ChVectorDynamic<int>& der_order,  ///< derivative order of given interpolation output (0: pos, 1: vel, 2: acc, ...)
    ChVectorDynamic<>* knots                ///< knot vector
) {
    if (p < 1)
        throw ChException("ChFunction_BSpline::Setup_Data() requires order >= 1.");

    if (x_interp.size() < p + 1)
        throw ChException("ChFunction_BSpline::Setup_Data() requires size(cpoints) >= order + 1.");

    if (knots && (size_t)knots->size() != (x_interp.size() + p) + 1)
        throw ChException("ChFunction_BSpline::Setup_Data() requires size(knots) = size(x_interp) + order + 1.");

    Recompute_Constrained(p, x_interp, y_dN_interp, der_order, knots);
}

ChFunction_BSpline::ChFunction_BSpline(const ChFunction_BSpline& other) {
    m_p = other.m_p;
    m_cpoints = other.m_cpoints;
    m_knots = other.m_knots;
    m_n = other.m_n;
    m_basis_tool = other.m_basis_tool;
}

void ChFunction_BSpline::Setup_Data(int p, ChVectorDynamic<> cpoints, ChVectorDynamic<>* knots) {
    m_p = p; // order
    m_cpoints = cpoints; // control points
    m_n = cpoints.size() + m_p; // number of knots
    m_basis_tool = chrono_types::make_shared<geometry::ChBasisToolsBspline>();

    // If empty knot vector, set to equispaced and clamped at ends
    if (knots)
        m_knots = *knots;
    else {
        m_knots.resize(m_n + 1);
        m_basis_tool->ComputeKnotUniformMultipleEnds(m_knots, m_p);
    }
}

ChVectorDynamic<> ChFunction_BSpline::Get_Control_Points_Abscissae() const {
    int m = m_cpoints.size(); // number of control points
    ChVectorDynamic<> cpoints_x(m);
    cpoints_x.setZero();
    // Averaging knots
    for (int j = 0; j < m; ++j) {
        for (int i = 1; i <= m_p; ++i)
            cpoints_x(j) += m_knots(j + i);
        cpoints_x(j) /= m_p;
    }
    return cpoints_x;
}

double ChFunction_BSpline::Get_y(double x) const {
    double y = 0;
    int spanU = m_basis_tool->FindSpan(m_p, x, m_knots);
    ChVectorDynamic<> N(m_p + 1);
    m_basis_tool->BasisEvaluate(m_p, spanU, x, m_knots, N);
    int uind = spanU - m_p;
    for (int i = 0; i <= m_p; i++)
        y += N(i) * m_cpoints(uind + i);
    return y;
}

double ChFunction_BSpline::Get_y_dx(double x) const {
    double yd = 0;
    int spanU = m_basis_tool->FindSpan(m_p, x, m_knots);
    ChMatrixDynamic<> DN(2, m_p + 1);
    m_basis_tool->BasisEvaluateDeriv(m_p, spanU, x, m_knots, DN);
    int uind = spanU - m_p;
    for (int i = 0; i <= m_p; ++i)
        yd += DN(1, i) * m_cpoints(uind + i);
    return yd;
}

double ChFunction_BSpline::Get_y_dxdx(double x) const {
    double ydd = 0;
    int spanU = m_basis_tool->FindSpan(m_p, x, m_knots);
    ChMatrixDynamic<> DN(3, m_p + 1);
    m_basis_tool->BasisEvaluateDeriv(m_p, spanU, x, m_knots, DN);
    int uind = spanU - m_p;
    for (int i = 0; i <= m_p; ++i)
        ydd += DN(2, i) * m_cpoints(uind + i);
    return ydd;
}

double ChFunction_BSpline::Get_y_dxdxdx(double x) const {
    double yddd = 0;
    int spanU = m_basis_tool->FindSpan(m_p, x, m_knots);
    ChMatrixDynamic<> DN(4, m_p + 1);
    m_basis_tool->BasisEvaluateDeriv(m_p, spanU, x, m_knots, DN);
    int uind = spanU - m_p;
    for (int i = 0; i <= m_p; ++i)
        yddd += DN(3, i) * m_cpoints(uind + i);
    return yddd;
}

void ChFunction_BSpline::Recompute_Constrained(
    int p,                                  ///< order
    const ChVectorDynamic<>& x_interp,      ///< parameters (eg. times) at which perform interpolation
    const ChVectorDynamic<>& y_dN_interp,   ///< output value to interpolate, Nth derivative: y(x)^(Nth)
    const ChVectorDynamic<int> der_order,   ///< derivative order of given interpolation output (0: pos, 1: vel, 2: acc, ...)
    ChVectorDynamic<>* knots                ///< knot vector
) {
    if (x_interp.size() != y_dN_interp.size() || x_interp.size() != der_order.size())
        throw ChException("ChFunction_BSpline::Recompute_Constrained() requires size(x_interp) == size(y_dN_interp) == size(der_order).");

    // Initial update of bspline to new data
    Setup_Data(p, x_interp, knots);

    int Ncpoints = x_interp.size(); // number of total control points

    // Evaluate matrix of basis functions and respective derivatives, e.g.
    // A = [N0,p(t0),     N1,p(t0),     ...,     Nm,p(t0)]
    //     [N0,p(t0)^(1), N1,p(t0)^(1), ..., Nm,p(t0)^(1)]
    //     [N0,p(t0)^(2), N1,p(t0)^(2), ..., Nm,p(t0)^(2)]
    //     [ ...                                         ]
    //     [N0,p(tn)^(2), N1,p(tn)^(2), ..., Nm,p(tn)^(2)]
    //     [N0,p(tn)^(1), N1,p(tn)^(1), ..., Nm,p(tn)^(1)]
    //     [N0,p(tn),     N1,p(tn),     ...,     Nm,p(tn)]
    ChMatrixDynamic<> A(Ncpoints, Ncpoints);
    int off = 0; // column offset: start placing data from this index

    for (int i = 0; i < Ncpoints; ++i) {
        double x = x_interp(i); // point at which perform interpolation
        int d = (int)der_order[i]; // constraint derivative order(0: pos, 1: vel, 2: acc, ...)
        int spanU = m_basis_tool->FindSpan(m_p, x, m_knots);

        // If multiple constraints happen at same time tk (eg. q(tk)=c1, v(tk)=c2, a(tk)=c3), do not increment offset
        if (i > 0)
            if (x_interp(i) != x_interp(i - 1))
                ++off;

        ChMatrixDynamic<> DN(d + 1, m_p + 1);
        geometry::ChBasisToolsBspline::BasisEvaluateDeriv(m_p, spanU, x, m_knots, DN);
        ChVectorDynamic<> N = DN.row(d);
        for (int j = 0; j < m_p + 1; ++j)
            A(i, j + off) = N(j);
    }

    // Control points solution & final bspline update
    ChVectorDynamic<> cpoints_constr = A.householderQr().solve(y_dN_interp);
    Setup_Data(m_p, cpoints_constr, knots);
}

void ChFunction_BSpline::ArchiveOut(ChArchiveOut& marchive) {
    // serialize parent class
    ChFunction::ArchiveOut(marchive);
    // serialize all member data:
    marchive << CHNVP(m_p);
    marchive << CHNVP(m_cpoints);
    marchive << CHNVP(m_knots);
    marchive << CHNVP(m_n);
}

void ChFunction_BSpline::ArchiveIn(ChArchiveIn& marchive) {
    // deserialize parent class
    ChFunction::ArchiveIn(marchive);
    // stream in all member data:
    marchive >> CHNVP(m_p);
    marchive >> CHNVP(m_cpoints);
    marchive >> CHNVP(m_knots);
    marchive >> CHNVP(m_n);
}

}  // end namespace chrono
