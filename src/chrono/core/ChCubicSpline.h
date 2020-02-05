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
// Authors: Radu Serban
// =============================================================================
//
// Implementation of 1-D piece-wise cubic spline curves.
//
// =============================================================================

#ifndef CH_CUBIC_SPLINE_H
#define CH_CUBIC_SPLINE_H

#include <vector>
#include <string>

#include "chrono/core/ChApiCE.h"
//#include "chrono/serialization/ChArchive.h"

namespace chrono {

/// Implementation of 1-D piece-wise cubic spline curves.
class ChApi ChCubicSpline {
  public:
    enum BCType {
        DEFAULT_BC,  ///< quadratic function over corresponding interval (first/last)
        FIRST_BC,    ///< imposed first derivative at corresponding endpoint (left/right)
        SECOND_BC    ///< imposed second derivative at corresponding endpoint (left/right)
    };

    /// Construct a cubic spline y = y(t).
    /// It is assumed that the t values are sorted in ascending order.
    ChCubicSpline(const std::vector<double>& t, const std::vector<double>& y);

    /// Set the boundary condition at the left endpoint.
    /// The specified value is ignored if type = DEFAULT_BC.
    void SetLeftBC(BCType type, double val);

    /// Set the boundary condition at the right endpoint.
    /// The specified value is ignored if type = DEFAULT_BC.
    void SetRightBC(BCType type, double val);

    /// Evaluate the cubic spline at the specified value t.
    void Evaluate(double t, double& y, double& yp, double& ypp);

  private:
    void Process();

    bool m_process;

    BCType m_left_bc_type;
    BCType m_right_bc_type;

    double m_left_bc;
    double m_right_bc;

    std::vector<double> m_t;
    std::vector<double> m_y;
    std::vector<double> m_ypp;
};

}  // end namespace chrono

#endif
