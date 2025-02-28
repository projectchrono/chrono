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

#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>

#include "chrono/core/ChCubicSpline.h"
#include "chrono/core/ChMatrix.h"
#include "chrono/utils/ChConstants.h"

namespace chrono {

ChCubicSpline::ChCubicSpline(const std::vector<double>& t, const std::vector<double>& y)
    : m_process(true),
      m_left_bc_type(DEFAULT_BC),
      m_right_bc_type(DEFAULT_BC),
      m_left_bc(0),
      m_right_bc(0),
      m_t(t),
      m_y(y) {
    int n = (int)t.size();
    assert(n >= 2);
    assert(n == y.size());
    assert(std::is_sorted(t.begin(), t.end()));
    m_ypp.resize(n);
}

void ChCubicSpline::SetLeftBC(BCType type, double val) {
    m_left_bc_type = type;
    m_left_bc = val;
    m_process = true;
}

void ChCubicSpline::SetRightBC(BCType type, double val) {
    m_right_bc_type = type;
    m_right_bc = val;
    m_process = true;
}

void ChCubicSpline::Process() {
    int n = (int)m_t.size();

    if (n == 2 && m_left_bc_type == DEFAULT_BC && m_right_bc_type == DEFAULT_BC) {
        m_ypp[0] = 0;
        m_ypp[1] = 0;
        return;
    }

    // Set up linear system
    ChMatrixDynamic<double> A(3, n);

    switch (m_left_bc_type) {
        case DEFAULT_BC:
            m_ypp[0] = 0;
            A(1, 0) = 1;
            A(0, 1) = -1;
            break;
        case FIRST_BC:
            m_ypp[0] = (m_y[1] - m_y[0]) / (m_t[1] - m_t[0]) - m_left_bc;
            A(1, 0) = (m_t[1] - m_t[0]) * CH_1_3;
            A(0, 1) = (m_t[1] - m_t[0]) / 6;
            break;
        case SECOND_BC:
            m_ypp[0] = m_left_bc;
            A(1, 0) = 1;
            A(0, 1) = 0;
            break;
    }

    for (int i = 1; i < n - 1; i++) {
        double dtp = m_t[i + 1] - m_t[i];
        double dtm = m_t[i] - m_t[i - 1];
        m_ypp[i] = (m_y[i + 1] - m_y[i]) / dtp - (m_y[i] - m_y[i - 1]) / dtm;
        A(0, i + 1) = dtp / 6;
        A(1, i + 0) = (dtp + dtm) * CH_1_3;
        A(2, i - 1) = dtm / 6;
    }

    switch (m_right_bc_type) {
        case DEFAULT_BC:
            m_ypp[n - 1] = 0;
            A(2, n - 2) = -1;
            A(1, n - 1) = 1;
            break;
        case FIRST_BC:
            m_ypp[n - 1] = m_right_bc - (m_y[n - 1] - m_y[n - 2]) / (m_t[n - 1] - m_t[n - 2]);
            A(2, n - 2) = (m_t[n - 1] - m_t[n - 2]) / 6;
            A(1, n - 1) = (m_t[n - 1] - m_t[n - 2]) * CH_1_3;
            break;
        case SECOND_BC:
            m_ypp[n - 1] = m_left_bc;
            A(2, n - 2) = 0;
            A(1, n - 1) = 1;
            break;
    }

    // Solve linear system (in place)
    for (int i = 1; i < n; i++) {
        double mult = A(2, i - 1) / A(1, i - 1);
        A(1, i) = A(1, i) - mult * A(0, i);
        m_ypp[i] -= mult * m_ypp[i - 1];
    }
    m_ypp[n - 1] = m_ypp[n - 1] / A(1, n - 1);
    for (int i = n - 2; i >= 0; i--) {
        m_ypp[i] = (m_ypp[i] - A(0, i + 1) * m_ypp[i + 1]) / A(1, i);
    }

    // Set processed flag.
    m_process = false;
}

void ChCubicSpline::Evaluate(double t, double& y, double& yp, double& ypp) {
    if (m_process)
        Process();

    auto n = m_t.size();
    assert(t >= m_t[0]);
    assert(t <= m_t[n - 1]);

    // Bracket the given value: m_t[i_left] < t <= m_t[i_left+1]
    int left = 0;
    for (size_t i = 1; i < n; i++) {
        if (t > m_t[i])
            left++;
        else
            break;
    }

    double dt = m_t[left + 1] - m_t[left];
    double tau = (t - m_t[left]) / dt;
    double a = dt * dt * (m_ypp[left + 1] - m_ypp[left]) / 6;
    double b = dt * dt * m_ypp[left] / 2;
    double c = (m_y[left + 1] - m_y[left]) - a - b;
    double d = m_y[left];

    y = a * tau * tau * tau + b * tau * tau + c * tau + d;
    yp = (3 * a * tau * tau + 2 * b * tau + c) / dt;
    ypp = (6 * a * tau + 2 * b) / (dt * dt);
}

}  // end of namespace chrono
