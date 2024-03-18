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

#include "chrono/utils/ChControllers.h"

namespace chrono {
namespace utils {

ChControllerPID::ChControllerPID() : m_P(1), m_I(0), m_D(0) {
    Reset();
}

ChControllerPID::ChControllerPID(double P, double I, double D) : m_P(P), m_I(I), m_D(D) {
    Reset();
}

ChControllerPID::ChControllerPID(const ChControllerPID& other) : ChObj(other) {
    m_P = other.m_P;
    m_I = other.m_I;
    m_D = other.m_D;
}

void ChControllerPID::Reset() {
    m_input_integral = 0;
    m_input_dt = 0;
    m_input = 0;
    m_output = 0;
    m_last_t = 0;
}

void ChControllerPID::SetGains(double P, double I, double D) {
    m_P = P;
    m_I = I;
    m_D = D;
}

double ChControllerPID::GetOutput(double input, double time) {
    double delta_t = (time - m_last_t);

    // No backward calling sequence
    if (delta_t < 0) {
        Reset();
        return 0;
    }

    // Multiple calls at same time do not perform updates
    if (delta_t == 0) {
        return m_output;
    }

    // Compute input derivative (finite difference approximation)
    m_input_dt = (input - m_input) / delta_t;

    // Compute input integral (trapezoidal rule )
    m_input_integral += (input + m_input) * 0.5 * delta_t;

    // Synchronize time and record last input
    m_input = input;
    m_last_t = time;
    SetChTime(time);

    // Compute output
    m_output = m_P * m_input + m_I * m_input_integral + m_D * m_input_dt;

    return m_output;
}

}  // end namespace utils
}  // end namespace chrono
