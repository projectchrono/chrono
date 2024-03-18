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

#include "chrono/core/ChGlobal.h"
#include "chrono/physics/ChController.h"

namespace chrono {

ChControllerPID::ChControllerPID() : P(1), I(0), D(0) {
    P = 1.;
    I = 0.;
    D = 0.;

    Reset();
}

ChControllerPID::ChControllerPID(const ChControllerPID& other) : ChObj(other) {
    P = other.P;
    I = other.I;
    D = other.D;
}

// Use Reset to set accumulator to zero, at beginning. For integrative part.

void ChControllerPID::Reset() {
    m_input_integral = 0.0;
    m_input_dt = 0.0;
    m_input = 0;
    m_output = 0;
    m_last_t = 0;
}

// Compute controller output value
double ChControllerPID::GetOutput(double input, double time) {
    double delta_t = (time - m_last_t);

    // please, no backward calling sequence!
    if (delta_t < 0) {
        Reset();
        return 0.0;
    }

    // please, multiple calls at same time do not perform updates!
    if (delta_t == 0) {
        return m_output;
    }

    // OK......

    // compute derivative of in (simple Bdf differentiation)
    m_input_dt = (input - m_input) / delta_t;

    // compute integration of in  (trapezoidal rule )
    m_input_integral += (input + m_input) * 0.5 * delta_t;

    // Go forward... synchronize time and last input recorder
    m_input = input;
    m_last_t = time;
    SetChTime(time);

    // ===  Compute PID components ================

    double Pcomp = P * m_input;
    double Icomp = I * m_input_integral;
    double Dcomp = D * m_input_dt;

    m_output = Pcomp + Icomp + Dcomp;

    return m_output;
}

}  // end namespace chrono
