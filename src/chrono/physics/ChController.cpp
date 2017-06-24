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

#include "chrono/physics/ChController.h"
#include "chrono/physics/ChGlobal.h"

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
    In_int = 0.0;
    In_dt = 0.0;
    In = 0;
    Out = 0;
    Pcomp = 0;
    Icomp = 0;
    Dcomp = 0;
    last_t = 0;
}

// Compute controller output value
double ChControllerPID::Get_Out(double new_in, double new_t) {
    double mdt = (new_t - last_t);

    // please, no backward calling sequence!
    if (mdt < 0) {
        Reset();
        return 0.0;
    }

    // please, multiple calls at same time do not perform updates!
    if (mdt == 0) {
        return Out;
    }

    // OK......

    // compute derivative of in (simple Bdf differentiation)
    In_dt = (new_in - In) / mdt;

    // compute integration of in  (trapezoidal rule )
    In_int += (new_in + In) * 0.5 * mdt;

    // Go forward... synchronize time and last input recorder
    In = new_in;
    last_t = new_t;
    SetChTime(new_t);

    // ===  Compute PID components ================

    Pcomp = P * In;
    Icomp = I * In_int;
    Dcomp = D * In_dt;

    Out = Pcomp + Icomp + Dcomp;

    return Out;
}

}  // end namespace chrono
