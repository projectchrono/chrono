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

#include "chrono/motion_functions/ChFunction_Sine.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChFunction_Sine)

ChFunction_Sine::ChFunction_Sine(const ChFunction_Sine& other) {
    amp = other.amp;
    phase = other.phase;
    freq = other.freq;
    w = other.w;
}

double ChFunction_Sine::Get_y(double x) const {
    return amp * (sin(phase + w * x));
}

double ChFunction_Sine::Get_y_dx(double x) const {
    return amp * w * (cos(phase + w * x));
}

double ChFunction_Sine::Get_y_dxdx(double x) const {
    return amp * -w * w * (sin(phase + w * x));
}

void ChFunction_Sine::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChFunction_Sine>();
    // serialize parent class
    ChFunction::ArchiveOut(marchive);
    // serialize all member data:
    marchive << CHNVP(amp);
    marchive << CHNVP(phase);
    marchive << CHNVP(freq);
}

void ChFunction_Sine::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChFunction_Sine>();
    // deserialize parent class
    ChFunction::ArchiveIn(marchive);
    // stream in all member data:
    marchive >> CHNVP(amp);
    marchive >> CHNVP(phase);
    marchive >> CHNVP(freq);
}

}  // end namespace chrono
