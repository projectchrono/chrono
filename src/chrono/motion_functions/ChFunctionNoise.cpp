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

#include "chrono/motion_functions/ChFunctionNoise.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChFunctionNoise)

ChFunctionNoise::ChFunctionNoise(const ChFunctionNoise& other) {
    amp = other.amp;
    freq = other.freq;
    amp_ratio = other.amp_ratio;
    octaves = other.octaves;
}

double ChFunctionNoise::Get_y(double x) const {
    return ChNoise(x, amp, freq, octaves, amp_ratio);
}

void ChFunctionNoise::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChFunctionNoise>();
    // serialize parent class
    ChFunction::ArchiveOut(marchive);
    // serialize all member data:
    marchive << CHNVP(amp);
    marchive << CHNVP(freq);
    marchive << CHNVP(amp_ratio);
    marchive << CHNVP(octaves);
}

void ChFunctionNoise::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChFunctionNoise>();
    // deserialize parent class
    ChFunction::ArchiveIn(marchive);
    // stream in all member data:
    marchive >> CHNVP(amp);
    marchive >> CHNVP(freq);
    marchive >> CHNVP(amp_ratio);
    marchive >> CHNVP(octaves);
}

}  // end namespace chrono
