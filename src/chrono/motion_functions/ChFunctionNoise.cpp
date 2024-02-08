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

#include "chrono/core/ChMathematics.h"

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

// Compute 1D harmonic multi-octave noise
double ChNoise(double x, double amp, double freq, int octaves, double amp_ratio) {
    double ret = 0;
    long oldseed = ChGetRandomSeed();
    double o_freq, o_amp, xA, xB, yA, yB, period;
    int iA, iB;

    o_freq = freq;
    o_amp = amp;

    for (int i = 1; i <= octaves; i++) {
        period = 1.0 / o_freq;
        xA = period * floor(x / period);
        xB = xA + period;
        iA = int(floor(x / period));
        iB = iA + 1;
        ChSetRandomSeed((long)(iA + 12345));
        ChRandom();
        ChRandom();
        ChRandom();  // just to puzzle the seed..
        yA = (ChRandom() - 0.5) * o_amp;
        ChSetRandomSeed((long)(iB + 12345));
        ChRandom();
        ChRandom();
        ChRandom();  // just to puzzle the seed..
        yB = (ChRandom() - 0.5) * o_amp;
        // cubic noise interpolation from (xA,yA) to (xB,yB), with flat extremal derivatives
        ret += yA + (yB - yA) * ((3 * (pow(((x - xA) / (xB - xA)), 2))) - 2 * (pow(((x - xA) / (xB - xA)), 3)));
        // for following octave, reduce amplitude...
        o_amp *= amp_ratio;
        o_freq *= 2.0;
    }
    
    // restore previous seed
    ChSetRandomSeed(oldseed);

    return ret;
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
