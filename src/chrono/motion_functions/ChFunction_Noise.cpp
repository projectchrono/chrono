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

#include "chrono/motion_functions/ChFunction_Noise.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChFunction_Noise)

ChFunction_Noise::ChFunction_Noise(const ChFunction_Noise& other) {
    amp = other.amp;
    freq = other.freq;
    amp_ratio = other.amp_ratio;
    octaves = other.octaves;
}

double ChFunction_Noise::Get_y(double x) const {
    return ChNoise(x, amp, freq, octaves, amp_ratio);
}

}  // end namespace chrono
