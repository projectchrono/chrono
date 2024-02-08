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
// Authors: Alessandro Tasora
// =============================================================================

#include "chrono/core/ChMathematics.h"

namespace chrono {

// Park-Miller hi-quality random generator

#define IA 16807
#define IM 2147483647
#define AM (1.0 / IM)
#define IQ 127773
#define IR 2836
#define MASK 123459876

static long CH_PAseed = 123;

void ChSetRandomSeed(long newseed) {
    if (CH_PAseed)
        CH_PAseed = newseed;
}

long ChGetRandomSeed() {
    return CH_PAseed;
}

double ChRandom() {
    long k;
    double ans;
    CH_PAseed ^= MASK;
    k = (CH_PAseed) / IQ;
    CH_PAseed = IA * (CH_PAseed - k * IQ) - IR * k;
    if (CH_PAseed < 0)
        CH_PAseed += IM;
    ans = AM * (CH_PAseed);
    CH_PAseed ^= MASK;
    return ans;
}

}  // end namespace chrono
