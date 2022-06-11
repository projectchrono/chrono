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

#include "chrono/assets/ChColor.h"
#include "chrono/core/ChClassFactory.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChColor)

ChColor ChColor::ComputeFalseColor(double v, double vmin, double vmax, bool out_of_range_as_bw) {
    ChColor c = {1.0, 1.0, 1.0};  // default white
    double dv;

    if (out_of_range_as_bw) {
        if (v < vmin)
            return ChColor(0, 0, 0);
        if (v > vmax)
            return ChColor(1, 1, 1);
    }

    if (v < vmin)
        v = vmin;
    if (v > vmax)
        v = vmax;
    dv = vmax - vmin;

    if (v < (vmin + 0.25 * dv)) {
        c.R = 0;
        c.G = (float)(4 * (v - vmin) / dv);
    } else if (v < (vmin + 0.5 * dv)) {
        c.R = 0;
        c.B = (float)(1 + 4 * (vmin + 0.25 * dv - v) / dv);
    } else if (v < (vmin + 0.75 * dv)) {
        c.R = (float)(4 * (v - vmin - 0.5 * dv) / dv);
        c.B = 0;
    } else {
        c.G = (float)(1 + 4 * (vmin + 0.75 * dv - v) / dv);
        c.B = 0;
    }

    return (c);
}

void ChColor::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChColor>();

    // serialize all member data:
    marchive << CHNVP(R);
    marchive << CHNVP(G);
    marchive << CHNVP(B);
}

void ChColor::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    /*int version =*/marchive.VersionRead<ChColor>();

    // stream in all member data:
    marchive >> CHNVP(R);
    marchive >> CHNVP(G);
    marchive >> CHNVP(B);
}

}  // end namespace chrono
