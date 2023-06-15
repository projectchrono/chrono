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

#include <algorithm>
#include <cmath>

#include "chrono/assets/ChColor.h"
#include "chrono/core/ChClassFactory.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChColor)

ChVector<float> ChColor::RGB2HSV(const ChColor& rgb) {
    const float& R = rgb.R;
    const float& G = rgb.G;
    const float& B = rgb.B;

    auto M = std::max(R, std::max(G, B));
    auto m = std::min(R, std::min(G, B));
    auto C = M - m;  // chroma

    float S;
    float H;
    float V = M;

    if (V > 0)
        S = C / V;
    else
        return ChVector<float>(-1, 0, 0); // undefined H

    if (C > 0) {
        if (R == V)
            H = (G - B) / C;
        else if (G == V)
            H = 2 + (B - R) / C;
        else
            H = 4 + (R - G) / C;
    } else {
        H = 0;
    }

    H = H * 60.0;  // degrees
    if (H < 0)
        H += 360.0;

    return ChVector<float>(H, S, V);
}

ChColor ChColor::HSV2RGB(const ChVector<float>& hsv) {
    const float& H = hsv[0];
    const float& S = hsv[1];
    const float& V = hsv[2];

    float C = V * S;  // chroma
    float HPrime = std::fmod(H / 60.0f, 6.0f);
    float X = C * (1 - std::fabs(std::fmod(HPrime, 2.0f) - 1));
    float M = V - C;

    ChColor c;

    if (0 <= HPrime && HPrime < 1) {
        c.R = C;
        c.G = X;
        c.B = 0;
    } else if (1 <= HPrime && HPrime < 2) {
        c.R = X;
        c.G = C;
        c.B = 0;
    } else if (2 <= HPrime && HPrime < 3) {
        c.R = 0;
        c.G = C;
        c.B = X;
    } else if (3 <= HPrime && HPrime < 4) {
        c.R = 0;
        c.G = X;
        c.B = C;
    } else if (4 <= HPrime && HPrime < 5) {
        c.R = X;
        c.G = 0;
        c.B = C;
    } else if (5 <= HPrime && HPrime < 6) {
        c.R = C;
        c.G = 0;
        c.B = X;
    } else {
        c.R = 0;
        c.G = 0;
        c.B = 0;
    }

    c.R += M;
    c.G += M;
    c.B += M;

    return c;
}

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

    return c;
}

void ChColor::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChColor>();

    // serialize all member data:
    marchive << CHNVP(R);
    marchive << CHNVP(G);
    marchive << CHNVP(B);
}

void ChColor::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/marchive.VersionRead<ChColor>();

    // stream in all member data:
    marchive >> CHNVP(R);
    marchive >> CHNVP(G);
    marchive >> CHNVP(B);
}

}  // end namespace chrono
