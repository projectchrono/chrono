// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2019 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Asher Elmquist
// =============================================================================
//
// =============================================================================
#ifndef WEATHER_H
#define WEATHER_H


struct Rain {
    float velocity[3];
    float rate;
    int type;
};

struct SunSky {
    float sun_intensity;
    float cloud_cover;
    int cloud_type;
};

struct Temperature {
    float degrees;
};

struct Wind {
    float velocity[3];
};

struct Snow {
    float velocity[3];
    float rate;
    int type;
};

struct Fog {
    float color[3];
    float density;
};

#endif