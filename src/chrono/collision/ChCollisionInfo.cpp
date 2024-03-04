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

#include "chrono/collision/ChCollisionInfo.h"

namespace chrono {

static double default_eff_radius = 0.1;

ChCollisionInfo::ChCollisionInfo()
    : modelA(nullptr),
      modelB(nullptr),
      shapeA(nullptr),
      shapeB(nullptr),
      vpA(VNULL),
      vpB(VNULL),
      vN(ChVector<>(1, 0, 0)),
      distance(0),
      eff_radius(default_eff_radius),
      reaction_cache(nullptr) {}

ChCollisionInfo::ChCollisionInfo(const ChCollisionInfo& other, const bool swap) {
    if (!swap) {
        modelA = other.modelA;
        modelB = other.modelB;
        shapeA = other.shapeA;
        shapeB = other.shapeB;
        vpA = other.vpA;
        vpB = other.vpB;
        vN = other.vN;
    } else {
        // copy by swapping models
        modelA = other.modelB;
        modelB = other.modelA;
        shapeA = other.shapeB;
        shapeB = other.shapeA;
        vpA = other.vpB;
        vpB = other.vpA;
        vN = -other.vN;
    }
    distance = other.distance;
    eff_radius = other.eff_radius;
    reaction_cache = other.reaction_cache;
}

void ChCollisionInfo::SwapModels() {
    ChCollisionModel* modeltemp;
    modeltemp = modelA;
    modelA = modelB;
    modelB = modeltemp;
    ChVector<> vtemp;
    vtemp = vpA;
    vpA = vpB;
    vpB = vtemp;
    vN = Vmul(vN, -1.0);
}

void ChCollisionInfo::SetDefaultEffectiveCurvatureRadius(double radius) {
    default_eff_radius = radius;
}

double ChCollisionInfo::GetDefaultEffectiveCurvatureRadius() {
    return default_eff_radius;
}

}  // end namespace chrono
