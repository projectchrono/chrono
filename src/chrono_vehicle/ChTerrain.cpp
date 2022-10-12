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
// Authors: Radu Serban
// =============================================================================
//
// Base class for a terrain subsystem.
//
// =============================================================================

#include "chrono_vehicle/ChTerrain.h"
#include "chrono_vehicle/ChWorldFrame.h"

namespace chrono {
namespace vehicle {

ChTerrain::ChTerrain() {}

double ChTerrain::GetHeight(const ChVector<>& loc) const {
    return 0;
}

ChVector<> ChTerrain::GetNormal(const ChVector<>& loc) const {
    return ChWorldFrame::Vertical();
}

float ChTerrain::GetCoefficientFriction(const ChVector<>& loc) const {
    return 0.8f;
}

void ChTerrain::GetProperties(const ChVector<>& loc, double& height, ChVector<>& normal, float& friction) const {
    height = GetHeight(loc);
    normal = GetNormal(loc);
    friction = GetCoefficientFriction(loc);
}

}  // end namespace vehicle
}  // end namespace chrono
