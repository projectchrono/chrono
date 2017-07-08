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
// Authors: Justin Madsen
// =============================================================================
//
// Simple flat horizontal terrain (infinite x-y extent)
//
// =============================================================================

#include "chrono_vehicle/terrain/FlatTerrain.h"

namespace chrono {
namespace vehicle {

FlatTerrain::FlatTerrain(double height) : m_height(height) {
}

}  // end namespace vehicle
}  // end namespace chrono
