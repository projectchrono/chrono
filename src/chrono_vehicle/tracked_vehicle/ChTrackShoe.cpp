// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Base class for a track shoe.
//
// The reference frame for a vehicle follows the ISO standard: Z-axis up, X-axis
// pointing forward, and Y-axis towards the left of the vehicle.
//
// =============================================================================

#include "chrono_vehicle/tracked_vehicle/ChTrackShoe.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChTrackShoe::ChTrackShoe(const std::string& name)
    : m_name(name), m_index(0), m_friction(0.6f), m_restitution(0.1f), m_young_modulus(2e5f), m_poisson_ratio(0.3f) {
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChTrackShoe::SetContactMaterial(float friction_coefficient,
                                     float restitution_coefficient,
                                     float young_modulus,
                                     float poisson_ratio) {
    m_friction = friction_coefficient;
    m_restitution = restitution_coefficient;
    m_young_modulus = young_modulus;
    m_poisson_ratio = poisson_ratio;
}

}  // end namespace vehicle
}  // end namespace chrono
