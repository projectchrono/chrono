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
ChTrackShoe::ChTrackShoe(const std::string& name) : ChPart(name), m_index(0) {}

ChTrackShoe::~ChTrackShoe() {
    if (!m_shoe)
        return;

    auto sys = m_shoe->GetSystem();
    if (sys) {
        sys->Remove(m_shoe);
    }
}

void ChTrackShoe::Initialize(std::shared_ptr<ChBodyAuxRef> chassis,
                             const ChVector<>& location,
                             const ChQuaternion<>& rotation) {
    // Mark as initialized
    m_initialized = true;
}

}  // end namespace vehicle
}  // end namespace chrono
