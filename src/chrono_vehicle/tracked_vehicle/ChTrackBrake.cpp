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
//
// Base class for a sprocket brake.
//
// =============================================================================

#include "chrono_vehicle/tracked_vehicle/ChTrackBrake.h"

namespace chrono {
namespace vehicle {

ChTrackBrake::ChTrackBrake(const std::string& name) : ChPart(name) {}

void ChTrackBrake::InitializeInertiaProperties() {
    m_mass = 0;
    m_inertia = ChMatrix33<>(0);
    m_com = ChFrame<>();
    m_xform = ChFrame<>();
}

void ChTrackBrake::UpdateInertiaProperties() {}

void ChTrackBrake::Initialize(std::shared_ptr<ChChassis> chassis, std::shared_ptr<ChSprocket> sprocket) {
    // Mark as initialized
    m_initialized = true;
}

}  // end namespace vehicle
}  // end namespace chrono
