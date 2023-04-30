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
// Base class for all steering subsystems
//
// =============================================================================

#include "chrono_vehicle/wheeled_vehicle/ChSteering.h"

namespace chrono {
namespace vehicle {

ChSteering::ChSteering(const std::string& name) : ChPart(name) {}

ChSteering::~ChSteering() {
    auto sys = m_link->GetSystem();
    if (sys) {
        sys->Remove(m_link);
    }
}

void ChSteering::Initialize(std::shared_ptr<ChChassis> chassis,
                            const ChVector<>& location,
                            const ChQuaternion<>& rotation) {
    // Mark as initialized
    m_initialized = true;
}

}  // end namespace vehicle
}  // end namespace chrono
