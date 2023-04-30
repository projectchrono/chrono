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
// Base class for a sub-chassis system for wheeled vehicles.
//
// =============================================================================

#include "chrono/physics/ChSystem.h"
#include "chrono_vehicle/wheeled_vehicle/ChSubchassis.h"

namespace chrono {
namespace vehicle {

ChSubchassis::ChSubchassis(const std::string& name) : ChPart(name) {}

ChSubchassis::~ChSubchassis() {
    auto sys = m_beam[0]->GetSystem();
    if (sys) {
        sys->Remove(m_beam[0]);
        sys->Remove(m_beam[1]);
    }
}

void ChSubchassis::Initialize(std::shared_ptr<ChChassis> chassis, const ChVector<>& location) {
    // Mark as initialized
    m_initialized = true;
}

}  // end namespace vehicle
}  // end namespace chrono
