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
// Base class for an anti-roll bar subsystem
//
// =============================================================================

#include "chrono_vehicle/wheeled_vehicle/ChAntirollBar.h"

namespace chrono {
namespace vehicle {

ChAntirollBar::ChAntirollBar(const std::string& name) : ChPart(name) {}

void ChAntirollBar::Initialize(std::shared_ptr<ChChassis> chassis,
                               std::shared_ptr<ChSuspension> suspension,
                               const ChVector<>& location) {
    // Mark as initialized
    m_initialized = true;
}

}  // end namespace vehicle
}  // end namespace chrono
