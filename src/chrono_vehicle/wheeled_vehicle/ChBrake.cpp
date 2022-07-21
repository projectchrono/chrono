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
// Base class for a wheel brake.
//
// =============================================================================

#include "chrono_vehicle/wheeled_vehicle/ChBrake.h"

namespace chrono {
namespace vehicle {

ChBrake::ChBrake(const std::string& name) : ChPart(name), m_can_lock(false) {}

void ChBrake::InitializeInertiaProperties() {
    m_mass = 0;
    m_inertia = ChMatrix33<>(0);
    m_com = ChFrame<>();
    m_xform = ChFrame<>();
}

void ChBrake::UpdateInertiaProperties() {}

}  // end namespace vehicle
}  // end namespace chrono
