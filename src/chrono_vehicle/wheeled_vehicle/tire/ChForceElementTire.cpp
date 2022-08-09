// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2022 projectchrono.org
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
// Base class for a force element tire model
//
// =============================================================================

#include "chrono_vehicle/wheeled_vehicle/tire/ChForceElementTire.h"

namespace chrono {
namespace vehicle {

ChForceElementTire::ChForceElementTire(const std::string& name) : ChTire(name) {}

void ChForceElementTire::InitializeInertiaProperties() {
    m_mass = GetTireMass();
    m_inertia.setZero();
    m_inertia.diagonal() = GetTireInertia().eigen();
    m_com = ChFrame<>();
}

void ChForceElementTire::UpdateInertiaProperties() {
    auto spindle = m_wheel->GetSpindle();
    m_xform = ChFrame<>(spindle->TransformPointLocalToParent(ChVector<>(0, GetOffset(), 0)), spindle->GetRot());
}

double ChForceElementTire::GetAddedMass() const {
    return GetTireMass();
}

ChVector<> ChForceElementTire::GetAddedInertia() const {
    return GetTireInertia();
}

}  // namespace vehicle
}  // namespace chrono
