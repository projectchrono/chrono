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
// Authors: Radu Serban, Justin Madsen
// =============================================================================
//
// Base class for all suspension subsystems
//
// =============================================================================

#include <algorithm>

#include "chrono_vehicle/wheeled_vehicle/ChSuspension.h"

namespace chrono {
namespace vehicle {

ChSuspension::ChSuspension(const std::string& name) : ChPart(name), m_steering_index(-1) {}

void ChSuspension::ApplyAxleTorque(VehicleSide side, double torque) {
    m_axle[side]->SetAppliedTorque(torque);
}

void ChSuspension::Synchronize(VehicleSide side, const TireForce& tire_force) {
    m_spindle[side]->Empty_forces_accumulators();
    m_spindle[side]->Accumulate_force(tire_force.force, tire_force.point, false);
    m_spindle[side]->Accumulate_torque(tire_force.moment, false);
}

void ChSuspension::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::NONE)
        return;

    // Add visualization for spindles
    AddVisualizationSpindle(LEFT, getSpindleRadius(), getSpindleWidth());
    AddVisualizationSpindle(RIGHT, getSpindleRadius(), getSpindleWidth());
}

void ChSuspension::RemoveVisualizationAssets() {
    // Make sure we only remove the spindle assets added by ChSuspension::AddVisualizationAssets.
    // This is important for the spindle bodies because a wheel or a tire may add its own assets
    // to the same body.
    {
        auto assets = m_spindle[LEFT]->GetAssets();
        auto it = std::find(assets.begin(), assets.end(), m_spindle_shapes[LEFT]);
        if (it != assets.end())
            assets.erase(it);
    }
    {
        auto assets = m_spindle[RIGHT]->GetAssets();
        auto it = std::find(assets.begin(), assets.end(), m_spindle_shapes[RIGHT]);
        if (it != assets.end())
            assets.erase(it);
    }
}

void ChSuspension::AddVisualizationSpindle(VehicleSide side, double radius, double width) {
    m_spindle_shapes[side] = std::make_shared<ChCylinderShape>();
    m_spindle_shapes[side]->GetCylinderGeometry().p1 = ChVector<>(0, width / 2, 0);
    m_spindle_shapes[side]->GetCylinderGeometry().p2 = ChVector<>(0, -width / 2, 0);
    m_spindle_shapes[side]->GetCylinderGeometry().rad = radius;
    m_spindle[side]->AddAsset(m_spindle_shapes[side]);
}

}  // end namespace vehicle
}  // end namespace chrono
