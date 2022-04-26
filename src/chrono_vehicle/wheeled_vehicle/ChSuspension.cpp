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
// Authors: Radu Serban, Justin Madsen
// =============================================================================
//
// Base class for all suspension subsystems
//
// =============================================================================

#include <algorithm>

#include "chrono_vehicle/ChWorldFrame.h"
#include "chrono_vehicle/wheeled_vehicle/ChSuspension.h"

namespace chrono {
namespace vehicle {

ChSuspension::ChSuspension(const std::string& name) : ChPart(name) {}

ChSuspension::~ChSuspension() {
    auto sys = m_spindle[0]->GetSystem();
    if (sys) {
        sys->Remove(m_spindle[0]);
        sys->Remove(m_spindle[1]);
        sys->Remove(m_axle[0]);
        sys->Remove(m_axle[1]);
        sys->Remove(m_axle_to_spindle[0]);
        sys->Remove(m_axle_to_spindle[1]);
        sys->Remove(m_revolute[0]);
        sys->Remove(m_revolute[1]);
    }
}

ChQuaternion<> ChSuspension::GetSpindleRot(VehicleSide side) const {
    return m_spindle[side]->GetRot() * ChWorldFrame::Quaternion();
}

void ChSuspension::ApplyAxleTorque(VehicleSide side, double torque) {
    m_axle[side]->SetAppliedTorque(torque);
}

void ChSuspension::Synchronize() {
    m_spindle[LEFT]->Empty_forces_accumulators();
    m_spindle[RIGHT]->Empty_forces_accumulators();
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
    ChPart::RemoveVisualizationAsset(m_spindle[LEFT], m_spindle_shapes[LEFT]);
    ChPart::RemoveVisualizationAsset(m_spindle[RIGHT], m_spindle_shapes[RIGHT]);
}

void ChSuspension::AddVisualizationSpindle(VehicleSide side, double radius, double width) {
    m_spindle_shapes[side] = chrono_types::make_shared<ChCylinderShape>();
    m_spindle_shapes[side]->GetCylinderGeometry().p1 = ChVector<>(0, width / 2, 0);
    m_spindle_shapes[side]->GetCylinderGeometry().p2 = ChVector<>(0, -width / 2, 0);
    m_spindle_shapes[side]->GetCylinderGeometry().rad = radius;
    m_spindle[side]->AddVisualShape(m_spindle_shapes[side]);
}

void ChSuspension::ApplyParkingBrake(bool brake) {
    m_revolute[0]->Lock(brake);
    m_revolute[1]->Lock(brake);
}

}  // end namespace vehicle
}  // end namespace chrono
