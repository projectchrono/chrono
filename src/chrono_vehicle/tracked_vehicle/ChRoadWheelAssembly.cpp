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
// Base class for a road wheel assembly (suspension).  A road wheel assembly
// contains a road wheel body (connected through a revolute joint to the chassis)
// with different suspension topologies.
//
// The reference frame for a vehicle follows the ISO standard: Z-axis up, X-axis
// pointing forward, and Y-axis towards the left of the vehicle.
//
// =============================================================================

#include "chrono_vehicle/tracked_vehicle/ChRoadWheelAssembly.h"
#include "chrono_vehicle/tracked_vehicle/ChTrackAssembly.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
ChRoadWheelAssembly::ChRoadWheelAssembly(const std::string& name, bool has_shock)
    : ChPart(name), m_has_shock(has_shock), m_track(nullptr) {}

// -----------------------------------------------------------------------------
void ChRoadWheelAssembly::Initialize(std::shared_ptr<ChChassis> chassis,
                                     const ChVector<>& location,
                                     ChTrackAssembly* track) {
    m_parent = chassis;
    m_rel_loc = location;
    m_track = track;

    m_road_wheel->Initialize(chassis->GetBody(), GetCarrierBody(), location, track);
}

// -----------------------------------------------------------------------------
void ChRoadWheelAssembly::SetOutput(bool state) {
    m_output = state;
    m_road_wheel->SetOutput(state);
}

// -----------------------------------------------------------------------------
void ChRoadWheelAssembly::ExportComponentList(rapidjson::Document& jsonDocument) const {
    ChPart::ExportComponentList(jsonDocument);

    jsonDocument.AddMember("has shock", m_has_shock, jsonDocument.GetAllocator());

    {
        rapidjson::Document jsonSubDocument(&jsonDocument.GetAllocator());
        jsonSubDocument.SetObject();
        m_road_wheel->ExportComponentList(jsonSubDocument);
        jsonDocument.AddMember("road wheel", jsonSubDocument, jsonDocument.GetAllocator());
    }
}

}  // end namespace vehicle
}  // end namespace chrono
