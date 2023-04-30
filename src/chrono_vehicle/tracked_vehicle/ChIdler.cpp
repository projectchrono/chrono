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
// Base class for an idler subsystem.  An idler consists of an idler wheel and
// a tensioner mechanism with different topologies.
//
// The reference frame for a vehicle follows the ISO standard: Z-axis up, X-axis
// pointing forward, and Y-axis towards the left of the vehicle.
//
// =============================================================================

#include "chrono_vehicle/tracked_vehicle/ChIdler.h"
#include "chrono_vehicle/tracked_vehicle/ChTrackAssembly.h"

namespace chrono {
namespace vehicle {

ChIdler::ChIdler(const std::string& name) : ChPart(name), m_track(nullptr) {}

void ChIdler::Initialize(std::shared_ptr<ChChassis> chassis, const ChVector<>& location, ChTrackAssembly* track) {
    m_parent = chassis;
    m_rel_loc = location;
    m_track = track;

    m_idler_wheel->Initialize(chassis, GetCarrierBody(), location, track);

    // Set collision flags for the idler wheel body
    m_idler_wheel->GetBody()->GetCollisionModel()->SetFamily(TrackedCollisionFamily::IDLERS);
    m_idler_wheel->GetBody()->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(TrackedCollisionFamily::WHEELS);

    // Mark as initialized
    m_initialized = true;
}

void ChIdler::SetOutput(bool state) {
    m_output = state;
    m_idler_wheel->SetOutput(state);
}

void ChIdler::ExportComponentList(rapidjson::Document& jsonDocument) const {
    ChPart::ExportComponentList(jsonDocument);

    {
        rapidjson::Document jsonSubDocument(&jsonDocument.GetAllocator());
        jsonSubDocument.SetObject();
        m_idler_wheel->ExportComponentList(jsonSubDocument);
        jsonDocument.AddMember("idler wheel", jsonSubDocument, jsonDocument.GetAllocator());
    }
}

}  // end namespace vehicle
}  // end namespace chrono
