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

void ChIdler::Initialize(std::shared_ptr<ChChassis> chassis, const ChVector3d& location, ChTrackAssembly* track) {
    m_parent = chassis;
    m_rel_loc = location;
    m_track = track;
    m_obj_tag = VehicleObjTag::Generate(GetVehicleTag(), VehiclePartTag::IDLER);

    // Call concrete construction function here, to create the carrier body!
    Construct(chassis, location, track);

    // Initialize the idler wheel, then override the tag of its body
    m_idler_wheel->Initialize(chassis, GetCarrierBody(), location, track);
    m_idler_wheel->GetBody()->SetTag(m_obj_tag);

    // Set collision flags for the idler wheel body
    m_idler_wheel->GetBody()->GetCollisionModel()->SetFamily(VehicleCollisionFamily::IDLER_FAMILY);
    m_idler_wheel->GetBody()->GetCollisionModel()->DisallowCollisionsWith(VehicleCollisionFamily::TRACK_WHEEL_FAMILY);

    // Mark as initialized
    ChPart::Initialize();
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

void ChIdler::Output(ChOutput& database) const {
    ChPart::Output(database);

    database.WriteSection(m_idler_wheel->GetName());
    m_idler_wheel->Output(database);
}

void ChIdler::WriteCheckpoint(ChCheckpoint& database) const {
    ChPart::WriteCheckpoint(database);

    m_idler_wheel->WriteCheckpoint(database);
}

void ChIdler::ReadCheckpoint(ChCheckpoint& database) {
    ChPart::ReadCheckpoint(database);

    m_idler_wheel->ReadCheckpoint(database);
}

}  // end namespace vehicle
}  // end namespace chrono
