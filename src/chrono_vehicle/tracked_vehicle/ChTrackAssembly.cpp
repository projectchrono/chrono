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
// Base class for a track assembly which consists of one sprocket, one idler,
// a collection of road wheel assemblies (suspensions), a collection of rollers,
// and a collection of track shoes.
//
// The reference frame for a vehicle follows the ISO standard: Z-axis up, X-axis
// pointing forward, and Y-axis towards the left of the vehicle.
//
// =============================================================================

#include <cmath>

#include "chrono/core/ChLog.h"

#include "chrono_vehicle/tracked_vehicle/ChTrackAssembly.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Get the complete state for the specified track shoe.
// -----------------------------------------------------------------------------
BodyState ChTrackAssembly::GetTrackShoeState(size_t id) const {
    BodyState state;

    state.pos = GetTrackShoePos(id);
    state.rot = GetTrackShoeRot(id);
    state.lin_vel = GetTrackShoeLinVel(id);
    state.ang_vel = GetTrackShoeAngVel(id);

    return state;
}

// -----------------------------------------------------------------------------
// Get the complete states for all track shoes.
// -----------------------------------------------------------------------------
void ChTrackAssembly::GetTrackShoeStates(BodyStates& states) const {
    size_t num_shoes = GetNumTrackShoes();
    assert(states.size() == num_shoes);

    for (size_t i = 0; i < num_shoes; ++i)
        states[i] = GetTrackShoeState(i);
}

// -----------------------------------------------------------------------------
// Initialize this track assembly subsystem.
// -----------------------------------------------------------------------------
void ChTrackAssembly::Initialize(std::shared_ptr<ChBodyAuxRef> chassis, const ChVector<>& location, bool create_shoes) {
    // Initialize the sprocket, idler, and brake
    GetSprocket()->Initialize(chassis, location + GetSprocketLocation(), this);
    m_idler->Initialize(chassis, location + GetIdlerLocation());
    m_brake->Initialize(GetSprocket()->GetRevolute());

    // Initialize the suspension subsystems
    for (size_t i = 0; i < m_suspensions.size(); ++i) {
        m_suspensions[i]->Initialize(chassis, location + GetRoadWhelAssemblyLocation(static_cast<int>(i)));
    }

    // Initialize the roller subsystems
    for (size_t i = 0; i < m_rollers.size(); ++i) {
        m_rollers[i]->Initialize(chassis, location + GetRollerLocation(static_cast<int>(i)));
    }

    if (!create_shoes) {
        RemoveTrackShoes();
        return;
    }

    // Assemble the track. This positions all track shoes around the sprocket,
    // road wheels, and idler. (Implemented by derived classes)
    bool ccw = Assemble(chassis);

    // Loop over all track shoes and allow them to connect themselves to their
    // neighbor.
    size_t num_shoes = GetNumTrackShoes();
    std::shared_ptr<ChTrackShoe> next;
    for (size_t i = 0; i < num_shoes; ++i) {
        if (ccw)
            next = (i == num_shoes - 1) ? GetTrackShoe(0) : GetTrackShoe(i + 1);
        else
            next = (i == 0) ? GetTrackShoe(num_shoes - 1) : GetTrackShoe(i - 1);
        GetTrackShoe(i)->Connect(next);
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChTrackAssembly::SetSprocketVisualizationType(VisualizationType vis) {
    GetSprocket()->SetVisualizationType(vis);
}

void ChTrackAssembly::SetIdlerVisualizationType(VisualizationType vis) {
    GetIdler()->SetVisualizationType(vis);
}

void ChTrackAssembly::SetRoadWheelAssemblyVisualizationType(VisualizationType vis) {
    for (size_t i = 0; i < m_suspensions.size(); ++i) {
        m_suspensions[i]->SetVisualizationType(vis);
    }
}

void ChTrackAssembly::SetRoadWheelVisualizationType(VisualizationType vis) {
    for (size_t i = 0; i < m_suspensions.size(); ++i) {
        m_suspensions[i]->GetRoadWheel()->SetVisualizationType(vis);
    }
}

void ChTrackAssembly::SetRollerVisualizationType(VisualizationType vis) {
    for (size_t i = 0; i < m_rollers.size(); ++i) {
        m_rollers[i]->SetVisualizationType(vis);
    }
}

void ChTrackAssembly::SetTrackShoeVisualizationType(VisualizationType vis) {
    SetVisualizationType(vis);
    for (size_t i = 0; i < GetNumTrackShoes(); ++i) {
        GetTrackShoe(i)->SetVisualizationType(vis);
    }
}

// -----------------------------------------------------------------------------
// Calculate and return the total mass of the track assembly
// -----------------------------------------------------------------------------
double ChTrackAssembly::GetMass() const {
    double mass = GetSprocket()->GetMass() + m_idler->GetMass();
    for (size_t i = 0; i < m_suspensions.size(); i++)
        mass += m_suspensions[i]->GetMass();
    for (size_t i = 0; i < m_rollers.size(); i++)
        mass += m_rollers[i]->GetMass();
    for (size_t i = 0; i < GetNumTrackShoes(); ++i)
        mass += GetTrackShoe(i)->GetMass();

    return mass;
}

// -----------------------------------------------------------------------------
// Update the state of this track assembly at the current time.
// -----------------------------------------------------------------------------
void ChTrackAssembly::Synchronize(double time, double braking, const TerrainForces& shoe_forces) {
    // Apply track shoe forces
    for (size_t i = 0; i < GetNumTrackShoes(); ++i) {
        GetTrackShoe(i)->m_shoe->Empty_forces_accumulators();
        GetTrackShoe(i)->m_shoe->Accumulate_force(shoe_forces[i].force, shoe_forces[i].point, false);
        GetTrackShoe(i)->m_shoe->Accumulate_torque(shoe_forces[i].moment, false);
    }

    // Apply braking input
    m_brake->Synchronize(braking);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChTrackAssembly::SetOutput(bool state) {
    m_output = state;
    GetSprocket()->SetOutput(state);
    m_brake->SetOutput(state);
    m_idler->SetOutput(state);
    for (auto suspension : m_suspensions)
        suspension->SetOutput(state);
    for (auto roller : m_rollers)
        roller->SetOutput(state);
    GetTrackShoe(0)->SetOutput(state);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChTrackAssembly::ExportComponentList(rapidjson::Document& jsonDocument) const {
    ChPart::ExportComponentList(jsonDocument);

    jsonDocument.AddMember("number shoes", static_cast<int>(GetNumTrackShoes()), jsonDocument.GetAllocator());

    {
        rapidjson::Document jsonSubDocument(&jsonDocument.GetAllocator());
        jsonSubDocument.SetObject();
        GetSprocket()->ExportComponentList(jsonSubDocument);
        jsonDocument.AddMember("sprocket", jsonSubDocument, jsonDocument.GetAllocator());
    }

    {
        rapidjson::Document jsonSubDocument(&jsonDocument.GetAllocator());
        jsonSubDocument.SetObject();
        m_brake->ExportComponentList(jsonSubDocument);
        jsonDocument.AddMember("brake", jsonSubDocument, jsonDocument.GetAllocator());
    }

    {
        rapidjson::Document jsonSubDocument(&jsonDocument.GetAllocator());
        jsonSubDocument.SetObject();
        m_idler->ExportComponentList(jsonSubDocument);
        jsonDocument.AddMember("idler", jsonSubDocument, jsonDocument.GetAllocator());
    }

    rapidjson::Value suspArray(rapidjson::kArrayType);
    for (auto suspension : m_suspensions) {
        rapidjson::Document jsonSubDocument(&jsonDocument.GetAllocator());
        jsonSubDocument.SetObject();
        suspension->ExportComponentList(jsonSubDocument);
        suspArray.PushBack(jsonSubDocument, jsonDocument.GetAllocator());
    }
    jsonDocument.AddMember("suspensions", suspArray, jsonDocument.GetAllocator());

    rapidjson::Value rollerArray(rapidjson::kArrayType);
    for (auto roller : m_rollers) {
        rapidjson::Document jsonSubDocument(&jsonDocument.GetAllocator());
        jsonSubDocument.SetObject();
        roller->ExportComponentList(jsonSubDocument);
        rollerArray.PushBack(jsonSubDocument, jsonDocument.GetAllocator());
    }
    jsonDocument.AddMember("rollers", rollerArray, jsonDocument.GetAllocator());

    {
        rapidjson::Document jsonSubDocument(&jsonDocument.GetAllocator());
        jsonSubDocument.SetObject();
        GetTrackShoe(0)->ExportComponentList(jsonSubDocument);
        jsonDocument.AddMember("shoe 0", jsonSubDocument, jsonDocument.GetAllocator());
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChTrackAssembly::Output(ChVehicleOutput& database) const {
    if (!m_output)
        return;

    database.WriteSection(GetSprocket()->GetName());
    GetSprocket()->Output(database);

    database.WriteSection(m_brake->GetName());
    m_brake->Output(database);

    database.WriteSection(m_idler->GetName());
    m_idler->Output(database);

    for (auto suspension : m_suspensions) {
        database.WriteSection(suspension->GetName());
        suspension->Output(database);
        database.WriteSection(suspension->GetRoadWheel()->GetName());
        suspension->GetRoadWheel()->Output(database);
    }

    for (auto roller : m_rollers) {
        database.WriteSection(roller->GetName());
        roller->Output(database);
    }

    database.WriteSection(GetTrackShoe(0)->GetName());
    GetTrackShoe(0)->Output(database);
}

// -----------------------------------------------------------------------------
// Log constraint violations
// -----------------------------------------------------------------------------
void ChTrackAssembly::LogConstraintViolations() {
    GetLog() << "SPROCKET constraint violations\n";
    GetSprocket()->LogConstraintViolations();
    GetLog() << "IDLER constraint violations\n";
    m_idler->LogConstraintViolations();
    for (size_t i = 0; i < m_suspensions.size(); i++) {
        GetLog() << "SUSPENSION #" << i << " constraint violations\n";
        m_suspensions[i]->LogConstraintViolations();
    }
    for (size_t i = 0; i < m_rollers.size(); i++) {
        GetLog() << "ROLLER #" << i << " constraint violations\n";
        m_rollers[i]->LogConstraintViolations();
    }
}

}  // end namespace vehicle
}  // end namespace chrono
