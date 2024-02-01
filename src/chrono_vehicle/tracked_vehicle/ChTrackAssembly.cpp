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
// a collection of track suspensions, a collection of rollers, and a collection
// of track shoes.
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

ChTrackAssembly::ChTrackAssembly(const std::string& name, VehicleSide side)
    : ChPart(name),
      m_side(side),
      m_idler_as_cylinder(true),
      m_roller_as_cylinder(true),
      m_roadwheel_as_cylinder(true) {}

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
void ChTrackAssembly::Initialize(std::shared_ptr<ChChassis> chassis, const ChVector<>& location, bool create_shoes) {
    m_parent = chassis;
    m_rel_loc = location;

    // Initialize the sprocket, idler, and brake
    GetSprocket()->Initialize(chassis, location + GetSprocketLocation(), this);
    m_brake->Initialize(chassis, GetSprocket());

    // Initialize the suspension subsystems
    for (size_t i = 0; i < m_suspensions.size(); ++i) {
        m_suspensions[i]->Initialize(chassis, location + GetRoadWhelAssemblyLocation(static_cast<int>(i)), this);
    }

    // Initialize the idler subsystem (after supspension, for idler templates that attach to a suspension arm)
    m_idler->Initialize(chassis, location + GetIdlerLocation(), this);

    // Initialize the roller subsystems (always attached to chassis body)
    for (size_t i = 0; i < m_rollers.size(); ++i) {
        m_rollers[i]->Initialize(chassis, chassis->GetBody(), location + GetRollerLocation(static_cast<int>(i)), this);
    }

    if (!create_shoes) {
        RemoveTrackShoes();
        return;
    }

    // Assemble the track. This positions all track shoes around the sprocket, road wheels, and idler
    // (implemented by derived classes)
    bool ccw = Assemble(chassis->GetBody());

    // Loop over all track shoes
    // - allow them to connect themselves to their neighbor
    // - add terrain loads to chassis container (these are used only when co-simulating with external terrain)
    size_t num_shoes = GetNumTrackShoes();
    for (size_t i = 0; i < num_shoes; ++i) {
        auto this_shoe = GetTrackShoe(i);
        auto next_shoe = (i == num_shoes - 1) ? GetTrackShoe(0) : GetTrackShoe(i + 1);
        this_shoe->Connect(next_shoe, this, chassis.get(), ccw);
        auto fload = chrono_types::make_shared<ChLoadBodyForce>(this_shoe->GetShoeBody(), VNULL, false, VNULL, false);
        auto tload = chrono_types::make_shared<ChLoadBodyTorque>(this_shoe->GetShoeBody(), VNULL, false);
        fload->SetNameString(this_shoe->m_name + "_terrain_force");
        tload->SetNameString(this_shoe->m_name + "_terrain_torque");
        m_shoe_terrain_forces.push_back(fload);
        m_shoe_terrain_torques.push_back(tload);
        chassis->AddTerrainLoad(fload);
        chassis->AddTerrainLoad(tload);
    }

    // Mark as initialized
    m_initialized = true;
}

// -----------------------------------------------------------------------------

void ChTrackAssembly::InitializeInertiaProperties() {
    m_mass = 0;

    GetSprocket()->AddMass(m_mass);

    m_idler->AddMass(m_mass);

    for (auto& suspension : m_suspensions)
        suspension->AddMass(m_mass);

    for (auto& roller : m_rollers)
        roller->AddMass(m_mass);

    for (size_t i = 0; i < GetNumTrackShoes(); ++i)
        GetTrackShoe(i)->AddMass(m_mass);
}

void ChTrackAssembly::UpdateInertiaProperties() {
    m_parent->GetTransform().TransformLocalToParent(ChFrame<>(m_rel_loc, QUNIT), m_xform);

    ChVector<> com(0);
    ChMatrix33<> inertia(0);

    GetSprocket()->AddInertiaProperties(com, inertia);

    m_idler->AddInertiaProperties(com, inertia);

    for (auto& suspension : m_suspensions)
        suspension->AddInertiaProperties(com, inertia);

    for (auto& roller : m_rollers)
        roller->AddInertiaProperties(com, inertia);

    for (size_t i = 0; i < GetNumTrackShoes(); ++i)
        GetTrackShoe(i)->AddInertiaProperties(com, inertia);

    m_com.coord.pos = GetTransform().TransformPointParentToLocal(com / GetMass());
    m_com.coord.rot = GetTransform().GetRot();

    const ChMatrix33<>& A = GetTransform().GetA();
    m_inertia = A.transpose() * (inertia - utils::CompositeInertia::InertiaShiftMatrix(com)) * A;
}

// -----------------------------------------------------------------------------
ChTrackSuspension::ForceTorque ChTrackAssembly::ReportSuspensionForce(size_t id) const {
    return m_suspensions[id]->ReportSuspensionForce();
}

double ChTrackAssembly::ReportTrackLength() const {
    if (GetTrackShoe(0))
        return GetTrackShoe(0)->GetPitch() * GetNumTrackShoes();
    return 0;
}

// -----------------------------------------------------------------------------
void ChTrackAssembly::SetSprocketVisualizationType(VisualizationType vis) {
    GetSprocket()->SetVisualizationType(vis);
}

void ChTrackAssembly::SetIdlerVisualizationType(VisualizationType vis) {
    GetIdler()->SetVisualizationType(vis);
}

void ChTrackAssembly::SetIdlerWheelVisualizationType(VisualizationType vis) {
    GetIdler()->GetIdlerWheel()->SetVisualizationType(vis);
}

void ChTrackAssembly::SetSuspensionVisualizationType(VisualizationType vis) {
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

void ChTrackAssembly::SetWheelCollisionType(bool roadwheel_as_cylinder,
                                            bool idler_as_cylinder,
                                            bool roller_as_cylinder) {
    m_roadwheel_as_cylinder = roadwheel_as_cylinder;
    m_idler_as_cylinder = idler_as_cylinder;
    m_roller_as_cylinder = roller_as_cylinder;
}

// -----------------------------------------------------------------------------
// Update the state of this track assembly at the current time.
// -----------------------------------------------------------------------------
void ChTrackAssembly::Synchronize(double time, double braking) {
    // Zero out applied torque on sprocket axle
    GetSprocket()->m_axle->SetAppliedTorque(0.0);

    // Apply braking input
    m_brake->Synchronize(braking);
}

void ChTrackAssembly::Synchronize(double time, double braking, const TerrainForces& shoe_forces) {
    Synchronize(time, braking);

    // Apply track shoe forces
    for (size_t i = 0; i < GetNumTrackShoes(); ++i) {
        m_shoe_terrain_forces[i]->SetForce(shoe_forces[i].force, false);
        m_shoe_terrain_forces[i]->SetApplicationPoint(shoe_forces[i].point, false);
        m_shoe_terrain_torques[i]->SetTorque(shoe_forces[i].moment, false);
    }
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
    if (GetNumTrackShoes() > 0)
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

    if (GetNumTrackShoes() > 0) {
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

    if (GetNumTrackShoes() > 0) {
        database.WriteSection(GetTrackShoe(0)->GetName());
        GetTrackShoe(0)->Output(database);
    }
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
