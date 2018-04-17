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
// Base class for a tracked vehicle system.
//
// The reference frame for a vehicle follows the ISO standard: Z-axis up, X-axis
// pointing forward, and Y-axis towards the left of the vehicle.
//
// =============================================================================

#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/tracked_vehicle/ChTrackedVehicle.h"

#include "chrono_thirdparty/rapidjson/document.h"
#include "chrono_thirdparty/rapidjson/prettywriter.h"
#include "chrono_thirdparty/rapidjson/stringbuffer.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChTrackedVehicle::ChTrackedVehicle(const std::string& name, ChMaterialSurface::ContactMethod contact_method)
    : ChVehicle(name, contact_method), m_contacts(new ChTrackContactManager) {
}

ChTrackedVehicle::ChTrackedVehicle(const std::string& name, ChSystem* system)
    : ChVehicle(name, system), m_contacts(new ChTrackContactManager) {
}

ChTrackedVehicle::~ChTrackedVehicle() {
    delete m_contacts;
}

// -----------------------------------------------------------------------------
// Initialize this vehicle at the specified global location and orientation.
// This base class implementation only initializes the chassis subsystem.
// Derived classes must extend this function to initialize all other tracked
// vehicle subsystems (the two track assemblies and the driveline).
// -----------------------------------------------------------------------------
void ChTrackedVehicle::Initialize(const ChCoordsys<>& chassisPos, double chassisFwdVel) {
    m_chassis->Initialize(m_system, chassisPos, chassisFwdVel, TrackedCollisionFamily::CHASSIS);

    // Disable contacts between chassis with all other tracked vehicle subsystems,
    // except the track shoes.
    m_chassis->GetBody()->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(TrackedCollisionFamily::IDLERS);
    m_chassis->GetBody()->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(TrackedCollisionFamily::WHEELS);
    m_chassis->GetBody()->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(TrackedCollisionFamily::ROLLERS);
}

// -----------------------------------------------------------------------------
// Update the state of this vehicle at the current time.
// The vehicle system is provided the current driver inputs (throttle between
// 0 and 1, steering between -1 and +1, braking between 0 and 1), the torque
// from the powertrain, and tire forces (expressed in the global reference
// frame).
// -----------------------------------------------------------------------------
void ChTrackedVehicle::Synchronize(double time,
                                   double steering,
                                   double braking,
                                   double powertrain_torque,
                                   const TerrainForces& shoe_forces_left,
                                   const TerrainForces& shoe_forces_right) {
    // Apply powertrain torque to the driveline's input shaft.
    m_driveline->Synchronize(steering, powertrain_torque);

    // Apply contact track shoe forces.
    m_tracks[LEFT]->Synchronize(time, braking, shoe_forces_left);
    m_tracks[RIGHT]->Synchronize(time, braking, shoe_forces_right);

    m_chassis->Synchronize(time);
}

// -----------------------------------------------------------------------------
// Advance the state of this vehicle by the specified time step.
// -----------------------------------------------------------------------------
void ChTrackedVehicle::Advance(double step) {
    // Invoke the base class method to perform the actual work.
    ChVehicle::Advance(step);

    // Process contacts.
    m_contacts->Process(this);
}

// -----------------------------------------------------------------------------
// Set visualization type for the various subsystems
// -----------------------------------------------------------------------------
void ChTrackedVehicle::SetSprocketVisualizationType(VisualizationType vis) {
    m_tracks[0]->SetSprocketVisualizationType(vis);
    m_tracks[1]->SetSprocketVisualizationType(vis);
}

void ChTrackedVehicle::SetIdlerVisualizationType(VisualizationType vis) {
    m_tracks[0]->SetIdlerVisualizationType(vis);
    m_tracks[1]->SetIdlerVisualizationType(vis);
}

void ChTrackedVehicle::SetRoadWheelAssemblyVisualizationType(VisualizationType vis) {
    m_tracks[0]->SetRoadWheelAssemblyVisualizationType(vis);
    m_tracks[1]->SetRoadWheelAssemblyVisualizationType(vis);
}

void ChTrackedVehicle::SetRoadWheelVisualizationType(VisualizationType vis) {
    m_tracks[0]->SetRoadWheelVisualizationType(vis);
    m_tracks[1]->SetRoadWheelVisualizationType(vis);
}

void ChTrackedVehicle::SetRollerVisualizationType(VisualizationType vis) {
    m_tracks[0]->SetRollerVisualizationType(vis);
    m_tracks[1]->SetRollerVisualizationType(vis);
}

void ChTrackedVehicle::SetTrackShoeVisualizationType(VisualizationType vis) {
    m_tracks[0]->SetTrackShoeVisualizationType(vis);
    m_tracks[1]->SetTrackShoeVisualizationType(vis);
}

// -----------------------------------------------------------------------------
// Enable/disable output for the various subsystems
// -----------------------------------------------------------------------------
void ChTrackedVehicle::SetTrackAssemblyOutput(VehicleSide side, bool state) {
    m_tracks[side]->SetOutput(state);
}

// -----------------------------------------------------------------------------
// Enable/disable collision for the various subsystems
// -----------------------------------------------------------------------------
void ChTrackedVehicle::SetSprocketCollide(bool state) {
    m_tracks[0]->GetSprocket()->SetCollide(state);
    m_tracks[1]->GetSprocket()->SetCollide(state);
}

void ChTrackedVehicle::SetIdlerCollide(bool state) {
    m_tracks[0]->GetIdler()->SetCollide(state);
    m_tracks[1]->GetIdler()->SetCollide(state);
}

void ChTrackedVehicle::SetRoadWheelCollide(bool state) {
    for (size_t i = 0; i < m_tracks[0]->GetNumRoadWheelAssemblies(); ++i)
        m_tracks[0]->GetRoadWheel(i)->SetCollide(state);
    for (size_t i = 0; i < m_tracks[1]->GetNumRoadWheelAssemblies(); ++i)
        m_tracks[1]->GetRoadWheel(i)->SetCollide(state);
}

void ChTrackedVehicle::SetRollerCollide(bool state) {
    for (size_t i = 0; i < m_tracks[0]->GetNumRollers(); ++i)
        m_tracks[0]->GetRoller(i)->SetCollide(state);
    for (size_t i = 0; i < m_tracks[1]->GetNumRollers(); ++i)
        m_tracks[1]->GetRoller(i)->SetCollide(state);
}

void ChTrackedVehicle::SetTrackShoeCollide(bool state) {
    for (size_t i = 0; i < m_tracks[0]->GetNumTrackShoes(); ++i)
        m_tracks[0]->GetTrackShoe(i)->SetCollide(state);
    for (size_t i = 0; i < m_tracks[1]->GetNumTrackShoes(); ++i)
        m_tracks[1]->GetTrackShoe(i)->SetCollide(state);
}

// -----------------------------------------------------------------------------
// Override collision flags for various subsystems
// -----------------------------------------------------------------------------
void ChTrackedVehicle::SetCollide(int flags) {
    m_chassis->SetCollide((flags & static_cast<int>(TrackedCollisionFlag::CHASSIS)) != 0);

    m_tracks[0]->GetIdler()->SetCollide((flags & static_cast<int>(TrackedCollisionFlag::IDLER_LEFT)) != 0);
    m_tracks[1]->GetIdler()->SetCollide((flags & static_cast<int>(TrackedCollisionFlag::IDLER_RIGHT)) != 0);

    m_tracks[0]->GetSprocket()->SetCollide((flags & static_cast<int>(TrackedCollisionFlag::SPROCKET_LEFT)) != 0);
    m_tracks[1]->GetSprocket()->SetCollide((flags & static_cast<int>(TrackedCollisionFlag::SPROCKET_RIGHT)) != 0);

    bool collide_wheelsL = (flags & static_cast<int>(TrackedCollisionFlag::WHEELS_LEFT)) != 0;
    bool collide_wheelsR = (flags & static_cast<int>(TrackedCollisionFlag::WHEELS_RIGHT)) != 0;
    for (size_t i = 0; i < m_tracks[0]->GetNumRoadWheelAssemblies(); ++i)
        m_tracks[0]->GetRoadWheel(i)->SetCollide(collide_wheelsL);
    for (size_t i = 0; i < m_tracks[1]->GetNumRoadWheelAssemblies(); ++i)
        m_tracks[1]->GetRoadWheel(i)->SetCollide(collide_wheelsR);

    bool collide_rollersL = (flags & static_cast<int>(TrackedCollisionFlag::ROLLERS_LEFT)) != 0;
    bool collide_rollersR = (flags & static_cast<int>(TrackedCollisionFlag::ROLLERS_RIGHT)) != 0;
    for (size_t i = 0; i < m_tracks[0]->GetNumRollers(); ++i)
        m_tracks[0]->GetRoller(i)->SetCollide(collide_rollersL);
    for (size_t i = 0; i < m_tracks[1]->GetNumRollers(); ++i)
        m_tracks[1]->GetRoller(i)->SetCollide(collide_rollersR);

    bool collide_shoesL = (flags & static_cast<int>(TrackedCollisionFlag::SHOES_LEFT)) != 0;
    bool collide_shoesR = (flags & static_cast<int>(TrackedCollisionFlag::SHOES_RIGHT)) != 0;
    for (size_t i = 0; i < m_tracks[0]->GetNumTrackShoes(); ++i)
        m_tracks[0]->GetTrackShoe(i)->SetCollide(collide_shoesL);
    for (size_t i = 0; i < m_tracks[1]->GetNumTrackShoes(); ++i)
        m_tracks[1]->GetTrackShoe(i)->SetCollide(collide_shoesR);
}

// -----------------------------------------------------------------------------
// Enable/disable collision between the chassis and all other vehicle
// subsystems. This only controls collisions between the chassis and the
// track shoes.  All other internal collisions involving the chassis are
// always ignored.
// -----------------------------------------------------------------------------
void ChTrackedVehicle::SetChassisVehicleCollide(bool state) {
    if (state) {
        // Chassis collides with track shoes
        m_chassis->GetBody()->GetCollisionModel()->SetFamilyMaskDoCollisionWithFamily(TrackedCollisionFamily::SHOES);
    } else {
        // Chassis does not collide with track shoes
        m_chassis->GetBody()->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(TrackedCollisionFamily::SHOES);
    }
}

// -----------------------------------------------------------------------------
// Calculate and return the total vehicle mass
// -----------------------------------------------------------------------------
double ChTrackedVehicle::GetVehicleMass() const {
    return m_chassis->GetMass() + m_tracks[0]->GetMass() + m_tracks[1]->GetMass();
}

// -----------------------------------------------------------------------------
// Log constraint violations
// -----------------------------------------------------------------------------
void ChTrackedVehicle::LogConstraintViolations() {
    GetLog().SetNumFormat("%16.4e");

    // Report constraint violations for the track assemblies.
    GetLog() << "\n---- LEFT TRACK ASSEMBLY constraint violations\n\n";
    m_tracks[0]->LogConstraintViolations();
    GetLog() << "\n---- RIGHT TRACK ASSEMBLY constraint violations\n\n";
    m_tracks[1]->LogConstraintViolations();

    GetLog().SetNumFormat("%g");
}

std::string ChTrackedVehicle::ExportComponentList() const {
    rapidjson::Document jsonDocument;
    jsonDocument.SetObject();

    std::string template_name = GetTemplateName();
    jsonDocument.AddMember("name", rapidjson::StringRef(m_name.c_str()), jsonDocument.GetAllocator());
    jsonDocument.AddMember("template", rapidjson::Value(template_name.c_str(), jsonDocument.GetAllocator()).Move(),
                           jsonDocument.GetAllocator());

    {
        rapidjson::Document jsonSubDocument(&jsonDocument.GetAllocator());
        jsonSubDocument.SetObject();
        m_chassis->ExportComponentList(jsonSubDocument);
        jsonDocument.AddMember("chassis", jsonSubDocument, jsonDocument.GetAllocator());
    }

    {
        rapidjson::Document jsonSubDocument(&jsonDocument.GetAllocator());
        jsonSubDocument.SetObject();
        m_tracks[0]->ExportComponentList(jsonSubDocument);
        jsonDocument.AddMember("left track", jsonSubDocument, jsonDocument.GetAllocator());
    }

    {
        rapidjson::Document jsonSubDocument(&jsonDocument.GetAllocator());
        jsonSubDocument.SetObject();
        m_tracks[1]->ExportComponentList(jsonSubDocument);
        jsonDocument.AddMember("right track", jsonSubDocument, jsonDocument.GetAllocator());
    }

    rapidjson::StringBuffer jsonBuffer;
    rapidjson::PrettyWriter<rapidjson::StringBuffer> jsonWriter(jsonBuffer);
    jsonDocument.Accept(jsonWriter);

    return jsonBuffer.GetString();
}

void ChTrackedVehicle::ExportComponentList(const std::string& filename) const {
    std::ofstream of(filename);
    of << ExportComponentList();
    of.close();
}

void ChTrackedVehicle::Output(int frame, ChVehicleOutput& database) const {
    database.WriteTime(frame, m_system->GetChTime());

    if (m_chassis->OutputEnabled()) {
        database.WriteSection(m_chassis->GetName());
        m_chassis->Output(database);
    }
    
    if (m_tracks[LEFT]->OutputEnabled()) {
        m_tracks[LEFT]->Output(database);
    }

    if (m_tracks[RIGHT]->OutputEnabled()) {
        m_tracks[RIGHT]->Output(database);
    }
}

}  // end namespace vehicle
}  // end namespace chrono
