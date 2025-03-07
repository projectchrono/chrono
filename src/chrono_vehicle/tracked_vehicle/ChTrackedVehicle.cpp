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

using std::cout;
using std::endl;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChTrackedVehicle::ChTrackedVehicle(const std::string& name, ChContactMethod contact_method)
    : ChVehicle(name, contact_method) {
    m_contact_manager = chrono_types::make_shared<ChTrackContactManager>();
}

ChTrackedVehicle::ChTrackedVehicle(const std::string& name, ChSystem* system) : ChVehicle(name, system) {
    m_contact_manager = chrono_types::make_shared<ChTrackContactManager>();
}

ChTrackedVehicle::~ChTrackedVehicle() {}

// -----------------------------------------------------------------------------
// Initialize this vehicle at the specified global location and orientation.
// This base class implementation only initializes the main chassis subsystem.
// Derived classes must extend this function to initialize all other tracked
// vehicle subsystems (the two track assemblies and the driveline).
// -----------------------------------------------------------------------------
void ChTrackedVehicle::Initialize(const ChCoordsys<>& chassisPos, double chassisFwdVel) {
    auto chassis_ct_model = m_chassis->GetBody()->GetCollisionModel();
    if (chassis_ct_model) {
        // Disable contacts between chassis with all other tracked vehicle subsystems, except the track shoes.
        m_chassis->GetBody()->GetCollisionModel()->DisallowCollisionsWith(VehicleCollisionFamily::IDLER_FAMILY);
        m_chassis->GetBody()->GetCollisionModel()->DisallowCollisionsWith(VehicleCollisionFamily::TRACK_WHEEL_FAMILY);
        m_chassis->GetBody()->GetCollisionModel()->DisallowCollisionsWith(VehicleCollisionFamily::ROLLER_FAMILY);
    }
    ChVehicle::Initialize(chassisPos, chassisFwdVel);
}

// -----------------------------------------------------------------------------
// Update the state of this vehicle at the current time.
// The vehicle system is provided the current driver inputs (throttle between 0
// and 1, steering between -1 and +1, braking between 0 and 1).
// The first version is used when the track-terrain interaction is handled by
// Chrono, within the same ChSystem. In this case, the track-terrain interaction
// occurs through the internal Chrono collision and contact mechanism.
// The second version is used in a co-simulation framework and provides the
// terrain forces on the track shoes (assumed to be expressed in the global
// reference frame).
// -----------------------------------------------------------------------------
void ChTrackedVehicle::Synchronize(double time, const DriverInputs& driver_inputs) {
    // Let the driveline combine driver inputs if needed
    double braking_left = 0;
    double braking_right = 0;
    if (m_driveline)
        m_driveline->CombineDriverInputs(driver_inputs, braking_left, braking_right);

    // Apply contact track shoe forces and braking.
    // Attention: these calls also zero out the applied torque to the sprocket axles
    // and so must be called before the driveline synchronization.
    m_tracks[LEFT]->Synchronize(time, braking_left);
    m_tracks[RIGHT]->Synchronize(time, braking_right);

    double powertrain_torque = m_powertrain_assembly ? m_powertrain_assembly->GetOutputTorque() : 0;
    double driveline_speed = m_driveline ? m_driveline->GetOutputDriveshaftSpeed() : 0;

    // Set driveshaft speed for the transmission output shaft
    if (m_powertrain_assembly)
        m_powertrain_assembly->Synchronize(time, driver_inputs, driveline_speed);

    // Apply powertrain torque to the driveline's input shaft
    if (m_driveline)
        m_driveline->Synchronize(time, driver_inputs, powertrain_torque);

    // Pass the steering input to any chassis connectors (in case one of them is actuated)
    for (auto& connector : m_chassis_connectors) {
        connector->Synchronize(time, driver_inputs);
    }

    m_chassis->Synchronize(time);
    for (auto& c : m_chassis_rear)
        c->Synchronize(time);

    // If in use, reset the collision manager
    if (m_collision_manager)
        m_collision_manager->Reset();
}

void ChTrackedVehicle::Synchronize(double time,
                                   const DriverInputs& driver_inputs,
                                   const TerrainForces& shoe_forces_left,
                                   const TerrainForces& shoe_forces_right) {
    // Let the driveline combine driver inputs if needed
    double braking_left = 0;
    double braking_right = 0;
    if (m_driveline)
        m_driveline->CombineDriverInputs(driver_inputs, braking_left, braking_right);

    // Apply contact track shoe forces and braking.
    // Attention: these calls also zero out the applied torque to the sprocket axles
    // and so must be called before the driveline synchronization.
    m_tracks[LEFT]->Synchronize(time, braking_left, shoe_forces_left);
    m_tracks[RIGHT]->Synchronize(time, braking_right, shoe_forces_right);

    double powertrain_torque = m_powertrain_assembly ? m_powertrain_assembly->GetOutputTorque() : 0;
    double driveline_speed = m_driveline ? m_driveline->GetOutputDriveshaftSpeed() : 0;

    // Set driveshaft speed for the transmission output shaft
    if (m_powertrain_assembly)
        m_powertrain_assembly->Synchronize(time, driver_inputs, driveline_speed);

    // Apply powertrain torque to the driveline's input shaft
    if (m_driveline)
        m_driveline->Synchronize(time, driver_inputs, powertrain_torque);

    // Pass the steering input to any chassis connectors (in case one of them is actuated)
    for (auto& connector : m_chassis_connectors) {
        connector->Synchronize(time, driver_inputs);
    }

    m_chassis->Synchronize(time);
    for (auto& c : m_chassis_rear)
        c->Synchronize(time);

    // If in use, reset the collision manager
    if (m_collision_manager)
        m_collision_manager->Reset();
}

// -----------------------------------------------------------------------------
// Advance the state of this vehicle by the specified time step.
// -----------------------------------------------------------------------------
void ChTrackedVehicle::Advance(double step) {
    // Advance state of the associated powertrain (if one is attached)
    if (m_powertrain_assembly) {
        m_powertrain_assembly->Advance(step);
    }

    // Advance the state of the two track assemblies
    m_tracks[LEFT]->Advance(step);
    m_tracks[RIGHT]->Advance(step);

    // Invoke base class function to advance state of underlying Chrono system
    ChVehicle::Advance(step);

    // Process contacts
    m_contact_manager->Process(this);
}

void ChTrackedVehicle::LockDifferential(bool lock) {
    if (m_driveline)
        m_driveline->LockDifferential(lock);
}

// Disconnect driveline
void ChTrackedVehicle::DisconnectDriveline() {
    if (m_driveline)
        m_driveline->Disconnect();
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

void ChTrackedVehicle::SetSuspensionVisualizationType(VisualizationType vis) {
    m_tracks[0]->SetSuspensionVisualizationType(vis);
    m_tracks[1]->SetSuspensionVisualizationType(vis);
}

void ChTrackedVehicle::SetIdlerWheelVisualizationType(VisualizationType vis) {
    m_tracks[0]->SetIdlerWheelVisualizationType(vis);
    m_tracks[1]->SetIdlerWheelVisualizationType(vis);
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
    m_tracks[0]->GetSprocket()->EnableCollision(state);
    m_tracks[1]->GetSprocket()->EnableCollision(state);
}

void ChTrackedVehicle::SetIdlerCollide(bool state) {
    m_tracks[0]->GetIdlerWheel()->EnableCollision(state);
    m_tracks[1]->GetIdlerWheel()->EnableCollision(state);
}

void ChTrackedVehicle::SetRoadWheelCollide(bool state) {
    for (size_t i = 0; i < m_tracks[0]->GetNumTrackSuspensions(); ++i)
        m_tracks[0]->GetRoadWheel(i)->EnableCollision(state);
    for (size_t i = 0; i < m_tracks[1]->GetNumTrackSuspensions(); ++i)
        m_tracks[1]->GetRoadWheel(i)->EnableCollision(state);
}

void ChTrackedVehicle::SetRollerCollide(bool state) {
    for (size_t i = 0; i < m_tracks[0]->GetNumRollers(); ++i)
        m_tracks[0]->GetRoller(i)->EnableCollision(state);
    for (size_t i = 0; i < m_tracks[1]->GetNumRollers(); ++i)
        m_tracks[1]->GetRoller(i)->EnableCollision(state);
}

void ChTrackedVehicle::SetTrackShoeCollide(bool state) {
    for (size_t i = 0; i < m_tracks[0]->GetNumTrackShoes(); ++i)
        m_tracks[0]->GetTrackShoe(i)->EnableCollision(state);
    for (size_t i = 0; i < m_tracks[1]->GetNumTrackShoes(); ++i)
        m_tracks[1]->GetTrackShoe(i)->EnableCollision(state);
}

// -----------------------------------------------------------------------------
// Override collision flags for various subsystems
// -----------------------------------------------------------------------------
void ChTrackedVehicle::EnableCollision(int flags) {
    m_chassis->EnableCollision((flags & static_cast<int>(TrackedCollisionFlag::CHASSIS)) != 0);

    for (auto& c : m_chassis_rear)
        c->EnableCollision((flags & static_cast<int>(TrackedCollisionFlag::CHASSIS)) != 0);

    m_tracks[0]->GetIdlerWheel()->EnableCollision((flags & static_cast<int>(TrackedCollisionFlag::IDLER_LEFT)) != 0);
    m_tracks[1]->GetIdlerWheel()->EnableCollision((flags & static_cast<int>(TrackedCollisionFlag::IDLER_RIGHT)) != 0);

    m_tracks[0]->GetSprocket()->EnableCollision((flags & static_cast<int>(TrackedCollisionFlag::SPROCKET_LEFT)) != 0);
    m_tracks[1]->GetSprocket()->EnableCollision((flags & static_cast<int>(TrackedCollisionFlag::SPROCKET_RIGHT)) != 0);

    bool collide_wheelsL = (flags & static_cast<int>(TrackedCollisionFlag::WHEELS_LEFT)) != 0;
    bool collide_wheelsR = (flags & static_cast<int>(TrackedCollisionFlag::WHEELS_RIGHT)) != 0;
    for (size_t i = 0; i < m_tracks[0]->GetNumTrackSuspensions(); ++i)
        m_tracks[0]->GetRoadWheel(i)->EnableCollision(collide_wheelsL);
    for (size_t i = 0; i < m_tracks[1]->GetNumTrackSuspensions(); ++i)
        m_tracks[1]->GetRoadWheel(i)->EnableCollision(collide_wheelsR);

    bool collide_rollersL = (flags & static_cast<int>(TrackedCollisionFlag::ROLLERS_LEFT)) != 0;
    bool collide_rollersR = (flags & static_cast<int>(TrackedCollisionFlag::ROLLERS_RIGHT)) != 0;
    for (size_t i = 0; i < m_tracks[0]->GetNumRollers(); ++i)
        m_tracks[0]->GetRoller(i)->EnableCollision(collide_rollersL);
    for (size_t i = 0; i < m_tracks[1]->GetNumRollers(); ++i)
        m_tracks[1]->GetRoller(i)->EnableCollision(collide_rollersR);

    bool collide_shoesL = (flags & static_cast<int>(TrackedCollisionFlag::SHOES_LEFT)) != 0;
    bool collide_shoesR = (flags & static_cast<int>(TrackedCollisionFlag::SHOES_RIGHT)) != 0;
    for (size_t i = 0; i < m_tracks[0]->GetNumTrackShoes(); ++i)
        m_tracks[0]->GetTrackShoe(i)->EnableCollision(collide_shoesL);
    for (size_t i = 0; i < m_tracks[1]->GetNumTrackShoes(); ++i)
        m_tracks[1]->GetTrackShoe(i)->EnableCollision(collide_shoesR);
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
        m_chassis->GetBody()->GetCollisionModel()->AllowCollisionsWith(VehicleCollisionFamily::SHOE_FAMILY);
        for (auto& c : m_chassis_rear)
            c->GetBody()->GetCollisionModel()->AllowCollisionsWith(VehicleCollisionFamily::SHOE_FAMILY);
    } else {
        // Chassis does not collide with track shoes
        m_chassis->GetBody()->GetCollisionModel()->DisallowCollisionsWith(VehicleCollisionFamily::SHOE_FAMILY);
        for (auto& c : m_chassis_rear)
            c->GetBody()->GetCollisionModel()->DisallowCollisionsWith(VehicleCollisionFamily::SHOE_FAMILY);
    }
}

// -----------------------------------------------------------------------------
// Enable user-defined contact forces between track shoes and idlers and/or
// road-wheels and/or ground.
// -----------------------------------------------------------------------------
void ChTrackedVehicle::EnableCustomContact(std::shared_ptr<ChTrackCustomContact> callback) {
    bool idler_shoe = callback->OverridesIdlerContact();
    bool wheel_shoe = callback->OverridesWheelContact();
    bool ground_shoe = callback->OverridesGroundContact();

    if (!idler_shoe && !wheel_shoe && !ground_shoe)
        return;

    // Use the narrow-phase callback mechanism to intercept all collisions between wheels and track shoes
    m_collision_manager = std::shared_ptr<ChTrackCollisionManager>(new ChTrackCollisionManager(this));
    m_collision_manager->m_idler_shoe = idler_shoe;
    m_collision_manager->m_wheel_shoe = wheel_shoe;
    m_collision_manager->m_ground_shoe = ground_shoe;
    m_system->GetCollisionSystem()->RegisterNarrowphaseCallback(m_collision_manager);

    // Add the provided callback as a load container to the system
    callback->m_collision_manager = m_collision_manager.get();
    m_system->Add(callback);
}

// -----------------------------------------------------------------------------
// Calculate the total vehicle mass
// -----------------------------------------------------------------------------
void ChTrackedVehicle::InitializeInertiaProperties() {
    m_mass = 0;

    m_chassis->AddMass(m_mass);

    for (auto& c : m_chassis_rear)
        c->AddMass(m_mass);

    m_tracks[0]->AddMass(m_mass);
    m_tracks[1]->AddMass(m_mass);
}

// -----------------------------------------------------------------------------
// Calculate current vehicle inertia properties
// -----------------------------------------------------------------------------
void ChTrackedVehicle::UpdateInertiaProperties() {
    // 1. Calculate the vehicle COM location relative to the global reference frame
    // 2. Calculate vehicle inertia relative to global reference frame
    ChVector3d com(0);
    ChMatrix33<> inertia(0);

    m_chassis->AddInertiaProperties(com, inertia);

    for (auto& c : m_chassis_rear)
        c->AddInertiaProperties(com, inertia);

    m_tracks[0]->AddInertiaProperties(com, inertia);
    m_tracks[1]->AddInertiaProperties(com, inertia);

    // 3. Express vehicle COM frame relative to vehicle reference frame
    m_com.SetPos(GetTransform().TransformPointParentToLocal(com / GetMass()));
    m_com.SetRot(GetTransform().GetRot());

    // 4. Express inertia relative to vehicle COM frame
    //    Notes: - vehicle COM frame aligned with vehicle frame
    //           - 'com' still scaled by total mass here
    const ChMatrix33<>& A = GetTransform().GetRotMat();
    m_inertia = A.transpose() * (inertia - utils::CompositeInertia::InertiaShiftMatrix(com)) * A;
}

// -----------------------------------------------------------------------------

void ChTrackedVehicle::LogConstraintViolations() {
    // Report constraint violations for the track assemblies.
    std::cout << "\n---- LEFT TRACK ASSEMBLY constraint violations\n\n";
    m_tracks[0]->LogConstraintViolations();
    std::cout << "\n---- RIGHT TRACK ASSEMBLY constraint violations\n\n";
    m_tracks[1]->LogConstraintViolations();
}

void ChTrackedVehicle::LogSubsystemTypes() {
    cout << "\nSubsystem types\n";

    {
        cout << "Chassis:        " << m_chassis->GetTemplateName() << "\n";

        int body_tag = m_chassis->GetBodyTag();
        auto vehicle_tag = VehicleObjTag::ExtractVehicleTag(body_tag);
        auto part_tag = VehicleObjTag::ExtractPartTag(body_tag);
        cout << "         vehicle tag: " << m_chassis->GetVehicleTag();
        cout << "         body tag:    " << body_tag << " [ " << vehicle_tag << " + " << part_tag << " ]" << endl;
    }

    if (m_powertrain_assembly) {
        cout << "Powertrain:\n";
        cout << "  Engine:       " << GetEngine()->GetTemplateName() << "\n";
        cout << "  Transmission: " << GetTransmission()->GetTemplateName() << "\n";
    }

    if (m_driveline)
        cout << "Driveline:      " << m_driveline->GetTemplateName() << "\n";

    for (int i = 0; i < 2; i++) {
        cout << "Track " << i << "\n";

        {
            auto sprocket = m_tracks[i]->GetSprocket();
            cout << "  Sprocket:   " << sprocket->GetTemplateName() << "\n";

            int body_tag = sprocket->GetBodyTag();
            auto vehicle_tag = VehicleObjTag::ExtractVehicleTag(body_tag);
            auto part_tag = VehicleObjTag::ExtractPartTag(body_tag);
            cout << "         vehicle tag: " << sprocket->GetVehicleTag();
            cout << "         body tag:    " << body_tag << " [ " << vehicle_tag << " + " << part_tag << " ]" << endl;
        }

        {
            auto idler = m_tracks[i]->GetIdler();
            cout << "  Idler:   " << idler->GetTemplateName() << "\n";

            int body_tag = idler->GetBodyTag();
            auto vehicle_tag = VehicleObjTag::ExtractVehicleTag(body_tag);
            auto part_tag = VehicleObjTag::ExtractPartTag(body_tag);
            cout << "         vehicle tag: " << idler->GetVehicleTag();
            cout << "         body tag:    " << body_tag << " [ " << vehicle_tag << " + " << part_tag << " ]" << endl;
        }

        {
            auto suspension = m_tracks[i]->GetTrackSuspension(0);
            cout << "  Suspension:   " << suspension->GetTemplateName() << "\n";

            int body_tag = suspension->GetBodyTag();
            auto vehicle_tag = VehicleObjTag::ExtractVehicleTag(body_tag);
            auto part_tag = VehicleObjTag::ExtractPartTag(body_tag);
            cout << "         vehicle tag: " << suspension->GetVehicleTag();
            cout << "         body tag:    " << body_tag << " [ " << vehicle_tag << " + " << part_tag << " ]" << endl;
        }

        {
            auto road_wheel = m_tracks[i]->GetRoadWheel(0);
            cout << "  Road-wheel:   " << road_wheel->GetTemplateName() << "\n";

            int body_tag = road_wheel->GetBodyTag();
            auto vehicle_tag = VehicleObjTag::ExtractVehicleTag(body_tag);
            auto part_tag = VehicleObjTag::ExtractPartTag(body_tag);
            cout << "         vehicle tag: " << road_wheel->GetVehicleTag();
            cout << "         body tag:    " << body_tag << " [ " << vehicle_tag << " + " << part_tag << " ]" << endl;
        }

        if (m_tracks[i]->GetNumRollers() > 0) {
            auto roller = m_tracks[i]->GetRoller(0);
            cout << "  Roller:   " << roller->GetTemplateName() << "\n";

            int body_tag = roller->GetBodyTag();
            auto vehicle_tag = VehicleObjTag::ExtractVehicleTag(body_tag);
            auto part_tag = VehicleObjTag::ExtractPartTag(body_tag);
            cout << "         vehicle tag: " << roller->GetVehicleTag();
            cout << "         body tag:    " << body_tag << " [ " << vehicle_tag << " + " << part_tag << " ]" << endl;
        }
    }
}

// -----------------------------------------------------------------------------

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

    //// TODO add array of rear chassis subsystems

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

    for (auto& c : m_chassis_rear) {
        if (c->OutputEnabled()) {
            database.WriteSection(c->GetName());
            c->Output(database);
        }
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
