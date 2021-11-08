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
// Base class for a wheeled vehicle model.
//
// =============================================================================

#include <fstream>

#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicle.h"

#include "chrono_thirdparty/rapidjson/document.h"
#include "chrono_thirdparty/rapidjson/prettywriter.h"
#include "chrono_thirdparty/rapidjson/stringbuffer.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChWheeledVehicle::ChWheeledVehicle(const std::string& name, ChContactMethod contact_method)
    : ChVehicle(name, contact_method), m_parking_on(false) {}

ChWheeledVehicle::ChWheeledVehicle(const std::string& name, ChSystem* system)
    : ChVehicle(name, system), m_parking_on(false) {}

// -----------------------------------------------------------------------------
// Initialize this vehicle at the specified global location and orientation.
// This base class implementation only initializes the main chassis subsystem.
// Derived classes must extend this function to initialize all other wheeled
// vehicle subsystems (axles, steerings, driveline).
// -----------------------------------------------------------------------------
void ChWheeledVehicle::Initialize(const ChCoordsys<>& chassisPos, double chassisFwdVel) {
    m_chassis->Initialize(m_system, chassisPos, chassisFwdVel, WheeledCollisionFamily::CHASSIS);
}

// -----------------------------------------------------------------------------
// Initialize a tire and attach it to one of the vehicle's wheels.
// -----------------------------------------------------------------------------
void ChWheeledVehicle::InitializeTire(std::shared_ptr<ChTire> tire,
                                      std::shared_ptr<ChWheel> wheel,
                                      VisualizationType tire_vis,
                                      ChTire::CollisionType tire_coll) {
    wheel->m_tire = tire;
    tire->Initialize(wheel);
    tire->SetVisualizationType(tire_vis);
    tire->SetCollisionType(tire_coll);
}

// -----------------------------------------------------------------------------
// Initialize a powertrain system and associate it with this vehicle.
// -----------------------------------------------------------------------------
void ChWheeledVehicle::InitializePowertrain(std::shared_ptr<ChPowertrain> powertrain) {
    m_powertrain = powertrain;
    powertrain->Initialize(m_chassis);
}

// -----------------------------------------------------------------------------
// Update the state of this vehicle at the current time.
// The vehicle system is provided the current driver inputs (throttle between 0
// and 1, steering between -1 and +1, braking between 0 and 1), and a reference
// to the terrain system.
// -----------------------------------------------------------------------------
void ChWheeledVehicle::Synchronize(double time, const ChDriver::Inputs& driver_inputs, const ChTerrain& terrain) {
    double powertrain_torque = 0;
    if (m_powertrain) {
        // Extract the torque from the powertrain.
        powertrain_torque = m_powertrain->GetOutputTorque();
        // Synchronize the associated powertrain system (pass throttle input).
        m_powertrain->Synchronize(time, driver_inputs.m_throttle, m_driveline->GetDriveshaft()->GetPos_dt());
    }

    // Apply powertrain torque to the driveline's input shaft.
    m_driveline->Synchronize(powertrain_torque);

    // Let the steering subsystems process the steering input.
    for (auto& steering : m_steerings) {
        steering->Synchronize(time, driver_inputs.m_steering);
    }

    // Pass the steering input to any chassis connectors (in case one of them is actuated)
    for (auto& connector : m_chassis_connectors) {
        connector->Synchronize(time, driver_inputs.m_steering);
    }

    // Synchronize the vehicle's axle subsystems
    for (auto& axle : m_axles) {
        for (auto& wheel : axle->GetWheels()) {
            if (wheel->m_tire)
                wheel->m_tire->Synchronize(time, terrain);
        }
        axle->Synchronize(driver_inputs.m_braking);
    }

    m_chassis->Synchronize(time);
    for (auto& c : m_chassis_rear)
        c->Synchronize(time);
}

// -----------------------------------------------------------------------------
// Advance the state of this vehicle by the specified time step.
// -----------------------------------------------------------------------------
void ChWheeledVehicle::Advance(double step) {
    if (m_powertrain) {
        // Advance state of the associated powertrain.
        m_powertrain->Advance(step);
    }

    // Advance state of all vehicle tires.
    // This is done before advancing the state of the multibody system in order to use
    // wheel states corresponding to current time.
    for (auto& axle : m_axles) {
        for (auto& wheel : axle->GetWheels()) {
            if (wheel->m_tire)
                wheel->m_tire->Advance(step);
        }
    }

    // Invoke base class function to advance state of underlying Chrono system.
    ChVehicle::Advance(step);
}

// -----------------------------------------------------------------------------
// Enable/disable differential locking.
// -----------------------------------------------------------------------------
void ChWheeledVehicle::LockAxleDifferential(int axle, bool lock) {
    m_driveline->LockAxleDifferential(axle, lock);
}

void ChWheeledVehicle::LockCentralDifferential(int which, bool lock) {
    m_driveline->LockCentralDifferential(which, lock);
}

// Disconnect driveline
void ChWheeledVehicle::DisconnectDriveline() {
    if (m_driveline)
        m_driveline->Disconnect();
}

// -----------------------------------------------------------------------------
// Brake behavior
// -----------------------------------------------------------------------------
void ChWheeledVehicle::EnableBrakeLocking(bool lock) {
    for (auto& axle : m_axles) {
        if (axle->m_brake_left)
            axle->m_brake_left->EnableLocking(lock);
        if (axle->m_brake_right)
            axle->m_brake_right->EnableLocking(lock);
    }
}

void ChWheeledVehicle::ApplyParkingBrake(bool lock) {
    if (m_parking_on == lock)
        return;

    for (auto& axle : m_axles) {
        axle->m_suspension->ApplyParkingBrake(lock);
    }
    m_parking_on = lock;
}

// -----------------------------------------------------------------------------
// Set visualization type for the various subsystems
// -----------------------------------------------------------------------------
void ChWheeledVehicle::SetSubchassisVisualizationType(VisualizationType vis) {
    for (auto& sc : m_subchassis)
        sc->SetVisualizationType(vis);
}

void ChWheeledVehicle::SetSuspensionVisualizationType(VisualizationType vis) {
    for (auto& axle : m_axles) {
        axle->m_suspension->SetVisualizationType(vis);
    }
}

void ChWheeledVehicle::SetSteeringVisualizationType(VisualizationType vis) {
    for (auto& steering : m_steerings) {
        steering->SetVisualizationType(vis);
    }
}

void ChWheeledVehicle::SetWheelVisualizationType(VisualizationType vis) {
    for (auto& axle : m_axles) {
        for (auto& wheel : axle->m_wheels) {
            wheel->SetVisualizationType(vis);
        }
    }
}

// -----------------------------------------------------------------------------
// Enable/disable collision between the chassis and all other vehicle subsystems
// This only controls collisions between the chassis and the tire systems.
// -----------------------------------------------------------------------------
void ChWheeledVehicle::SetChassisVehicleCollide(bool state) {
    if (state) {
        // Chassis collides with tires
        m_chassis->GetBody()->GetCollisionModel()->SetFamilyMaskDoCollisionWithFamily(WheeledCollisionFamily::TIRES);
        for (auto& c : m_chassis_rear)
            c->GetBody()->GetCollisionModel()->SetFamilyMaskDoCollisionWithFamily(WheeledCollisionFamily::TIRES);
    } else {
        // Chassis does not collide with tires
        m_chassis->GetBody()->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(WheeledCollisionFamily::TIRES);
        for (auto& c : m_chassis_rear)
            c->GetBody()->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(WheeledCollisionFamily::TIRES);
    }
}

// -----------------------------------------------------------------------------
// Enable/disable output from the various subsystems
// -----------------------------------------------------------------------------
void ChWheeledVehicle::SetSuspensionOutput(int id, bool state) {
    m_axles[id]->m_suspension->SetOutput(state);
}

void ChWheeledVehicle::SetSteeringOutput(int id, bool state) {
    m_steerings[id]->SetOutput(state);
}

void ChWheeledVehicle::SetAntirollbarOutput(int id, bool state) {
    assert(m_axles[id]->m_antirollbar);
    m_axles[id]->m_antirollbar->SetOutput(state);
}

void ChWheeledVehicle::SetDrivelineOutput(bool state) {
    m_driveline->SetOutput(state);
}

// -----------------------------------------------------------------------------
// Get the specified wheel or tire (axle, side, location)
// -----------------------------------------------------------------------------
std::shared_ptr<ChWheel> ChWheeledVehicle::GetWheel(int axle, VehicleSide side, WheelLocation location) const {
    return m_axles[axle]->GetWheel(side, location);
}

std::shared_ptr<ChTire> ChWheeledVehicle::GetTire(int axle, VehicleSide side, WheelLocation location) const {
    return m_axles[axle]->GetWheel(side, location)->GetTire();
}

// -----------------------------------------------------------------------------
// Get the specified brake (axle, side)
// -----------------------------------------------------------------------------
std::shared_ptr<ChBrake> ChWheeledVehicle::GetBrake(int axle, VehicleSide side) const {
    return m_axles[axle]->GetBrake(side);
}

// -----------------------------------------------------------------------------
// Calculate and return the total vehicle mass
// -----------------------------------------------------------------------------
double ChWheeledVehicle::GetVehicleMass() const {
    double mass = m_chassis->GetMass();

    for (auto& c : m_chassis_rear)
        mass += c->GetMass();

    for (auto& sc : m_subchassis)
        mass += sc->GetMass();

    for (auto& axle : m_axles) {
        mass += axle->m_suspension->GetMass();
        for (auto& wheel : axle->GetWheels())
            mass += wheel->GetMass();
        if (axle->m_antirollbar)
            mass += axle->m_antirollbar->GetMass();
    }

    for (auto& steering : m_steerings)
        mass += steering->GetMass();

    return mass;
}

// -----------------------------------------------------------------------------
// Calculate and return the current vehicle COM location
// -----------------------------------------------------------------------------
ChVector<> ChWheeledVehicle::GetVehicleCOMPos() const {
    ChVector<> com(0, 0, 0);

    com += m_chassis->GetMass() * m_chassis->GetCOMPos();

    for (auto& c : m_chassis_rear)
        com += c->GetMass() * c->GetCOMPos();

    for (auto& sc : m_subchassis)
        com += sc->GetMass() * sc->GetCOMPos();

    for (auto& axle : m_axles) {
        com += axle->m_suspension->GetMass() * axle->m_suspension->GetCOMPos();
        for (auto& wheel : axle->GetWheels())
            com += wheel->GetMass() * wheel->GetPos();
        if (axle->m_antirollbar)
            com += axle->m_antirollbar->GetMass() * axle->m_antirollbar->GetCOMPos();
    }
    
    for (auto steering : m_steerings)
        com += steering->GetMass() * steering->GetCOMPos();

    return com / GetVehicleMass();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
const ChVector<>& ChWheeledVehicle::GetSpindlePos(int axle, VehicleSide side) const {
    return m_axles[axle]->m_suspension->GetSpindlePos(side);
}

ChQuaternion<> ChWheeledVehicle::GetSpindleRot(int axle, VehicleSide side) const {
    return m_axles[axle]->m_suspension->GetSpindleRot(side);
}

const ChVector<>& ChWheeledVehicle::GetSpindleLinVel(int axle, VehicleSide side) const {
    return m_axles[axle]->m_suspension->GetSpindleLinVel(side);
}

ChVector<> ChWheeledVehicle::GetSpindleAngVel(int axle, VehicleSide side) const {
    return m_axles[axle]->m_suspension->GetSpindleAngVel(side);
}

double ChWheeledVehicle::GetSpindleOmega(int axle, VehicleSide side) const {
    return m_axles[axle]->m_suspension->GetAxleSpeed(side);
}

// -----------------------------------------------------------------------------
// Estimate the maximum steering angle based on a bicycle model, from the vehicle
// minimum turning radius, the wheelbase, and the track of the front suspension.
// -----------------------------------------------------------------------------
double ChWheeledVehicle::GetMaxSteeringAngle() const {
    return std::asin(GetWheelbase() / (GetMinTurningRadius() - 0.5 * GetWheeltrack(0)));
}

// -----------------------------------------------------------------------------
// Log constraint violations
// -----------------------------------------------------------------------------
void ChWheeledVehicle::LogConstraintViolations() {
    GetLog().SetNumFormat("%16.4e");

    // Report constraint violations for the suspension joints
    for (size_t i = 0; i < m_axles.size(); i++) {
        GetLog() << "\n---- AXLE " << i << " LEFT side suspension constraint violations\n\n";
        m_axles[i]->m_suspension->LogConstraintViolations(LEFT);
        GetLog() << "\n---- AXLE " << i << " RIGHT side suspension constraint violations\n\n";
        m_axles[i]->m_suspension->LogConstraintViolations(RIGHT);
    }

    // Report constraint violations for the steering joints
    for (size_t i = 0; i < m_steerings.size(); i++) {
        GetLog() << "\n---- STEERING subsystem " << i << " constraint violations\n\n";
        m_steerings[i]->LogConstraintViolations();
    }

    GetLog().SetNumFormat("%g");
}

// -----------------------------------------------------------------------------

void ChWheeledVehicle::LogSubsystemTypes() {
    GetLog() << "\nSubsystem types\n";
    GetLog() << "Chassis:        " << m_chassis->GetTemplateName().c_str() << "\n";
    GetLog() << "Powertrain:     " << m_powertrain->GetTemplateName().c_str() << "\n";
    GetLog() << "Driveline:      " << m_driveline->GetTemplateName().c_str() << "\n";

    for (int i = 0; i < m_steerings.size(); i++) {
        GetLog() << "Steering " << i << ":     " << m_steerings[i]->GetTemplateName().c_str() << "\n";
    }

    for (int i = 0; i < m_axles.size(); i++) {
        GetLog() << "Axle " << i << "\n";
        GetLog() << "  Suspension:   " << m_axles[i]->m_suspension->GetTemplateName().c_str() << "\n";
        if (m_axles[i]->m_antirollbar)
            GetLog() << "  Antiroll bar: " << m_axles[i]->m_brake_left->GetTemplateName().c_str() << "\n";
        if (m_axles[i]->m_brake_left)
            GetLog() << "  Brake:        " << m_axles[i]->m_brake_left->GetTemplateName().c_str() << "\n";
        GetLog() << "  Tire:         " << GetTire(i, LEFT)->GetTemplateName().c_str() << "\n";
    }
}

// -----------------------------------------------------------------------------

std::string ChWheeledVehicle::ExportComponentList() const {
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

    rapidjson::Value sterringArray(rapidjson::kArrayType);
    for (auto& steering : m_steerings) {
        rapidjson::Document jsonSubDocument(&jsonDocument.GetAllocator());
        jsonSubDocument.SetObject();
        steering->ExportComponentList(jsonSubDocument);
        sterringArray.PushBack(jsonSubDocument, jsonDocument.GetAllocator());
    }
    jsonDocument.AddMember("steering", sterringArray, jsonDocument.GetAllocator());

    rapidjson::Value suspArray(rapidjson::kArrayType);
    for (auto& axle : m_axles) {
        rapidjson::Document jsonSubDocument(&jsonDocument.GetAllocator());
        jsonSubDocument.SetObject();
        axle->m_suspension->ExportComponentList(jsonSubDocument);
        suspArray.PushBack(jsonSubDocument, jsonDocument.GetAllocator());
    }
    jsonDocument.AddMember("suspension", suspArray, jsonDocument.GetAllocator());

    rapidjson::Value brakeArray(rapidjson::kArrayType);
    for (auto& axle : m_axles) {
        {
            rapidjson::Document jsonSubDocument(&jsonDocument.GetAllocator());
            jsonSubDocument.SetObject();
            axle->m_brake_left->ExportComponentList(jsonSubDocument);
            brakeArray.PushBack(jsonSubDocument, jsonDocument.GetAllocator());
        }
        {
            rapidjson::Document jsonSubDocument(&jsonDocument.GetAllocator());
            jsonSubDocument.SetObject();
            axle->m_brake_right->ExportComponentList(jsonSubDocument);
            brakeArray.PushBack(jsonSubDocument, jsonDocument.GetAllocator());
        }
    }
    jsonDocument.AddMember("brake", brakeArray, jsonDocument.GetAllocator());

    rapidjson::Value arArray(rapidjson::kArrayType);
    for (auto& axle : m_axles) {
        if (axle->m_antirollbar) {
            rapidjson::Document jsonSubDocument(&jsonDocument.GetAllocator());
            jsonSubDocument.SetObject();
            axle->m_antirollbar->ExportComponentList(jsonSubDocument);
            arArray.PushBack(jsonSubDocument, jsonDocument.GetAllocator());
        }
    }
    jsonDocument.AddMember("anti-roll bar", arArray, jsonDocument.GetAllocator());

    rapidjson::StringBuffer jsonBuffer;
    rapidjson::PrettyWriter<rapidjson::StringBuffer> jsonWriter(jsonBuffer);
    jsonDocument.Accept(jsonWriter);

    return jsonBuffer.GetString();
}

void ChWheeledVehicle::ExportComponentList(const std::string& filename) const {
    std::ofstream of(filename);
    of << ExportComponentList();
    of.close();
}

void ChWheeledVehicle::Output(int frame, ChVehicleOutput& database) const {
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

    for (auto& steering : m_steerings) {
        if (steering->OutputEnabled()) {
            database.WriteSection(steering->GetName());
            steering->Output(database);
        }
    }

    for (auto& axle : m_axles) {
        if (axle->m_suspension->OutputEnabled()) {
            database.WriteSection(axle->m_suspension->GetName());
            axle->m_suspension->Output(database);
        }
        if (axle->m_brake_left->OutputEnabled()) {
            database.WriteSection(axle->m_brake_left->GetName());
            axle->m_brake_left->Output(database);
        }
        if (axle->m_brake_right->OutputEnabled()) {
            database.WriteSection(axle->m_brake_right->GetName());
            axle->m_brake_right->Output(database);
        }
        if (axle->m_antirollbar && axle->m_antirollbar->OutputEnabled()) {
            database.WriteSection(axle->m_antirollbar->GetName());
            axle->m_antirollbar->Output(database);
        }
    }
}

}  // end namespace vehicle
}  // end namespace chrono
