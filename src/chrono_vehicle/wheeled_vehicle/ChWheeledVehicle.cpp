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

#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicle.h"

#include "chrono_thirdparty/rapidjson/document.h"
#include "chrono_thirdparty/rapidjson/prettywriter.h"
#include "chrono_thirdparty/rapidjson/stringbuffer.h"

using std::cout;
using std::endl;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChWheeledVehicle::ChWheeledVehicle(const std::string& name, ChContactMethod contact_method)
    : ChVehicle(name, contact_method), m_parking_on(false) {}

ChWheeledVehicle::ChWheeledVehicle(const std::string& name, ChSystem* system)
    : ChVehicle(name, system), m_parking_on(false) {}

// -----------------------------------------------------------------------------
// Initialize a tire and attach it to one of the vehicle's wheels.
// -----------------------------------------------------------------------------
void ChWheeledVehicle::InitializeTire(std::shared_ptr<ChTire> tire,
                                      std::shared_ptr<ChWheel> wheel,
                                      VisualizationType tire_vis,
                                      ChTire::CollisionType tire_coll) {
    wheel->m_tire = tire;
    tire->Initialize(wheel);
    tire->InitializeInertiaProperties();
    tire->SetVisualizationType(tire_vis);
    tire->SetCollisionType(tire_coll);

    // Recalculate vehicle mass to include the mass of the tire.
    InitializeInertiaProperties();
}

// -----------------------------------------------------------------------------
// Update the state of this vehicle at the current time.
// The vehicle system is provided the current driver inputs (throttle between 0
// and 1, steering between -1 and +1, braking between 0 and 1), and a reference
// to the terrain system.
// -----------------------------------------------------------------------------
void ChWheeledVehicle::Synchronize(double time, const DriverInputs& driver_inputs) {
    double powertrain_torque = m_powertrain_assembly ? m_powertrain_assembly->GetOutputTorque() : 0;
    double driveline_speed = m_driveline ? m_driveline->GetOutputDriveshaftSpeed() : 0;

    // Set driveshaft speed for the transmission output shaft
    if (m_powertrain_assembly)
        m_powertrain_assembly->Synchronize(time, driver_inputs, driveline_speed);

    // Apply powertrain torque to the driveline's input shaft
    if (m_driveline)
        m_driveline->Synchronize(time, driver_inputs, powertrain_torque);

    // Let the steering subsystems process the steering input
    for (auto& steering : m_steerings) {
        steering->Synchronize(time, driver_inputs);
    }

    // Pass the steering input to any chassis connectors (in case one of them is actuated)
    for (auto& connector : m_chassis_connectors) {
        connector->Synchronize(time, driver_inputs);
    }

    // Synchronize the vehicle's axle subsystems and any associated tires
    for (auto& axle : m_axles) {
        axle->Synchronize(time, driver_inputs);
    }

    // Synchronize the main and rear chassis subsystems
    m_chassis->Synchronize(time);
    for (auto& c : m_chassis_rear)
        c->Synchronize(time);
}

void ChWheeledVehicle::Synchronize(double time, const DriverInputs& driver_inputs, const ChTerrain& terrain) {
    // Synchronize any associated tires
    for (auto& axle : m_axles) {
        for (auto& wheel : axle->GetWheels()) {
            if (wheel->m_tire)
                wheel->m_tire->Synchronize(time, terrain);
        }
    }

    Synchronize(time, driver_inputs);
}

// -----------------------------------------------------------------------------
// Advance the state of this vehicle by the specified time step.
// -----------------------------------------------------------------------------
void ChWheeledVehicle::Advance(double step) {
    // Advance state of the associated powertrain (if any)
    if (m_powertrain_assembly) {
        m_powertrain_assembly->Advance(step);
    }

    // Advance state of all axles and vehicle tires.
    // This is done before advancing the state of the multibody system in order to use wheel states corresponding to
    // current time.
    for (auto& axle : m_axles) {
        for (auto& wheel : axle->GetWheels()) {
            if (wheel->m_tire)
                wheel->m_tire->Advance(step);
        }
        axle->Advance(step);
    }

    // Invoke base class function to advance state of underlying Chrono system.
    ChVehicle::Advance(step);
}

// -----------------------------------------------------------------------------
// Enable/disable differential locking.
// -----------------------------------------------------------------------------
void ChWheeledVehicle::LockAxleDifferential(int axle, bool lock) {
    if (m_driveline)
        m_driveline->LockAxleDifferential(axle, lock);
}

void ChWheeledVehicle::LockCentralDifferential(int which, bool lock) {
    if (m_driveline)
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
        if (!axle->m_suspension->IsSteerable())
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

void ChWheeledVehicle::SetTireVisualizationType(VisualizationType vis) {
    for (auto& axle : m_axles) {
        for (auto& wheel : axle->m_wheels) {
            if (wheel->GetTire())
                wheel->GetTire()->SetVisualizationType(vis);
        }
    }
}

// -----------------------------------------------------------------------------

void ChWheeledVehicle::SetWheelCollide(bool state) {
    for (auto& axle : m_axles) {
        for (auto& wheel : axle->m_wheels) {
            wheel->EnableCollision(state);
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
        m_chassis->GetBody()->GetCollisionModel()->AllowCollisionsWith(VehicleCollisionFamily::TIRE_FAMILY);
        m_chassis->GetBody()->GetCollisionModel()->AllowCollisionsWith(VehicleCollisionFamily::WHEEL_FAMILY);
        for (auto& c : m_chassis_rear) {
            c->GetBody()->GetCollisionModel()->AllowCollisionsWith(VehicleCollisionFamily::TIRE_FAMILY);
            c->GetBody()->GetCollisionModel()->AllowCollisionsWith(VehicleCollisionFamily::WHEEL_FAMILY);
        }
    } else {
        // Chassis does not collide with tires
        m_chassis->GetBody()->GetCollisionModel()->DisallowCollisionsWith(VehicleCollisionFamily::TIRE_FAMILY);
        m_chassis->GetBody()->GetCollisionModel()->DisallowCollisionsWith(VehicleCollisionFamily::WHEEL_FAMILY);
        for (auto& c : m_chassis_rear) {
            c->GetBody()->GetCollisionModel()->DisallowCollisionsWith(VehicleCollisionFamily::TIRE_FAMILY);
            c->GetBody()->GetCollisionModel()->DisallowCollisionsWith(VehicleCollisionFamily::WHEEL_FAMILY);
        }
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

void ChWheeledVehicle::SetSubchassisOutput(int id, bool state) {
    m_subchassis[id]->SetOutput(state);
}

void ChWheeledVehicle::SetAntirollbarOutput(int id, bool state) {
    assert(m_axles[id]->m_antirollbar);
    m_axles[id]->m_antirollbar->SetOutput(state);
}

void ChWheeledVehicle::SetDrivelineOutput(bool state) {
    if (m_driveline)
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
// Calculate the total vehicle mass
// -----------------------------------------------------------------------------
void ChWheeledVehicle::InitializeInertiaProperties() {
    m_mass = 0;

    m_chassis->AddMass(m_mass);

    for (auto& c : m_chassis_rear)
        c->AddMass(m_mass);

    for (auto& sc : m_subchassis)
        sc->AddMass(m_mass);

    for (auto& axle : m_axles) {
        axle->m_suspension->AddMass(m_mass);

        for (auto& wheel : axle->GetWheels()) {
            wheel->AddMass(m_mass);
            if (wheel->GetTire()) {
                wheel->GetTire()->AddMass(m_mass);
            }
        }

        if (axle->m_antirollbar)
            axle->m_antirollbar->AddMass(m_mass);
    }

    for (auto& steering : m_steerings)
        steering->AddMass(m_mass);
}

// -----------------------------------------------------------------------------
// Calculate current vehicle inertia properties
// -----------------------------------------------------------------------------
void ChWheeledVehicle::UpdateInertiaProperties() {
    // 1. Calculate vehicle COM location relative to the global reference frame
    // 2. Calculate vehicle inertia relative to global reference frame
    ChVector3d com(0);
    ChMatrix33<> inertia(0);

    m_chassis->AddInertiaProperties(com, inertia);

    for (auto& c : m_chassis_rear)
        c->AddInertiaProperties(com, inertia);

    for (auto& sc : m_subchassis)
        sc->AddInertiaProperties(com, inertia);

    for (auto& axle : m_axles) {
        axle->m_suspension->AddInertiaProperties(com, inertia);

        for (auto& wheel : axle->GetWheels()) {
            wheel->AddInertiaProperties(com, inertia);
            if (wheel->GetTire()) {
                wheel->GetTire()->AddInertiaProperties(com, inertia);
            }
        }

        if (axle->m_antirollbar)
            axle->m_antirollbar->AddInertiaProperties(com, inertia);
    }

    for (auto& steering : m_steerings)
        steering->AddInertiaProperties(com, inertia);

    // 3. Express vehicle COM frame relative to vehicle reference frame
    m_com.SetPos(GetTransform().TransformPointParentToLocal(com / GetMass()));
    m_com.SetRot(GetTransform().GetRot());

    // 4. Express inertia relative to vehicle COM frame
    //    Notes: - vehicle COM frame aligned with vehicle frame
    //           - 'com' still scaled by total mass here
    const ChMatrix33<>& A = GetTransform().GetRotMat();
    m_inertia = A.transpose() * (inertia - utils::CompositeInertia::InertiaShiftMatrix(com) / GetMass()) * A;
}

// -----------------------------------------------------------------------------
const ChVector3d& ChWheeledVehicle::GetSpindlePos(int axle, VehicleSide side) const {
    return m_axles[axle]->m_suspension->GetSpindlePos(side);
}

ChQuaternion<> ChWheeledVehicle::GetSpindleRot(int axle, VehicleSide side) const {
    return m_axles[axle]->m_suspension->GetSpindleRot(side);
}

const ChVector3d& ChWheeledVehicle::GetSpindleLinVel(int axle, VehicleSide side) const {
    return m_axles[axle]->m_suspension->GetSpindleLinVel(side);
}

ChVector3d ChWheeledVehicle::GetSpindleAngVel(int axle, VehicleSide side) const {
    return m_axles[axle]->m_suspension->GetSpindleAngVel(side);
}

double ChWheeledVehicle::GetSpindleOmega(int axle, VehicleSide side) const {
    return m_axles[axle]->m_suspension->GetAxleSpeed(side);
}

// Note that this function cannot be a member function of ChWheel or ChTire since it requires the frames of both the
// wheel and the chassis.
double ChWheeledVehicle::GetSteeringAngle(int axle, VehicleSide side) const {
    // Spindle body
    auto spindle = m_axles[axle]->m_suspension->GetSpindle(side);
    // Spindle body frame expressed in chassis frame
    auto spindle_loc = m_chassis->GetBody()->TransformParentToLocal(*spindle);
    // Use projection of the spindle_loc Y axis onto the XY plane
    auto y_axis = spindle_loc.GetRotMat().GetAxisY();
    // Calculate steering angle (positive for turning to the left)
    double angle = -std::atan2(y_axis[0], y_axis[1]);

    return angle;
}

// -----------------------------------------------------------------------------
// Estimate the maximum steering angle based on a bicycle model, from the vehicle
// minimum turning radius, the wheelbase, and the track of the front suspension.
// -----------------------------------------------------------------------------
double ChWheeledVehicle::GetMaxSteeringAngle() const {
    return std::asin(GetWheelbase() / (GetMinTurningRadius() - 0.5 * GetWheeltrack(0)));
}

// -----------------------------------------------------------------------------

void ChWheeledVehicle::LogConstraintViolations() {
    // Report constraint violations for the suspension joints
    for (size_t i = 0; i < m_axles.size(); i++) {
        cout << "\n---- AXLE " << i << " LEFT side suspension constraint violations\n\n";
        m_axles[i]->m_suspension->LogConstraintViolations(LEFT);
        cout << "\n---- AXLE " << i << " RIGHT side suspension constraint violations\n\n";
        m_axles[i]->m_suspension->LogConstraintViolations(RIGHT);
    }

    // Report constraint violations for the steering joints
    for (size_t i = 0; i < m_steerings.size(); i++) {
        cout << "\n---- STEERING subsystem " << i << " constraint violations\n\n";
        m_steerings[i]->LogConstraintViolations();
    }
}

void ChWheeledVehicle::LogSubsystemTypes() {
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

    for (int i = 0; i < m_steerings.size(); i++) {
        cout << "Steering " << i << ":     " << m_steerings[i]->GetTemplateName() << "\n";

        int body_tag = m_steerings[i]->GetBodyTag();
        auto vehicle_tag = VehicleObjTag::ExtractVehicleTag(body_tag);
        auto part_tag = VehicleObjTag::ExtractPartTag(body_tag);
        cout << "         vehicle tag: " << m_steerings[i]->GetVehicleTag();
        cout << "         body tag:    " << body_tag << " [ " << vehicle_tag << " + " << part_tag << " ]" << endl;
    }

    for (int i = 0; i < m_axles.size(); i++) {
        cout << "Axle " << i << "\n";

        {
            cout << "  Suspension:   " << m_axles[i]->m_suspension->GetTemplateName() << "\n";

            int body_tag = m_axles[i]->m_suspension->GetBodyTag();
            auto vehicle_tag = VehicleObjTag::ExtractVehicleTag(body_tag);
            auto part_tag = VehicleObjTag::ExtractPartTag(body_tag);
            cout << "         vehicle tag: " << m_axles[i]->m_suspension->GetVehicleTag();
            cout << "         body tag:    " << body_tag << " [ " << vehicle_tag << " + " << part_tag << " ]" << endl;
        }

        if (m_axles[i]->m_antirollbar) {
            cout << "  Antiroll bar: " << m_axles[i]->m_antirollbar->GetTemplateName() << "\n";

            int body_tag = m_axles[i]->m_antirollbar->GetBodyTag();
            auto vehicle_tag = VehicleObjTag::ExtractVehicleTag(body_tag);
            auto part_tag = VehicleObjTag::ExtractPartTag(body_tag);
            cout << "         vehicle tag: " << m_axles[i]->m_antirollbar->GetVehicleTag();
            cout << "         body tag:    " << body_tag << " [ " << vehicle_tag << " + " << part_tag << " ]" << endl;
        }
        
        if (m_axles[i]->m_brake_left) {
            cout << "  Brake:        " << m_axles[i]->m_brake_left->GetTemplateName() << "\n";
            cout << "         vehicle tag: " << m_axles[i]->m_wheels[0]->GetVehicleTag();
        }

        {
            cout << "  Wheel:        " << m_axles[i]->m_wheels[0]->GetTemplateName() << "\n";

            int body_tag = m_axles[i]->m_wheels[0]->GetBodyTag();
            auto vehicle_tag = VehicleObjTag::ExtractVehicleTag(body_tag);
            auto part_tag = VehicleObjTag::ExtractPartTag(body_tag);
            cout << "         vehicle tag: " << m_axles[i]->m_wheels[0]->GetVehicleTag();
            cout << "         body tag:    " << body_tag << " [ " << vehicle_tag << " + " << part_tag << " ]" << endl;
        }

        if (m_axles[i]->m_wheels.size() == 2) {
            cout << "  Tire:         " << GetTire(i, LEFT, SINGLE)->GetTemplateName() << "\n";
        } else {
            cout << "  Tire:         " << GetTire(i, LEFT, INNER)->GetTemplateName() << "\n";
        }
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

    rapidjson::Value subchassisArray(rapidjson::kArrayType);
    for (auto& subchassis : m_subchassis) {
        rapidjson::Document jsonSubDocument(&jsonDocument.GetAllocator());
        jsonSubDocument.SetObject();
        subchassis->ExportComponentList(jsonSubDocument);
        subchassisArray.PushBack(jsonSubDocument, jsonDocument.GetAllocator());
    }
    jsonDocument.AddMember("subchassis", subchassisArray, jsonDocument.GetAllocator());

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
        if (axle->m_brake_left) {
            rapidjson::Document jsonSubDocument(&jsonDocument.GetAllocator());
            jsonSubDocument.SetObject();
            axle->m_brake_left->ExportComponentList(jsonSubDocument);
            brakeArray.PushBack(jsonSubDocument, jsonDocument.GetAllocator());
        }
        if (axle->m_brake_right) {
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

    for (auto& subchassis : m_subchassis) {
        if (subchassis->OutputEnabled()) {
            database.WriteSection(subchassis->GetName());
            subchassis->Output(database);
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
        if (axle->m_brake_left && axle->m_brake_left->OutputEnabled()) {
            database.WriteSection(axle->m_brake_left->GetName());
            axle->m_brake_left->Output(database);
        }
        if (axle->m_brake_right && axle->m_brake_right->OutputEnabled()) {
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
