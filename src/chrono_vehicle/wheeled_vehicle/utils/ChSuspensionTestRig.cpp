// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Justin Madsen
// =============================================================================
//
// Implementation of a suspension testing mechanism (as a vehicle).
// The tested suspension can be specified through a stand-alone JSON file (and
// may or may not include a steering subsystem), or as a specified axle in a
// vehicle JSON specification file.
//
// The reference frame follows the ISO standard: Z-axis up, X-axis
// pointing forward, and Y-axis towards the left of the vehicle.
//
// =============================================================================

#include <cstdio>
#include <algorithm>

#include "chrono/assets/ChCylinderShape.h"

#include "chrono_vehicle/wheeled_vehicle/utils/ChSuspensionTestRig.h"

#include "chrono_vehicle/wheeled_vehicle/suspension/DoubleWishbone.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/DoubleWishboneReduced.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/SolidAxle.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/MultiLink.h"

#include "chrono_vehicle/wheeled_vehicle/steering/PitmanArm.h"
#include "chrono_vehicle/wheeled_vehicle/steering/RackPinion.h"

#include "chrono_vehicle/wheeled_vehicle/wheel/Wheel.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "thirdparty/rapidjson/document.h"
#include "thirdparty/rapidjson/filereadstream.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// These utility functions return a ChVector and a ChQuaternion, respectively,
// from the specified JSON array.
// -----------------------------------------------------------------------------
static ChVector<> loadVector(const Value& a) {
    assert(a.IsArray());
    assert(a.Size() == 3);
    return ChVector<>(a[0u].GetDouble(), a[1u].GetDouble(), a[2u].GetDouble());
}

static ChQuaternion<> loadQuaternion(const Value& a) {
    assert(a.IsArray());
    assert(a.Size() == 4);
    return ChQuaternion<>(a[0u].GetDouble(), a[1u].GetDouble(), a[2u].GetDouble(), a[3u].GetDouble());
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChSuspensionTestRig::LoadSteering(const std::string& filename) {
    FILE* fp = fopen(filename.c_str(), "r");

    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));

    fclose(fp);

    Document d;
    d.ParseStream(is);

    // Check that the given file is a steering specification file.
    assert(d.HasMember("Type"));
    std::string type = d["Type"].GetString();
    assert(type.compare("Steering") == 0);

    // Extract the driveline type.
    assert(d.HasMember("Template"));
    std::string subtype = d["Template"].GetString();

    // Create the steering using the appropriate template.
    // Create the driveline using the appropriate template.
    if (subtype.compare("PitmanArm") == 0) {
        m_steering = std::make_shared<PitmanArm>(d);
    } else if (subtype.compare("RackPinion") == 0) {
        m_steering = std::make_shared<RackPinion>(d);
    }

    GetLog() << "  Loaded JSON: " << filename.c_str() << "\n";
}

void ChSuspensionTestRig::LoadSuspension(const std::string& filename) {
    FILE* fp = fopen(filename.c_str(), "r");

    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));

    fclose(fp);

    Document d;
    d.ParseStream(is);

    // Check that the given file is a suspension specification file.
    assert(d.HasMember("Type"));
    std::string type = d["Type"].GetString();
    assert(type.compare("Suspension") == 0);

    // Extract the suspension type.
    assert(d.HasMember("Template"));
    std::string subtype = d["Template"].GetString();

    // Create the suspension using the appropriate template.
    if (subtype.compare("DoubleWishbone") == 0) {
        m_suspension = std::make_shared<DoubleWishbone>(d);
    } else if (subtype.compare("DoubleWishboneReduced") == 0) {
        m_suspension = std::make_shared<DoubleWishboneReduced>(d);
    } else if (subtype.compare("SolidAxle") == 0) {
        m_suspension = std::make_shared<SolidAxle>(d);
    } else if (subtype.compare("MultiLink") == 0) {
        m_suspension = std::make_shared<MultiLink>(d);
    }

    GetLog() << "  Loaded JSON: " << filename.c_str() << "\n";
}

void ChSuspensionTestRig::LoadWheel(const std::string& filename, int side) {
    FILE* fp = fopen(filename.c_str(), "r");

    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));

    fclose(fp);

    Document d;
    d.ParseStream(is);

    // Check that the given file is a wheel specification file.
    assert(d.HasMember("Type"));
    std::string type = d["Type"].GetString();
    assert(type.compare("Wheel") == 0);

    // Extract the wheel type.
    assert(d.HasMember("Template"));
    std::string subtype = d["Template"].GetString();

    // Create the wheel using the appropriate template.
    if (subtype.compare("Wheel") == 0) {
        m_wheels[side] = std::make_shared<Wheel>(d);
    }

    GetLog() << "  Loaded JSON: " << filename.c_str() << "\n";
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChSuspensionTestRig::ChSuspensionTestRig(const std::string& filename,
                                         int axle_index,
                                         ChMaterialSurfaceBase::ContactMethod contact_method)
    : ChVehicle(contact_method) {
    // ---------------------------------------------------------------
    // Open and parse the input file (vehicle JSON specification file)
    // ---------------------------------------------------------------
    FILE* fp = fopen(filename.c_str(), "r");

    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));

    fclose(fp);

    Document d;
    d.ParseStream(is);

    // Read top-level data
    assert(d.HasMember("Type"));
    assert(d.HasMember("Template"));
    assert(d.HasMember("Name"));

    // Create the chassis (ground) body, fixed, no visualization
    m_chassis = std::shared_ptr<ChBodyAuxRef>(m_system->NewBodyAuxRef());
    m_chassis->SetIdentifier(0);
    m_chassis->SetName("ground");
    m_chassis->SetFrame_COG_to_REF(ChFrame<>(ChVector<>(0, 0, 0), ChQuaternion<>(1, 0, 0, 0)));
    m_chassis->SetBodyFixed(true);

    auto blue = std::make_shared<ChColorAsset>();
    blue->SetColor(ChColor(0.2f, 0.2f, 0.8f));
    m_chassis->AddAsset(blue);

    m_system->Add(m_chassis);

    // Extract the specified axle from the vehicle's list of suspension subsystems.
    // Note that we ignore antiroll bar and brake subsystems.
    // Create the suspension and wheel subsystems.
    int num_axles = d["Axles"].Size();
    assert(axle_index >= 0 && axle_index < num_axles);

    std::string file_name = d["Axles"][axle_index]["Suspension Input File"].GetString();
    LoadSuspension(vehicle::GetDataFile(file_name));
    m_suspLoc = loadVector(d["Axles"][axle_index]["Suspension Location"]);

    int steering_index = -1;
    if (d["Axles"][axle_index].HasMember("Steering Index")) {
        steering_index = d["Axles"][axle_index]["Steering Index"].GetInt();
    }

    m_wheels.resize(2);

    file_name = d["Axles"][axle_index]["Left Wheel Input File"].GetString();
    LoadWheel(vehicle::GetDataFile(file_name), LEFT);
    file_name = d["Axles"][axle_index]["Right Wheel Input File"].GetString();
    LoadWheel(vehicle::GetDataFile(file_name), RIGHT);

    // Create the steering subsystem, if needed.
    if (steering_index >= 0) {
        std::string file_name = d["Steering Subsystems"][steering_index]["Input File"].GetString();
        LoadSteering(vehicle::GetDataFile(file_name));
        m_steeringLoc = loadVector(d["Steering Subsystems"][steering_index]["Location"]);
        m_steeringRot = loadQuaternion(d["Steering Subsystems"][steering_index]["Orientation"]);
    }

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

ChSuspensionTestRig::ChSuspensionTestRig(const std::string& filename,
                                         ChMaterialSurfaceBase::ContactMethod contact_method)
    : ChVehicle(contact_method) {
    // -----------------------------------------------------------
    // Open and parse the input file (rig JSON specification file)
    // -----------------------------------------------------------

    FILE* fp = fopen(filename.c_str(), "r");

    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));

    fclose(fp);

    Document d;
    d.ParseStream(is);

    // Read top-level data
    assert(d.HasMember("Type"));
    assert(d.HasMember("Template"));
    assert(d.HasMember("Name"));

    // Create the chassis (ground) body, fixed, no visualizastion
    m_chassis = std::shared_ptr<ChBodyAuxRef>(m_system->NewBodyAuxRef());
    m_chassis->SetIdentifier(0);
    m_chassis->SetName("ground");
    m_chassis->SetFrame_COG_to_REF(ChFrame<>(ChVector<>(0, 0, 0), ChQuaternion<>(1, 0, 0, 0)));
    m_chassis->SetBodyFixed(true);

    auto blue = std::make_shared<ChColorAsset>();
    blue->SetColor(ChColor(0.2f, 0.2f, 0.8f));
    m_chassis->AddAsset(blue);

    m_system->Add(m_chassis);

    // Create the suspension and wheel subsystems.
    assert(d.HasMember("Suspension"));
    m_wheels.resize(2);

    std::string file_name = d["Suspension"]["Input File"].GetString();
    LoadSuspension(vehicle::GetDataFile(file_name));
    m_suspLoc = loadVector(d["Suspension"]["Location"]);

    file_name = d["Suspension"]["Left Wheel Input File"].GetString();
    LoadWheel(vehicle::GetDataFile(file_name), LEFT);
    file_name = d["Suspension"]["Right Wheel Input File"].GetString();
    LoadWheel(vehicle::GetDataFile(file_name), RIGHT);

    // Create the steering subsystem, if specified
    if (d.HasMember("Steering")) {
        std::string file_name = d["Steering"]["Input File"].GetString();
        LoadSteering(vehicle::GetDataFile(file_name));
        m_steeringLoc = loadVector(d["Steering"]["Location"]);
        m_steeringRot = loadQuaternion(d["Steering"]["Orientation"]);
    }

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

void ChSuspensionTestRig::Initialize(const ChCoordsys<>& chassisPos) {
    // ---------------------------------
    // Initialize the vehicle subsystems
    // ---------------------------------

    // Initialize the suspension and steering subsystems.
    if (HasSteering()) {
        m_steering->Initialize(m_chassis, m_steeringLoc, m_steeringRot);
        m_suspension->Initialize(m_chassis, m_suspLoc, m_steering->GetSteeringLink());
    } else {
        m_suspension->Initialize(m_chassis, m_suspLoc, m_chassis);
    }

    // Initialize the two wheels
    m_wheels[0]->Initialize(m_suspension->GetSpindle(LEFT));
    m_wheels[1]->Initialize(m_suspension->GetSpindle(RIGHT));

    // --------------------------------------------
    // Create and initialize the shaker post bodies
    // --------------------------------------------

    double post_height = 0.1;
    double post_rad = 0.4;

    // Left post body (green)
    ChVector<> spindle_L_pos = m_suspension->GetSpindlePos(LEFT);
    ChVector<> post_L_pos = spindle_L_pos;
    post_L_pos.z -= (m_wheels[LEFT]->GetRadius() + post_height / 2.0);

    m_post_L = std::shared_ptr<ChBody>(m_system->NewBody());
    m_post_L->SetPos(post_L_pos);
    m_system->Add(m_post_L);
    AddVisualize_post(m_post_L, m_chassis, post_height, post_rad, ChColor(0.1f, 0.8f, 0.15f));

    // Right post body (red)
    ChVector<> spindle_R_pos = m_suspension->GetSpindlePos(RIGHT);
    ChVector<> post_R_pos = spindle_R_pos;
    post_R_pos.z -= (m_wheels[RIGHT]->GetRadius() + post_height / 2.0);

    m_post_R = std::shared_ptr<ChBody>(m_system->NewBody());
    m_post_R->SetPos(post_R_pos);
    m_system->Add(m_post_R);
    AddVisualize_post(m_post_R, m_chassis, post_height, post_rad, ChColor(0.8f, 0.1f, 0.1f));

    // ------------------------------------------
    // Create and initialize joints and actuators
    // ------------------------------------------

    // Prismatic joints to force vertical translation
    m_post_L_prismatic = std::make_shared<ChLinkLockPrismatic>();
    m_post_L_prismatic->SetNameString("L_post_prismatic");
    m_post_L_prismatic->Initialize(m_chassis, m_post_L, ChCoordsys<>(ChVector<>(post_L_pos), QUNIT));
    m_system->AddLink(m_post_L_prismatic);

    m_post_R_prismatic = std::make_shared<ChLinkLockPrismatic>();
    m_post_R_prismatic->SetNameString("R_post_prismatic");
    m_post_R_prismatic->Initialize(m_chassis, m_post_R, ChCoordsys<>(ChVector<>(post_R_pos), QUNIT));
    m_system->AddLink(m_post_R_prismatic);

    // Post actuators
    ChVector<> m1_L = post_L_pos;
    m1_L.z -= 1.0;  // offset marker 1 location 1 meter below marker 2
    m_post_L_linact = std::make_shared<ChLinkLinActuator>();
    m_post_L_linact->SetNameString("L_post_linActuator");
    m_post_L_linact->Initialize(m_chassis, m_post_L, false, ChCoordsys<>(m1_L, QUNIT), ChCoordsys<>(post_L_pos, QUNIT));
    m_post_L_linact->Set_lin_offset(1.0);

    auto func_L = std::make_shared<ChFunction_Const>(0);
    m_post_L_linact->Set_dist_funct(func_L);
    m_system->AddLink(m_post_L_linact);

    ChVector<> m1_R = post_R_pos;
    m1_R.z -= 1.0;  // offset marker 1 location 1 meter below marker 2
    m_post_R_linact = std::make_shared<ChLinkLinActuator>();
    m_post_R_linact->SetNameString("R_post_linActuator");
    m_post_R_linact->Initialize(m_chassis, m_post_R, false, ChCoordsys<>(m1_R, QUNIT), ChCoordsys<>(post_R_pos, QUNIT));
    m_post_R_linact->Set_lin_offset(1.0);

    auto func_R = std::make_shared<ChFunction_Const>(0);
    m_post_R_linact->Set_dist_funct(func_R);
    m_system->AddLink(m_post_R_linact);

    // Constrain spindles in a horizontal plane (based on current post location)
    m_post_L_ptPlane = std::make_shared<ChLinkLockPointPlane>();
    m_post_L_ptPlane->SetNameString("L_post_pointPlane");
    m_post_L_ptPlane->Initialize(m_suspension->GetSpindle(LEFT), m_post_L, ChCoordsys<>(spindle_L_pos, QUNIT));
    m_system->AddLink(m_post_L_ptPlane);

    m_post_R_ptPlane = std::make_shared<ChLinkLockPointPlane>();
    m_post_R_ptPlane->SetNameString("R_post_pointPlane");
    m_post_R_ptPlane->Initialize(m_suspension->GetSpindle(RIGHT), m_post_R, ChCoordsys<>(spindle_R_pos, QUNIT));
    m_system->AddLink(m_post_R_ptPlane);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
WheelState ChSuspensionTestRig::GetWheelState(VehicleSide side) const {
    WheelState state;

    state.pos = GetWheelPos(side);
    state.rot = GetWheelRot(side);
    state.lin_vel = GetWheelLinVel(side);
    state.ang_vel = ChVector<>(0, 0, 0);

    ChVector<> ang_vel_loc = state.rot.RotateBack(state.ang_vel);
    state.omega = ang_vel_loc.y;

    return state;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double ChSuspensionTestRig::GetVehicleMass() const {
    double mass = m_suspension->GetMass();
    if (HasSteering())
        mass += m_steering->GetMass();
    for (size_t i = 0; i < m_wheels.size(); i++)
        mass += m_wheels[i]->GetMass();

    return mass;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double ChSuspensionTestRig::GetActuatorDisp(VehicleSide side) {
    double time = GetSystem()->GetChTime();
    if (side == LEFT)
        return m_post_L_linact->Get_dist_funct()->Get_y(time);
    else
        return m_post_R_linact->Get_dist_funct()->Get_y(time);
}

double ChSuspensionTestRig::GetActuatorForce(VehicleSide side) {
    if (side == LEFT)
        return m_post_L_linact->Get_react_force().x;
    else
        return m_post_R_linact->Get_react_force().x;
}

double ChSuspensionTestRig::GetActuatorMarkerDist(VehicleSide side) {
    if (side == LEFT)
        return m_post_L_linact->GetDist();
    else
        return m_post_R_linact->GetDist();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChSuspensionTestRig::Synchronize(double time,
                                      double steering,
                                      double disp_L,
                                      double disp_R,
                                      const TireForces& tire_forces) {
    if (HasSteering()) {
        // Let the steering subsystem process the steering input.
        m_steering->Synchronize(time, steering);
    }

    // Apply the displacements to the left/right post actuators
    if (auto func_L = std::dynamic_pointer_cast<ChFunction_Const>(m_post_L_linact->Get_dist_funct()))
        func_L->Set_yconst(disp_L);
    if (auto func_R = std::dynamic_pointer_cast<ChFunction_Const>(m_post_R_linact->Get_dist_funct()))
        func_R->Set_yconst(disp_R);

    // Apply tire forces to spindle bodies.
    m_suspension->Synchronize(LEFT, tire_forces[0]);
    m_suspension->Synchronize(RIGHT, tire_forces[1]);

    // Cache driver inputs.
    m_steer = steering;
    m_displ_L = disp_L;
    m_displ_R = disp_R;
}

// -----------------------------------------------------------------------------
// Log constraint violations
// -----------------------------------------------------------------------------
void ChSuspensionTestRig::LogConstraintViolations() {
    GetLog().SetNumFormat("%16.4e");

    // Report constraint violations for the suspension joints
    GetLog() << "\n---- LEFT side suspension constraint violations\n\n";
    m_suspension->LogConstraintViolations(LEFT);
    GetLog() << "\n---- RIGHT side suspension constraint violations\n\n";
    m_suspension->LogConstraintViolations(RIGHT);

    // Report constraint violations for the steering joints
    if (HasSteering()) {
        GetLog() << "\n---- STEERING constrain violations\n\n";
        m_steering->LogConstraintViolations();
    }

    GetLog().SetNumFormat("%g");
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChSuspensionTestRig::AddVisualize_post(std::shared_ptr<ChBody> post_body,
                                            std::shared_ptr<ChBody> ground_body,
                                            double height,
                                            double rad,
                                            const ChColor& color) {
    // Platform (on post body)
    auto base_cyl = std::make_shared<ChCylinderShape>();
    base_cyl->GetCylinderGeometry().rad = rad;
    base_cyl->GetCylinderGeometry().p1 = ChVector<>(0, 0, height / 2.0);
    base_cyl->GetCylinderGeometry().p2 = ChVector<>(0, 0, -height / 2.0);
    post_body->AddAsset(base_cyl);

    auto col = std::make_shared<ChColorAsset>();
    col->SetColor(color);
    post_body->AddAsset(col);

    // Piston (on post body)
    auto piston = std::make_shared<ChCylinderShape>();
    piston->GetCylinderGeometry().rad = rad / 6.0;
    piston->GetCylinderGeometry().p1 = ChVector<>(0, 0, -height / 2.0);
    piston->GetCylinderGeometry().p2 = ChVector<>(0, 0, -height * 12.0);
    post_body->AddAsset(piston);  // add asset to post body

    // Post sleve (on chassis/ground body)
    auto cyl = std::make_shared<ChCylinderShape>();
    cyl->GetCylinderGeometry().rad = rad / 4.0;
    cyl->GetCylinderGeometry().p1 = post_body->GetPos() - ChVector<>(0, 0, 8 * height);
    cyl->GetCylinderGeometry().p2 = post_body->GetPos() - ChVector<>(0, 0, 16 * height);
    ground_body->AddAsset(cyl);
}

}  // end namespace vehicle
}  // end namespace chrono
