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

#include "chrono_vehicle/wheeled_vehicle/test_rig/ChSuspensionTestRig.h"

#include "chrono_vehicle/wheeled_vehicle/suspension/DoubleWishbone.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/DoubleWishboneReduced.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/SolidAxle.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/MultiLink.h"

#include "chrono_vehicle/wheeled_vehicle/steering/PitmanArm.h"
#include "chrono_vehicle/wheeled_vehicle/steering/RackPinion.h"

#include "chrono_vehicle/wheeled_vehicle/wheel/Wheel.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_thirdparty/rapidjson/document.h"
#include "chrono_thirdparty/rapidjson/filereadstream.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Defintion of a chassis for a suspension test rig
// -----------------------------------------------------------------------------
class ChSuspensionTestRigChassis : public ChChassis {
  public:
    ChSuspensionTestRigChassis();
    virtual double GetMass() const override { return m_mass; }
    virtual const ChMatrix33<>& GetInertia() const override { return m_inertia; }
    virtual const ChVector<>& GetLocalPosCOM() const override { return m_COM_loc; }
    virtual ChCoordsys<> GetLocalDriverCoordsys() const override { return m_driverCsys; }

  private:
    ChMatrix33<> m_inertia;

    static const double m_mass;
    static const ChVector<> m_inertiaXX;
    static const ChVector<> m_COM_loc;
    static const ChCoordsys<> m_driverCsys;
};

const double ChSuspensionTestRigChassis::m_mass = 1;
const ChVector<> ChSuspensionTestRigChassis::m_inertiaXX(1, 1, 1);
const ChVector<> ChSuspensionTestRigChassis::m_COM_loc(0, 0, 0);
const ChCoordsys<> ChSuspensionTestRigChassis::m_driverCsys(ChVector<>(0, 0, 0), ChQuaternion<>(1, 0, 0, 0));

ChSuspensionTestRigChassis::ChSuspensionTestRigChassis() : ChChassis("Ground") {
    m_inertia = ChMatrix33<>(m_inertiaXX);
}

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
// Static variables
// -----------------------------------------------------------------------------
const double ChSuspensionTestRig::m_post_height = 0.1;
const double ChSuspensionTestRig::m_post_radius = 0.4;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChSuspensionTestRig::LoadSteering(const std::string& filename) {
    FILE* fp = fopen(filename.c_str(), "r");

    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));

    fclose(fp);

    Document d;
    d.ParseStream<ParseFlag::kParseCommentsFlag>(is);

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
    d.ParseStream<ParseFlag::kParseCommentsFlag>(is);

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
    d.ParseStream<ParseFlag::kParseCommentsFlag>(is);

    // Check that the given file is a wheel specification file.
    assert(d.HasMember("Type"));
    std::string type = d["Type"].GetString();
    assert(type.compare("Wheel") == 0);

    // Extract the wheel type.
    assert(d.HasMember("Template"));
    std::string subtype = d["Template"].GetString();

    // Create the wheel using the appropriate template.
    if (subtype.compare("Wheel") == 0) {
        m_wheel[side] = std::make_shared<Wheel>(d);
    }

    GetLog() << "  Loaded JSON: " << filename.c_str() << "\n";
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChSuspensionTestRig::ChSuspensionTestRig(const std::string& filename,
                                         int axle_index,
                                         double displ_limit,
                                         std::shared_ptr<ChTire> tire_left,
                                         std::shared_ptr<ChTire> tire_right,
                                         ChMaterialSurfaceBase::ContactMethod contact_method)
    : ChVehicle(contact_method), m_displ_limit(displ_limit) {
    // Open and parse the input file (vehicle JSON specification file)
    FILE* fp = fopen(filename.c_str(), "r");

    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));

    fclose(fp);

    Document d;
    d.ParseStream<ParseFlag::kParseCommentsFlag>(is);

    // Read top-level data
    assert(d.HasMember("Type"));
    assert(d.HasMember("Template"));
    assert(d.HasMember("Name"));

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

    m_tire[LEFT] = tire_left;
    m_tire[RIGHT] = tire_right;
}

ChSuspensionTestRig::ChSuspensionTestRig(const std::string& filename,
                                         std::shared_ptr<ChTire> tire_left,
                                         std::shared_ptr<ChTire> tire_right,
                                         ChMaterialSurfaceBase::ContactMethod contact_method)
    : ChVehicle(contact_method) {
    // Open and parse the input file (rig JSON specification file)
    FILE* fp = fopen(filename.c_str(), "r");

    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));

    fclose(fp);

    Document d;
    d.ParseStream<ParseFlag::kParseCommentsFlag>(is);

    // Read top-level data
    assert(d.HasMember("Type"));
    assert(d.HasMember("Template"));
    assert(d.HasMember("Name"));

    // Create the suspension and wheel subsystems.
    assert(d.HasMember("Suspension"));

    std::string file_name = d["Suspension"]["Input File"].GetString();
    LoadSuspension(vehicle::GetDataFile(file_name));
    m_suspLoc = loadVector(d["Suspension"]["Location"]);

    file_name = d["Suspension"]["Left Wheel Input File"].GetString();
    LoadWheel(vehicle::GetDataFile(file_name), LEFT);
    file_name = d["Suspension"]["Right Wheel Input File"].GetString();
    LoadWheel(vehicle::GetDataFile(file_name), RIGHT);

    // Read displacement limit
    assert(d.HasMember("Displacement Limit"));
    m_displ_limit = d["Displacement Limit"].GetDouble();

    // Create the steering subsystem, if specified
    if (d.HasMember("Steering")) {
        std::string file_name = d["Steering"]["Input File"].GetString();
        LoadSteering(vehicle::GetDataFile(file_name));
        m_steeringLoc = loadVector(d["Steering"]["Location"]);
        m_steeringRot = loadQuaternion(d["Steering"]["Orientation"]);
    }

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";

    m_tire[LEFT] = tire_left;
    m_tire[RIGHT] = tire_right;
}

void ChSuspensionTestRig::Initialize(const ChCoordsys<>& chassisPos, double chassisFwdVel) {
    // ----------------------------
    // Create the chassis subsystem
    // ----------------------------
    m_chassis = std::make_shared<ChSuspensionTestRigChassis>();
    m_chassis->Initialize(m_system, chassisPos, 0);
    m_chassis->GetBody()->SetBodyFixed(true);

    // ---------------------------------
    // Initialize the vehicle subsystems
    // ---------------------------------

    // Initialize the suspension and steering subsystems.
    if (HasSteering()) {
        m_steering->Initialize(m_chassis->GetBody(), m_steeringLoc, m_steeringRot);
        m_suspension->Initialize(m_chassis->GetBody(), m_suspLoc, m_steering->GetSteeringLink());
    } else {
        m_suspension->Initialize(m_chassis->GetBody(), m_suspLoc, m_chassis->GetBody());
    }

    // Initialize the two wheels
    m_wheel[LEFT]->Initialize(m_suspension->GetSpindle(LEFT));
    m_wheel[RIGHT]->Initialize(m_suspension->GetSpindle(RIGHT));

    // --------------------------------------------
    // Create and initialize the shaker post bodies
    // --------------------------------------------

    // Rotation by 90 degrees about x
    ChMatrix33<> y2z(chrono::Q_from_AngX(CH_C_PI / 2));

    // Left post body (green)
    ChVector<> spindle_L_pos = m_suspension->GetSpindlePos(LEFT);
    ChVector<> post_L_pos = spindle_L_pos;
    post_L_pos.z() -= (m_tire[LEFT]->GetRadius() + m_post_height / 2.0);

    m_post[LEFT] = std::shared_ptr<ChBody>(m_system->NewBody());
    m_post[LEFT]->SetPos(post_L_pos);
    m_post[LEFT]->SetMass(100);
    m_post[LEFT]->SetCollide(true);
    m_system->Add(m_post[LEFT]);
    AddVisualize_post(LEFT, ChColor(0.1f, 0.8f, 0.15f));

    m_post[LEFT]->GetCollisionModel()->ClearModel();
    m_post[LEFT]->GetCollisionModel()->AddCylinder(m_post_radius, m_post_radius, m_post_height / 2, ChVector<>(0), y2z);
    m_post[LEFT]->GetCollisionModel()->BuildModel();

    // Right post body (red)
    ChVector<> spindle_R_pos = m_suspension->GetSpindlePos(RIGHT);
    ChVector<> post_R_pos = spindle_R_pos;
    post_R_pos.z() -= (m_tire[RIGHT]->GetRadius() + m_post_height / 2.0);

    m_post[RIGHT] = std::shared_ptr<ChBody>(m_system->NewBody());
    m_post[RIGHT]->SetPos(post_R_pos);
    m_post[RIGHT]->SetMass(100);
    m_post[RIGHT]->SetCollide(true);
    m_system->Add(m_post[RIGHT]);
    AddVisualize_post(RIGHT, ChColor(0.8f, 0.1f, 0.1f));

    m_post[RIGHT]->GetCollisionModel()->ClearModel();
    m_post[RIGHT]->GetCollisionModel()->AddCylinder(m_post_radius, m_post_radius, m_post_height / 2, ChVector<>(0),
                                                    y2z);
    m_post[RIGHT]->GetCollisionModel()->BuildModel();

    // ------------------------------------------
    // Create and initialize joints and actuators
    // ------------------------------------------

    // Prismatic joints to force vertical translation
    m_post_prismatic[LEFT] = std::make_shared<ChLinkLockPrismatic>();
    m_post_prismatic[LEFT]->SetNameString("L_post_prismatic");
    m_post_prismatic[LEFT]->Initialize(m_chassis->GetBody(), m_post[LEFT], ChCoordsys<>(ChVector<>(post_L_pos), QUNIT));
    m_system->AddLink(m_post_prismatic[LEFT]);

    m_post_prismatic[RIGHT] = std::make_shared<ChLinkLockPrismatic>();
    m_post_prismatic[RIGHT]->SetNameString("R_post_prismatic");
    m_post_prismatic[RIGHT]->Initialize(m_chassis->GetBody(), m_post[RIGHT],
                                        ChCoordsys<>(ChVector<>(post_R_pos), QUNIT));
    m_system->AddLink(m_post_prismatic[RIGHT]);

    // Post actuators
    ChVector<> m1_L = post_L_pos;
    m1_L.z() -= 1.0;  // offset marker 1 location 1 meter below marker 2
    m_post_linact[LEFT] = std::make_shared<ChLinkLinActuator>();
    m_post_linact[LEFT]->SetNameString("L_post_linActuator");
    m_post_linact[LEFT]->Initialize(m_chassis->GetBody(), m_post[LEFT], false, ChCoordsys<>(m1_L, QUNIT),
                                    ChCoordsys<>(post_L_pos, QUNIT));
    m_post_linact[LEFT]->Set_lin_offset(1.0);

    auto func_L = std::make_shared<ChFunction_Const>(0);
    m_post_linact[LEFT]->Set_dist_funct(func_L);
    m_system->AddLink(m_post_linact[LEFT]);

    ChVector<> m1_R = post_R_pos;
    m1_R.z() -= 1.0;  // offset marker 1 location 1 meter below marker 2
    m_post_linact[RIGHT] = std::make_shared<ChLinkLinActuator>();
    m_post_linact[RIGHT]->SetNameString("R_post_linActuator");
    m_post_linact[RIGHT]->Initialize(m_chassis->GetBody(), m_post[RIGHT], false, ChCoordsys<>(m1_R, QUNIT),
                                     ChCoordsys<>(post_R_pos, QUNIT));
    m_post_linact[RIGHT]->Set_lin_offset(1.0);

    auto func_R = std::make_shared<ChFunction_Const>(0);
    m_post_linact[RIGHT]->Set_dist_funct(func_R);
    m_system->AddLink(m_post_linact[RIGHT]);

    // -----------------------------
    // Initialize the tire subsystem
    // -----------------------------

    m_tire[LEFT]->Initialize(GetWheelBody(LEFT), LEFT);
    m_tire[RIGHT]->Initialize(GetWheelBody(RIGHT), RIGHT);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------

void ChSuspensionTestRig::SetSuspensionVisualizationType(VisualizationType vis) {
    m_suspension->SetVisualizationType(vis);
}

void ChSuspensionTestRig::SetSteeringVisualizationType(VisualizationType vis) {
    m_steering->SetVisualizationType(vis);
}

void ChSuspensionTestRig::SetWheelVisualizationType(VisualizationType vis) {
    m_wheel[LEFT]->SetVisualizationType(vis);
    m_wheel[RIGHT]->SetVisualizationType(vis);
}

void ChSuspensionTestRig::SetTireVisualizationType(VisualizationType vis) {
    m_tire[LEFT]->SetVisualizationType(vis);
    m_tire[RIGHT]->SetVisualizationType(vis);
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
    state.omega = ang_vel_loc.y();

    return state;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double ChSuspensionTestRig::GetVehicleMass() const {
    double mass = m_suspension->GetMass();
    if (HasSteering())
        mass += m_steering->GetMass();
    mass += m_wheel[LEFT]->GetMass() + m_wheel[RIGHT]->GetMass();

    return mass;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double ChSuspensionTestRig::GetActuatorDisp(VehicleSide side) {
    double time = GetSystem()->GetChTime();
    return m_post_linact[side]->Get_dist_funct()->Get_y(time);
}

double ChSuspensionTestRig::GetActuatorForce(VehicleSide side) {
    return m_post_linact[side]->Get_react_force().x();
}

double ChSuspensionTestRig::GetActuatorMarkerDist(VehicleSide side) {
    return m_post_linact[side]->GetDist();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChSuspensionTestRig::Synchronize(double time, double steering, double disp_L, double disp_R) {
    // Synchronize the tire subsystems.
    m_tire[LEFT]->Synchronize(time, GetWheelState(LEFT), m_terrain);
    m_tire[RIGHT]->Synchronize(time, GetWheelState(RIGHT), m_terrain);

    // Let the steering subsystem process the steering input.
    if (HasSteering()) {
        m_steering->Synchronize(time, steering);
    }

    // Apply the displacements to the left/right post actuators
    if (auto func_L = std::dynamic_pointer_cast<ChFunction_Const>(m_post_linact[LEFT]->Get_dist_funct()))
        func_L->Set_yconst(disp_L * m_displ_limit);
    if (auto func_R = std::dynamic_pointer_cast<ChFunction_Const>(m_post_linact[RIGHT]->Get_dist_funct()))
        func_R->Set_yconst(disp_R * m_displ_limit);

    // Apply tire forces to spindle bodies.
    m_suspension->Synchronize(LEFT, m_tire[LEFT]->GetTireForce());
    m_suspension->Synchronize(RIGHT, m_tire[RIGHT]->GetTireForce());

    // Udpate the height of the underlying "terrain" object, using the current z positions
    // of the post bodies.
    m_terrain.m_height_L = m_post[LEFT]->GetPos().z() + m_post_height / 2;
    m_terrain.m_height_R = m_post[RIGHT]->GetPos().z() + m_post_height / 2;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChSuspensionTestRig::Advance(double step) {
    // Advance states of tire subsystems
    m_tire[LEFT]->Advance(step);
    m_tire[RIGHT]->Advance(step);

    // Invoke the base class method
    ChVehicle::Advance(step);
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
void ChSuspensionTestRig::AddVisualize_post(VehicleSide side, const ChColor& color) {
    // Platform (on post body)
    auto base_cyl = std::make_shared<ChCylinderShape>();
    base_cyl->GetCylinderGeometry().rad = m_post_radius;
    base_cyl->GetCylinderGeometry().p1 = ChVector<>(0, 0, m_post_height / 2.0);
    base_cyl->GetCylinderGeometry().p2 = ChVector<>(0, 0, -m_post_height / 2.0);
    m_post[side]->AddAsset(base_cyl);

    auto col = std::make_shared<ChColorAsset>();
    col->SetColor(color);
    m_post[side]->AddAsset(col);

    // Piston (on post body)
    auto piston = std::make_shared<ChCylinderShape>();
    piston->GetCylinderGeometry().rad = m_post_radius / 6.0;
    piston->GetCylinderGeometry().p1 = ChVector<>(0, 0, -m_post_height / 2.0);
    piston->GetCylinderGeometry().p2 = ChVector<>(0, 0, -m_post_height * 12.0);
    m_post[side]->AddAsset(piston);  // add asset to post body

    // Post sleve (on chassis/ground body)
    auto cyl = std::make_shared<ChCylinderShape>();
    cyl->GetCylinderGeometry().rad = m_post_radius / 4.0;
    cyl->GetCylinderGeometry().p1 = m_post[side]->GetPos() - ChVector<>(0, 0, 8 * m_post_height);
    cyl->GetCylinderGeometry().p2 = m_post[side]->GetPos() - ChVector<>(0, 0, 16 * m_post_height);
    m_chassis->GetBody()->AddAsset(cyl);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChSuspensionTestRig::Terrain::Terrain() : m_height_L(0), m_height_R(0) {}

double ChSuspensionTestRig::Terrain::GetHeight(double x, double y) const {
    return (y < 0) ? m_height_L : m_height_R;
}

ChVector<> ChSuspensionTestRig::Terrain::GetNormal(double x, double y) const {
    return ChVector<>(0, 0, 1);
}

}  // end namespace vehicle
}  // end namespace chrono
