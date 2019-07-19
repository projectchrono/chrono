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
// Implementation of a suspension testing mechanism (as a vehicle).
// The tested suspension can be specified:
// - through a stand-alone JSON file (may or may not include a steering subsystem)
// - as a specified axle in a vehicle JSON specification file
// - as a specified axle in an existing vehicle (which must have been initialized)
//
// The reference frame follows the ISO standard: Z-axis up, X-axis
// pointing forward, and Y-axis towards the left of the vehicle.
//
// =============================================================================

#include "chrono_vehicle/wheeled_vehicle/test_rig/ChSuspensionTestRig.h"

#include <algorithm>
#include <cstdio>

#include "chrono/assets/ChColorAsset.h"
#include "chrono/assets/ChCylinderShape.h"
//
#include "chrono_vehicle/chassis/ChRigidChassis.h"
//
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
//
#include "chrono_thirdparty/rapidjson/document.h"
#include "chrono_thirdparty/rapidjson/filereadstream.h"
#include "chrono_thirdparty/rapidjson/prettywriter.h"
#include "chrono_thirdparty/rapidjson/stringbuffer.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Definition of a chassis for a suspension test rig
// -----------------------------------------------------------------------------
class ChSuspensionTestRigChassis : public ChRigidChassis {
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

ChSuspensionTestRigChassis::ChSuspensionTestRigChassis() : ChRigidChassis("Ground") {
    m_inertia = ChMatrix33<>(m_inertiaXX);
}

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double ChSuspensionTestRig::m_post_hheight = 0.05;
const double ChSuspensionTestRig::m_post_radius = 0.4;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChSuspensionTestRig::ChSuspensionTestRig(ChWheeledVehicle& vehicle,
                                         int axle_index,
                                         double displ_limit,
                                         std::shared_ptr<ChTire> tire_left,
                                         std::shared_ptr<ChTire> tire_right,
                                         ChMaterialSurface::ContactMethod contact_method)
    : ChVehicle("SuspensionTestRig", contact_method),
      m_displ_limit(displ_limit),
      m_ride_height(-1),
      m_vis_suspension(VisualizationType::PRIMITIVES),
      m_vis_steering(VisualizationType::PRIMITIVES),
      m_vis_wheel(VisualizationType::NONE),
      m_vis_tire(VisualizationType::PRIMITIVES) {
    assert(axle_index >= 0 && axle_index < vehicle.GetNumberAxles());

    // Load suspension subsystem
    m_suspension = vehicle.GetSuspension(axle_index);
    m_suspLoc = m_suspension->GetLocation();

    // Load wheel subsystems
    m_wheel[LEFT] = vehicle.GetWheel(WheelID(axle_index, LEFT));
    m_wheel[RIGHT] = vehicle.GetWheel(WheelID(axle_index, RIGHT));

    // Load steering subsystem (if needed)
    int steering_index = m_suspension->GetSteeringIndex();
    if (steering_index >= 0) {
        m_steering = vehicle.GetSteering(steering_index);
        m_steeringLoc = m_steering->GetPosition().pos;
        m_steeringRot = m_steering->GetPosition().rot;
    }

    m_tire[LEFT] = tire_left;
    m_tire[RIGHT] = tire_right;

    Create();
}

ChSuspensionTestRig::ChSuspensionTestRig(const std::string& filename,
                                         int axle_index,
                                         double displ_limit,
                                         std::shared_ptr<ChTire> tire_left,
                                         std::shared_ptr<ChTire> tire_right,
                                         ChMaterialSurface::ContactMethod contact_method)
    : ChVehicle("SuspensionTestRig", contact_method),
      m_displ_limit(displ_limit),
      m_ride_height(-1),
      m_vis_suspension(VisualizationType::PRIMITIVES),
      m_vis_steering(VisualizationType::PRIMITIVES),
      m_vis_wheel(VisualizationType::NONE),
      m_vis_tire(VisualizationType::PRIMITIVES) {
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
    m_suspension = ReadSuspensionJSON(vehicle::GetDataFile(file_name));
    m_suspLoc = ReadVectorJSON(d["Axles"][axle_index]["Suspension Location"]);

    int steering_index = -1;
    if (d["Axles"][axle_index].HasMember("Steering Index")) {
        steering_index = d["Axles"][axle_index]["Steering Index"].GetInt();
    }

    file_name = d["Axles"][axle_index]["Left Wheel Input File"].GetString();
    m_wheel[VehicleSide::LEFT] = ReadWheelJSON(vehicle::GetDataFile(file_name));
    file_name = d["Axles"][axle_index]["Right Wheel Input File"].GetString();
    m_wheel[VehicleSide::RIGHT] = ReadWheelJSON(vehicle::GetDataFile(file_name));

    // Create the steering subsystem, if needed.
    if (steering_index >= 0) {
        std::string file_name = d["Steering Subsystems"][steering_index]["Input File"].GetString();
        m_steering = ReadSteeringJSON(vehicle::GetDataFile(file_name));
        m_steeringLoc = ReadVectorJSON(d["Steering Subsystems"][steering_index]["Location"]);
        m_steeringRot = ReadQuaternionJSON(d["Steering Subsystems"][steering_index]["Orientation"]);
    }

    // Create the anti-roll bar subsystem, if one exists.
    if (d["Axles"][axle_index].HasMember("Antirollbar Input File")) {
        std::string file_name = d["Axles"][axle_index]["Antirollbar Input File"].GetString();
        m_antirollbar = ReadAntirollbarJSON(vehicle::GetDataFile(file_name));
        m_antirollbarLoc = ReadVectorJSON(d["Axles"][axle_index]["Antirollbar Location"]);
    }

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";

    m_tire[LEFT] = tire_left;
    m_tire[RIGHT] = tire_right;

    Create();
}

ChSuspensionTestRig::ChSuspensionTestRig(const std::string& filename,
                                         std::shared_ptr<ChTire> tire_left,
                                         std::shared_ptr<ChTire> tire_right,
                                         ChMaterialSurface::ContactMethod contact_method)
    : ChVehicle("SuspensionTestRig", contact_method),
      m_vis_suspension(VisualizationType::PRIMITIVES),
      m_vis_steering(VisualizationType::PRIMITIVES),
      m_vis_wheel(VisualizationType::NONE),
      m_vis_tire(VisualizationType::PRIMITIVES) {
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
    m_suspension = ReadSuspensionJSON(vehicle::GetDataFile(file_name));
    m_suspLoc = ReadVectorJSON(d["Suspension"]["Location"]);

    file_name = d["Suspension"]["Left Wheel Input File"].GetString();
    m_wheel[VehicleSide::LEFT] = ReadWheelJSON(vehicle::GetDataFile(file_name));
    file_name = d["Suspension"]["Right Wheel Input File"].GetString();
    m_wheel[VehicleSide::RIGHT] = ReadWheelJSON(vehicle::GetDataFile(file_name));

    // Read initial ride height (if specified)
    if (d.HasMember("Ride Height")) {
        m_ride_height = d["Ride Height"].GetDouble();
    } else {
        m_ride_height = -1;
    }

    // Read displacement limit
    assert(d.HasMember("Displacement Limit"));
    m_displ_limit = d["Displacement Limit"].GetDouble();

    // Create the steering subsystem, if specified
    if (d.HasMember("Steering")) {
        std::string file_name = d["Steering"]["Input File"].GetString();
        m_steering = ReadSteeringJSON(vehicle::GetDataFile(file_name));
        m_steeringLoc = ReadVectorJSON(d["Steering"]["Location"]);
        m_steeringRot = ReadQuaternionJSON(d["Steering"]["Orientation"]);
    }

    // Create the anti-roll bar subsystem, if one exists.
    if (d["Suspension"].HasMember("Antirollbar Input File")) {
        std::string file_name = d["Suspension"]["Antirollbar Input File"].GetString();
        m_antirollbar = ReadAntirollbarJSON(vehicle::GetDataFile(file_name));
        m_antirollbarLoc = ReadVectorJSON(d["Suspension"]["Antirollbar Location"]);
    }

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";

    m_tire[LEFT] = tire_left;
    m_tire[RIGHT] = tire_right;

    Create();
}

void ChSuspensionTestRig::Create() {
    // ----------------------------
    // Create the chassis subsystem
    // ----------------------------
    m_chassis = std::make_shared<ChSuspensionTestRigChassis>();
    m_chassis->Initialize(m_system, ChCoordsys<>(), 0);
    m_chassis->SetFixed(true);

    // ---------------------------------
    // Initialize the vehicle subsystems
    // ---------------------------------

    // Initialize the suspension and steering subsystems.
    if (HasSteering()) {
        m_steering->Initialize(m_chassis->GetBody(), m_steeringLoc, m_steeringRot);
        m_suspension->Initialize(m_chassis->GetBody(), m_suspLoc, m_steering->GetSteeringLink(), 0);
    } else {
        m_suspension->Initialize(m_chassis->GetBody(), m_suspLoc, m_chassis->GetBody(), -1);
    }

    // Initialize the antirollbar subsystem.
    if (HasAntirollbar()) {
        m_antirollbar->Initialize(m_chassis->GetBody(), m_antirollbarLoc, m_suspension->GetLeftBody(),
                                  m_suspension->GetRightBody());
    }

    // Initialize the two wheels
    m_wheel[LEFT]->Initialize(m_suspension->GetSpindle(LEFT));
    m_wheel[RIGHT]->Initialize(m_suspension->GetSpindle(RIGHT));

    // Initialize the tire subsystem
    m_tire[LEFT]->Initialize(GetWheelBody(LEFT), LEFT);
    m_tire[RIGHT]->Initialize(GetWheelBody(RIGHT), RIGHT);

    // --------------------------------------------
    // Imobilize wheels
    // --------------------------------------------

    // --------------------------------------------
    // Create and initialize the shaker post bodies
    // --------------------------------------------

    // Left post body (green)
    ChVector<> spindle_L_pos = m_suspension->GetSpindlePos(LEFT);
    ChVector<> post_L_pos = spindle_L_pos - ChVector<>(0, 0, m_tire[LEFT]->GetRadius());

    m_post[LEFT] = std::shared_ptr<ChBody>(m_system->NewBody());
    m_post[LEFT]->SetPos(post_L_pos);
    m_post[LEFT]->SetMass(100);
    m_post[LEFT]->SetCollide(true);
    m_system->Add(m_post[LEFT]);
    AddVisualize_post(LEFT, ChColor(0.1f, 0.8f, 0.15f));

    m_post[LEFT]->GetCollisionModel()->ClearModel();
    m_post[LEFT]->GetCollisionModel()->AddCylinder(m_post_radius, m_post_radius, m_post_hheight,
                                                   ChVector<>(0, 0, -m_post_hheight),
                                                   ChMatrix33<>(Q_from_AngX(CH_C_PI / 2)));
    m_post[LEFT]->GetCollisionModel()->BuildModel();

    // Right post body (red)
    ChVector<> spindle_R_pos = m_suspension->GetSpindlePos(RIGHT);
    ChVector<> post_R_pos = spindle_R_pos - ChVector<>(0, 0, m_tire[RIGHT]->GetRadius());

    m_post[RIGHT] = std::shared_ptr<ChBody>(m_system->NewBody());
    m_post[RIGHT]->SetPos(post_R_pos);
    m_post[RIGHT]->SetMass(100);
    m_post[RIGHT]->SetCollide(true);
    m_system->Add(m_post[RIGHT]);
    AddVisualize_post(RIGHT, ChColor(0.8f, 0.1f, 0.1f));

    m_post[RIGHT]->GetCollisionModel()->ClearModel();
    m_post[RIGHT]->GetCollisionModel()->AddCylinder(m_post_radius, m_post_radius, m_post_hheight,
                                                    ChVector<>(0, 0, -m_post_hheight),
                                                    ChMatrix33<>(Q_from_AngX(CH_C_PI / 2)));
    m_post[RIGHT]->GetCollisionModel()->BuildModel();

    // ------------------------------------------
    // Create and initialize joints and actuators
    // ------------------------------------------

    auto func_L = std::make_shared<ChFunction_Const>();
    auto func_R = std::make_shared<ChFunction_Const>();

    m_post_linact[LEFT] = std::make_shared<ChLinkMotorLinearPosition>();
    m_post_linact[LEFT]->SetNameString("L_post_linActuator");
    m_post_linact[LEFT]->SetMotionFunction(func_L);
    m_post_linact[LEFT]->Initialize(m_chassis->GetBody(), m_post[LEFT],
                                    ChFrame<>(ChVector<>(post_L_pos), Q_from_AngY(CH_C_PI_2)));
    m_system->AddLink(m_post_linact[LEFT]);

    m_post_linact[RIGHT] = std::make_shared<ChLinkMotorLinearPosition>();
    m_post_linact[RIGHT]->SetNameString("R_post_linActuator");
    m_post_linact[RIGHT]->SetMotionFunction(func_R);
    m_post_linact[RIGHT]->Initialize(m_chassis->GetBody(), m_post[RIGHT],
                                     ChFrame<>(ChVector<>(post_R_pos), Q_from_AngY(CH_C_PI_2)));
    m_system->AddLink(m_post_linact[RIGHT]);
}

void ChSuspensionTestRig::Initialize() {
    if (!m_driver) {
        throw ChException("No driver system provided");
    }

    // Calculate post displacement offset (if any) to set reference position at specified ride height
    m_displ_offset = 0;
    m_displ_delay = 0;
    if (m_ride_height > 0) {
        m_displ_offset = -m_ride_height - m_post[LEFT]->GetPos().z();
        m_displ_delay = 0.1;
    }

    // Set visualization modes
    m_suspension->SetVisualizationType(m_vis_suspension);
    m_steering->SetVisualizationType(m_vis_steering);
    m_wheel[LEFT]->SetVisualizationType(m_vis_wheel);
    m_wheel[RIGHT]->SetVisualizationType(m_vis_wheel);
    m_tire[LEFT]->SetVisualizationType(m_vis_tire);
    m_tire[RIGHT]->SetVisualizationType(m_vis_tire);

    // Initialize the driver system
    m_driver->SetTimeDelay(m_displ_delay);
    m_driver->Initialize();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChSuspensionTestRig::SetDriver(std::unique_ptr<ChDriverSTR> driver) {
    m_driver = std::move(driver);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double ChSuspensionTestRig::GetWheelOmega(VehicleSide side) const {
    auto rot = GetWheelRot(side);
    auto ang_vel = GetWheelAngVel(side);
    auto ang_vel_loc = rot.RotateBack(ang_vel);
    return ang_vel_loc.y();
}

WheelState ChSuspensionTestRig::GetWheelState(VehicleSide side) const {
    WheelState state;

    state.pos = GetWheelPos(side);
    state.rot = GetWheelRot(side);
    state.lin_vel = GetWheelLinVel(side);
    state.ang_vel = GetWheelAngVel(side);

    ChVector<> ang_vel_loc = state.rot.RotateBack(state.ang_vel);
    state.omega = ang_vel_loc.y();

    return state;
}

double ChSuspensionTestRig::GetMass() const {
    double mass = m_suspension->GetMass();
    if (HasSteering())
        mass += m_steering->GetMass();
    mass += m_wheel[LEFT]->GetMass() + m_wheel[RIGHT]->GetMass();

    return mass;
}

double ChSuspensionTestRig::GetActuatorDisp(VehicleSide side) {
    double time = GetSystem()->GetChTime();
    return m_post_linact[side]->GetMotionFunction()->Get_y(time);
}

double ChSuspensionTestRig::GetActuatorForce(VehicleSide side) {
    return m_post_linact[side]->Get_react_force().x();
}

double ChSuspensionTestRig::GetActuatorMarkerDist(VehicleSide side) {
    return m_post_linact[side]->GetMotorPos();
}

double ChSuspensionTestRig::GetRideHeight() const {
    // Note: the chassis reference frame is constructed at a height of 0.
    return -(m_post[LEFT]->GetPos().z() + m_post[RIGHT]->GetPos().z()) / 2;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChSuspensionTestRig::Advance(double step) {
    double time = GetChTime();

    // Set post displacements
    double displ_left = 0;
    double displ_right = 0;

    if (time < m_displ_delay) {
        m_steering_input = 0;
        m_left_input = 0;
        m_right_input = 0;
        displ_left = m_displ_offset * time / m_displ_delay;
        displ_right = m_displ_offset * time / m_displ_delay;
    } else {
        m_steering_input = m_driver->GetSteering();
        m_left_input = m_driver->GetDisplacementLeft();
        m_right_input = m_driver->GetDisplacementRight();

        displ_left = m_displ_offset + m_displ_limit * m_left_input;
        displ_right = m_displ_offset + m_displ_limit * m_right_input;
    }

    m_tireforce[LEFT] = m_tire[LEFT]->ReportTireForce(&m_terrain);
    m_tireforce[RIGHT] = m_tire[RIGHT]->ReportTireForce(&m_terrain);

    auto tire_force_L = m_tire[LEFT]->GetTireForce();
    auto tire_force_R = m_tire[RIGHT]->GetTireForce();

    auto wheel_state_L = GetWheelState(LEFT);
    auto wheel_state_R = GetWheelState(RIGHT);

    // Synchronize driver system
    m_driver->Synchronize(time);

    // Synchronize the tire subsystems
    m_tire[LEFT]->Synchronize(time, wheel_state_L, m_terrain);
    m_tire[RIGHT]->Synchronize(time, wheel_state_R, m_terrain);

    // Let the steering subsystem process the steering input
    if (HasSteering()) {
        m_steering->Synchronize(time, m_steering_input);
    }

    // Apply tire forces to spindle bodies
    m_suspension->Synchronize(LEFT, tire_force_L);
    m_suspension->Synchronize(RIGHT, tire_force_R);

    // Update post displacements
    auto func_L = std::static_pointer_cast<ChFunction_Const>(m_post_linact[LEFT]->GetMotionFunction());
    auto func_R = std::static_pointer_cast<ChFunction_Const>(m_post_linact[RIGHT]->GetMotionFunction());
    func_L->Set_yconst(displ_left);
    func_R->Set_yconst(displ_right);

    // Update the height of the underlying terrain object, using the current z positions of the post bodies.
    m_terrain.m_height_L = m_post[LEFT]->GetPos().z();
    m_terrain.m_height_R = m_post[RIGHT]->GetPos().z();

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
    base_cyl->GetCylinderGeometry().p1 = ChVector<>(0, 0, 0);
    base_cyl->GetCylinderGeometry().p2 = ChVector<>(0, 0, -2 * m_post_hheight);
    m_post[side]->AddAsset(base_cyl);

    auto col = std::make_shared<ChColorAsset>();
    col->SetColor(color);
    m_post[side]->AddAsset(col);

    // Piston (on post body)
    auto piston = std::make_shared<ChCylinderShape>();
    piston->GetCylinderGeometry().rad = m_post_radius / 6.0;
    piston->GetCylinderGeometry().p1 = ChVector<>(0, 0, -2 * m_post_hheight);
    piston->GetCylinderGeometry().p2 = ChVector<>(0, 0, -m_post_hheight * 20.0);
    m_post[side]->AddAsset(piston);

    // Post sleeve (on chassis/ground body)
    auto cyl = std::make_shared<ChCylinderShape>();
    cyl->GetCylinderGeometry().rad = m_post_radius / 4.0;
    cyl->GetCylinderGeometry().p1 = m_post[side]->GetPos() - ChVector<>(0, 0, 16 * m_post_hheight);
    cyl->GetCylinderGeometry().p2 = m_post[side]->GetPos() - ChVector<>(0, 0, 32 * m_post_hheight);
    m_chassis->GetBody()->AddAsset(cyl);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChSuspensionTestRig::Terrain::Terrain() : m_height_L(0), m_height_R(0) {}

double ChSuspensionTestRig::Terrain::GetHeight(double x, double y) const {
    return (y < 0) ? m_height_R : m_height_L;
}

ChVector<> ChSuspensionTestRig::Terrain::GetNormal(double x, double y) const {
    return ChVector<>(0, 0, 1);
}

float ChSuspensionTestRig::Terrain::GetCoefficientFriction(double x, double y) const {
    return 0.8f;
}

}  // end namespace vehicle
}  // end namespace chrono
