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

//// RADU
//// Todo: what do we want to do for axles with double wheels?

#include "chrono_vehicle/wheeled_vehicle/test_rig/ChSuspensionTestRig.h"

#include <algorithm>
#include <cstdio>

#include "chrono/assets/ChColorAsset.h"
#include "chrono/assets/ChCylinderShape.h"

#include "chrono_vehicle/chassis/ChRigidChassis.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

#include "chrono_thirdparty/rapidjson/prettywriter.h"
#include "chrono_thirdparty/rapidjson/stringbuffer.h"

#ifdef CHRONO_POSTPROCESS
#include "chrono_postprocess/ChGnuPlot.h"
#endif

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// =============================================================================
// Definition of a chassis for a suspension test rig
class TestRigChassis : public ChRigidChassis {
  public:
    TestRigChassis();
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

const double TestRigChassis::m_mass = 1;
const ChVector<> TestRigChassis::m_inertiaXX(1, 1, 1);
const ChVector<> TestRigChassis::m_COM_loc(0, 0, 0);
const ChCoordsys<> TestRigChassis::m_driverCsys(ChVector<>(0, 0, 0), ChQuaternion<>(1, 0, 0, 0));

TestRigChassis::TestRigChassis() : ChRigidChassis("Ground") {
    m_inertia = ChMatrix33<>(m_inertiaXX);
}

// =============================================================================
// Definition of a terrain object for use by a suspension test rig.
// Note that this assumes an ISO world frame.
class TestRigTerrain : public ChTerrain {
  public:
    TestRigTerrain();
    virtual double GetHeight(const ChVector<>& loc) const override;
    virtual ChVector<> GetNormal(const ChVector<>& loc) const override;
    virtual float GetCoefficientFriction(const ChVector<>& loc) const override;
    double m_height_L;
    double m_height_R;
};

TestRigTerrain::TestRigTerrain() : m_height_L(-1000), m_height_R(-1000) {}

double TestRigTerrain::GetHeight(const ChVector<>& loc) const {
    return (loc.y() < 0) ? m_height_R : m_height_L;
}

ChVector<> TestRigTerrain::GetNormal(const ChVector<>& loc) const {
    return ChVector<>(0, 0, 1);
}

float TestRigTerrain::GetCoefficientFriction(const ChVector<>& loc) const {
    return 0.8f;
}

// =============================================================================
// Static variables
const double ChSuspensionTestRigPlatform::m_post_hheight = 0.05;
const double ChSuspensionTestRigPlatform::m_post_radius = 0.4;

const double ChSuspensionTestRigPushrod::m_rod_length = 3;
const double ChSuspensionTestRigPushrod::m_rod_radius = 0.02;

// =============================================================================
// Base class implementation
// =============================================================================
ChSuspensionTestRig::ChSuspensionTestRig(ChWheeledVehicle& vehicle,
                                         int axle_index,
                                         double displ_limit,
                                         std::shared_ptr<ChTire> tire_left,
                                         std::shared_ptr<ChTire> tire_right,
                                         ChContactMethod contact_method)
    : ChVehicle("SuspensionTestRig", contact_method),
      m_displ_limit(displ_limit),
      m_ride_height(-1),
      m_vis_suspension(VisualizationType::PRIMITIVES),
      m_vis_steering(VisualizationType::PRIMITIVES),
      m_vis_wheel(VisualizationType::NONE),
      m_vis_tire(VisualizationType::PRIMITIVES),
      m_plot_output(false),
      m_plot_output_step(0),
      m_next_plot_output_time(0),
      m_csv(nullptr) {
    assert(axle_index >= 0 && axle_index < vehicle.GetNumberAxles());

    // Load suspension subsystem
    m_suspension = vehicle.GetSuspension(axle_index);
    m_suspLoc = m_suspension->GetLocation();

    // Load wheel subsystems
    m_wheel[LEFT] = vehicle.GetAxle(axle_index)->m_wheels[0];
    m_wheel[RIGHT] = vehicle.GetAxle(axle_index)->m_wheels[1];

    // Load steering subsystem (if needed)
    int steering_index = m_suspension->GetSteeringIndex();
    if (steering_index >= 0) {
        m_steering = vehicle.GetSteering(steering_index);
        m_steeringLoc = m_steering->GetPosition().pos;
        m_steeringRot = m_steering->GetPosition().rot;
    }

    m_tire[LEFT] = tire_left;
    m_tire[RIGHT] = tire_right;
}

ChSuspensionTestRig::ChSuspensionTestRig(const std::string& filename,
                                         int axle_index,
                                         double displ_limit,
                                         std::shared_ptr<ChTire> tire_left,
                                         std::shared_ptr<ChTire> tire_right,
                                         ChContactMethod contact_method)
    : ChVehicle("SuspensionTestRig", contact_method),
      m_displ_limit(displ_limit),
      m_ride_height(-1),
      m_vis_suspension(VisualizationType::PRIMITIVES),
      m_vis_steering(VisualizationType::PRIMITIVES),
      m_vis_wheel(VisualizationType::NONE),
      m_vis_tire(VisualizationType::PRIMITIVES),
      m_plot_output(false),
      m_plot_output_step(0),
      m_next_plot_output_time(0),
      m_csv(nullptr) {
    // Open and parse the input file (vehicle JSON specification file)
    Document d = ReadFileJSON(filename);
    if (d.IsNull())
        return;

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
}

ChSuspensionTestRig::ChSuspensionTestRig(const std::string& filename,
                                         std::shared_ptr<ChTire> tire_left,
                                         std::shared_ptr<ChTire> tire_right,
                                         ChContactMethod contact_method)
    : ChVehicle("SuspensionTestRig", contact_method),
      m_vis_suspension(VisualizationType::PRIMITIVES),
      m_vis_steering(VisualizationType::PRIMITIVES),
      m_vis_wheel(VisualizationType::NONE),
      m_vis_tire(VisualizationType::PRIMITIVES),
      m_plot_output(false),
      m_plot_output_step(0),
      m_next_plot_output_time(0),
      m_csv(nullptr) {
    // Open and parse the input file (rig JSON specification file)
    Document d = ReadFileJSON(filename);
    if (d.IsNull())
        return;

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
}

ChSuspensionTestRig::~ChSuspensionTestRig() {
    delete m_csv;
}

void ChSuspensionTestRig::InitializeSubsystems() {
    // Create the chassis subsystem
    m_chassis = chrono_types::make_shared<TestRigChassis>();
    m_chassis->Initialize(m_system, ChCoordsys<>(), 0);
    m_chassis->SetFixed(true);

    // Create the terrain system
    m_terrain = std::unique_ptr<ChTerrain>(new TestRigTerrain);

    // Initialize the suspension and steering subsystems.
    if (m_steering) {
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
    m_wheel[LEFT]->Initialize(m_suspension->GetSpindle(LEFT), LEFT);
    m_wheel[RIGHT]->Initialize(m_suspension->GetSpindle(RIGHT), RIGHT);

    // Associate tires to wheels
    m_wheel[LEFT]->SetTire(m_tire[LEFT]);
    m_wheel[RIGHT]->SetTire(m_tire[RIGHT]);

    // Initialize the tire subsystem
    m_tire[LEFT]->Initialize(m_wheel[LEFT]);
    m_tire[RIGHT]->Initialize(m_wheel[RIGHT]);

    // Imobilize wheels
}

void ChSuspensionTestRig::Initialize() {
    if (!m_driver) {
        throw ChException("No driver system provided");
    }

    // Initialize reference spindle vertical positions at design configuration.
    m_spindle_ref[LEFT] = m_suspension->GetSpindlePos(LEFT).z();
    m_spindle_ref[RIGHT] = m_suspension->GetSpindlePos(RIGHT).z();

    // Calculate displacement offset to set rig at specified ride height (if any).
    // The rig will be moved dynamically to this configuration over a time interval m_displ_delay.
    m_displ_offset = 0;
    m_displ_delay = 0;
    if (m_ride_height > 0) {
        m_displ_offset = CalcDisplacementOffset();
        m_displ_delay = 0.1;
    }

    // Set visualization modes
    m_suspension->SetVisualizationType(m_vis_suspension);
    if (m_steering)
        m_steering->SetVisualizationType(m_vis_steering);
    m_wheel[LEFT]->SetVisualizationType(m_vis_wheel);
    m_wheel[RIGHT]->SetVisualizationType(m_vis_wheel);
    m_tire[LEFT]->SetVisualizationType(m_vis_tire);
    m_tire[RIGHT]->SetVisualizationType(m_vis_tire);

    // Initialize the driver system
    m_driver->m_delay = m_displ_delay;
    m_driver->Initialize();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChSuspensionTestRig::SetDriver(std::shared_ptr<ChDriverSTR> driver) {
    m_driver = driver;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double ChSuspensionTestRig::GetMass() const {
    // Note: do not include mass of the wheels, as these are already accounted for in suspension.
    double mass = m_suspension->GetMass();
    if (m_steering)
        mass += m_steering->GetMass();

    return mass;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
TerrainForce ChSuspensionTestRig::ReportTireForce(VehicleSide side) const {
    return m_tire[side]->ReportTireForce(m_terrain.get());
}

ChSuspension::Force ChSuspensionTestRig::ReportSuspensionForce(VehicleSide side) const {
    return m_suspension->ReportSuspensionForce(side);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double ChSuspensionTestRig::GetWheelTravel(VehicleSide side) const {
    if (GetChTime() < m_displ_delay)
        return 0;
    return m_suspension->GetSpindlePos(side).z() - m_spindle_ref[side];
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChSuspensionTestRig::Advance(double step) {
    double time = GetChTime();

    // Set actuator displacements
    double displ_left = 0;
    double displ_right = 0;
    double displ_speed_left = 0;
    double displ_speed_right = 0;

    if (time < m_displ_delay) {
        // Automatic phase to bring rig at specified initial ride height
        m_steering_input = 0;
        m_left_input = 0;
        m_right_input = 0;
        displ_left = m_displ_offset * time / m_displ_delay;
        displ_right = m_displ_offset * time / m_displ_delay;

        // Update spindle vertical reference positions
        m_spindle_ref[LEFT] = m_suspension->GetSpindlePos(LEFT).z();
        m_spindle_ref[RIGHT] = m_suspension->GetSpindlePos(RIGHT).z();
    } else {
        // Use actual driver inputs to set current actuator displacements
        m_steering_input = m_driver->GetSteering();
        m_left_input = m_driver->GetDisplacementLeft();
        m_right_input = m_driver->GetDisplacementRight();
        double left_input_speed = m_driver->GetDisplacementSpeedLeft();
        double right_input_speed = m_driver->GetDisplacementSpeedRight();

        displ_left = m_displ_offset + m_displ_limit * m_left_input;
        displ_right = m_displ_offset + m_displ_limit * m_right_input;
        displ_speed_left = m_displ_limit * left_input_speed;
        displ_speed_right = m_displ_limit * right_input_speed;
    }

    // Synchronize driver system
    m_driver->Synchronize(time);

    // Synchronize the tire subsystems
    m_tire[LEFT]->Synchronize(time, *m_terrain);
    m_tire[RIGHT]->Synchronize(time, *m_terrain);

    // Let the steering subsystem process the steering input
    if (m_steering) {
        m_steering->Synchronize(time, m_steering_input);
    }

    // Prepare suspension for accepting tire forces
    m_suspension->Synchronize();

    // Synchronize wheels (this applies tire forces to spindle bodies)
    for (auto wheel : m_wheel) {
        wheel->Synchronize();
    }

    // Update actuators
    UpdateActuators(displ_left, displ_speed_left, displ_right, displ_speed_right);

    // Update the height of the underlying terrain object, using the current z positions of the post bodies.
    static_cast<TestRigTerrain*>(m_terrain.get())->m_height_L = CalcTerrainHeight(LEFT);
    static_cast<TestRigTerrain*>(m_terrain.get())->m_height_R = CalcTerrainHeight(RIGHT);

    // Advance states of tire subsystems
    m_tire[LEFT]->Advance(step);
    m_tire[RIGHT]->Advance(step);

    // Invoke the base class method
    ChVehicle::Advance(step);

    // Generate output for plotting if requested
    time = GetChTime();
    if (!m_driver->Started()) {
        m_next_plot_output_time = time + m_plot_output_step;
    } else if (m_plot_output && time > m_next_plot_output_time) {
        CollectPlotData(time);
        m_next_plot_output_time += m_plot_output_step;
    }
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
    if (m_steering) {
        GetLog() << "\n---- STEERING constrain violations\n\n";
        m_steering->LogConstraintViolations();
    }

    GetLog().SetNumFormat("%g");
}

// -----------------------------------------------------------------------------

void ChSuspensionTestRig::SetSuspensionOutput(bool state) {
    m_suspension->SetOutput(state);
}

void ChSuspensionTestRig::SetSteeringOutput(bool state) {
    if (m_steering)
        m_steering->SetOutput(state);
}

void ChSuspensionTestRig::SetAntirollbarOutput(bool state) {
    if (m_antirollbar)
        m_antirollbar->SetOutput(state);
}

void ChSuspensionTestRig::Output(int frame, ChVehicleOutput& database) const {
    database.WriteTime(frame, m_system->GetChTime());

    if (m_suspension->OutputEnabled()) {
        database.WriteSection(m_suspension->GetName());
        m_suspension->Output(database);
    }

    if (m_steering && m_steering->OutputEnabled()) {
        database.WriteSection(m_steering->GetName());
        m_steering->Output(database);
    }

    if (m_antirollbar && m_antirollbar->OutputEnabled()) {
        database.WriteSection(m_antirollbar->GetName());
        m_antirollbar->Output(database);
    }
}

void ChSuspensionTestRig::SetPlotOutput(double output_step) {
    m_plot_output = true;
    m_plot_output_step = output_step;
    m_csv = new utils::CSV_writer(" ");
}

// =============================================================================
// ChSuspensionTestRigPlatform class implementation
// =============================================================================
ChSuspensionTestRigPlatform::ChSuspensionTestRigPlatform(ChWheeledVehicle& vehicle,  // vehicle source
                                                         int axle_index,             // index of test suspension
                                                         double displ_limit,         // limits for post displacement
                                                         std::shared_ptr<ChTire> tire_left,   // left tire
                                                         std::shared_ptr<ChTire> tire_right,  // right tire
                                                         ChContactMethod contact_method       // contact method
                                                         )
    : ChSuspensionTestRig(vehicle, axle_index, displ_limit, tire_left, tire_right, contact_method) {
    InitializeSubsystems();
    Create();
}

ChSuspensionTestRigPlatform::ChSuspensionTestRigPlatform(
    const std::string& filename,         // JSON file with vehicle specification
    int axle_index,                      // index of the suspension to be tested
    double displ_limit,                  // limits for post displacement
    std::shared_ptr<ChTire> tire_left,   // left tire
    std::shared_ptr<ChTire> tire_right,  // right tire
    ChContactMethod contact_method       // contact method
    )
    : ChSuspensionTestRig(filename, axle_index, displ_limit, tire_left, tire_right, contact_method) {
    InitializeSubsystems();
    Create();
}

ChSuspensionTestRigPlatform::ChSuspensionTestRigPlatform(
    const std::string& filename,         // JSON file with test rig specification
    std::shared_ptr<ChTire> tire_left,   // left tire
    std::shared_ptr<ChTire> tire_right,  // right tire
    ChContactMethod contact_method       // contact method
    )
    : ChSuspensionTestRig(filename, tire_left, tire_right, contact_method) {
    InitializeSubsystems();
    Create();
}

void ChSuspensionTestRigPlatform::Create() {
    // Create a contact material for the two posts (shared)
    //// TODO: are default material properties ok?
    MaterialInfo minfo;
    auto post_mat = minfo.CreateMaterial(m_system->GetContactMethod());

    // Create the left post body (green)
    ChVector<> spindle_L_pos = m_suspension->GetSpindlePos(LEFT);
    ChVector<> post_L_pos = spindle_L_pos - ChVector<>(0, 0, m_tire[LEFT]->GetRadius());

    m_post[LEFT] = std::shared_ptr<ChBody>(m_system->NewBody());
    m_post[LEFT]->SetPos(post_L_pos);
    m_post[LEFT]->SetMass(100);
    m_post[LEFT]->SetCollide(true);
    m_system->Add(m_post[LEFT]);
    AddPostVisualization(LEFT, ChColor(0.1f, 0.8f, 0.15f));

    m_post[LEFT]->GetCollisionModel()->ClearModel();
    m_post[LEFT]->GetCollisionModel()->AddCylinder(post_mat, m_post_radius, m_post_radius, m_post_hheight,
                                                   ChVector<>(0, 0, -m_post_hheight),
                                                   ChMatrix33<>(Q_from_AngX(CH_C_PI / 2)));
    m_post[LEFT]->GetCollisionModel()->BuildModel();

    // Create the right post body (red)
    ChVector<> spindle_R_pos = m_suspension->GetSpindlePos(RIGHT);
    ChVector<> post_R_pos = spindle_R_pos - ChVector<>(0, 0, m_tire[RIGHT]->GetRadius());

    m_post[RIGHT] = std::shared_ptr<ChBody>(m_system->NewBody());
    m_post[RIGHT]->SetPos(post_R_pos);
    m_post[RIGHT]->SetMass(100);
    m_post[RIGHT]->SetCollide(true);
    m_system->Add(m_post[RIGHT]);
    AddPostVisualization(RIGHT, ChColor(0.8f, 0.1f, 0.1f));

    m_post[RIGHT]->GetCollisionModel()->ClearModel();
    m_post[RIGHT]->GetCollisionModel()->AddCylinder(post_mat, m_post_radius, m_post_radius, m_post_hheight,
                                                    ChVector<>(0, 0, -m_post_hheight),
                                                    ChMatrix33<>(Q_from_AngX(CH_C_PI / 2)));
    m_post[RIGHT]->GetCollisionModel()->BuildModel();

    // Create and initialize joints and actuators
    auto func_L = chrono_types::make_shared<ChFunction_Setpoint>();
    auto func_R = chrono_types::make_shared<ChFunction_Setpoint>();

    m_post_linact[LEFT] = chrono_types::make_shared<ChLinkMotorLinearPosition>();
    m_post_linact[LEFT]->SetNameString("L_post_linActuator");
    m_post_linact[LEFT]->SetMotionFunction(func_L);
    m_post_linact[LEFT]->Initialize(m_chassis->GetBody(), m_post[LEFT],
                                    ChFrame<>(ChVector<>(post_L_pos), Q_from_AngY(CH_C_PI_2)));
    m_system->AddLink(m_post_linact[LEFT]);

    m_post_linact[RIGHT] = chrono_types::make_shared<ChLinkMotorLinearPosition>();
    m_post_linact[RIGHT]->SetNameString("R_post_linActuator");
    m_post_linact[RIGHT]->SetMotionFunction(func_R);
    m_post_linact[RIGHT]->Initialize(m_chassis->GetBody(), m_post[RIGHT],
                                     ChFrame<>(ChVector<>(post_R_pos), Q_from_AngY(CH_C_PI_2)));
    m_system->AddLink(m_post_linact[RIGHT]);
}

void ChSuspensionTestRigPlatform::AddPostVisualization(VehicleSide side, const ChColor& color) {
    // Platform (on post body)
    auto base_cyl = chrono_types::make_shared<ChCylinderShape>();
    base_cyl->GetCylinderGeometry().rad = m_post_radius;
    base_cyl->GetCylinderGeometry().p1 = ChVector<>(0, 0, 0);
    base_cyl->GetCylinderGeometry().p2 = ChVector<>(0, 0, -2 * m_post_hheight);
    m_post[side]->AddAsset(base_cyl);

    auto col = chrono_types::make_shared<ChColorAsset>();
    col->SetColor(color);
    m_post[side]->AddAsset(col);

    // Piston (on post body)
    auto piston = chrono_types::make_shared<ChCylinderShape>();
    piston->GetCylinderGeometry().rad = m_post_radius / 6.0;
    piston->GetCylinderGeometry().p1 = ChVector<>(0, 0, -2 * m_post_hheight);
    piston->GetCylinderGeometry().p2 = ChVector<>(0, 0, -m_post_hheight * 20.0);
    m_post[side]->AddAsset(piston);

    // Post sleeve (on chassis/ground body)
    auto cyl = chrono_types::make_shared<ChCylinderShape>();
    cyl->GetCylinderGeometry().rad = m_post_radius / 4.0;
    cyl->GetCylinderGeometry().p1 = m_post[side]->GetPos() - ChVector<>(0, 0, 16 * m_post_hheight);
    cyl->GetCylinderGeometry().p2 = m_post[side]->GetPos() - ChVector<>(0, 0, 32 * m_post_hheight);
    m_chassis->GetBody()->AddAsset(cyl);
}

double ChSuspensionTestRigPlatform::CalcDisplacementOffset() {
    return -m_ride_height - m_post[LEFT]->GetPos().z();
}

double ChSuspensionTestRigPlatform::CalcTerrainHeight(VehicleSide side) {
    // Update the height of the underlying terrain object, using the current z positions of the post bodies.
    return m_post[side]->GetPos().z();
}

void ChSuspensionTestRigPlatform::UpdateActuators(double displ_left,
                                                  double displ_speed_left,
                                                  double displ_right,
                                                  double displ_speed_right) {
    double time = GetSystem()->GetChTime();
    auto func_L = std::static_pointer_cast<ChFunction_Setpoint>(m_post_linact[LEFT]->GetMotionFunction());
    auto func_R = std::static_pointer_cast<ChFunction_Setpoint>(m_post_linact[RIGHT]->GetMotionFunction());
    func_L->SetSetpointAndDerivatives(displ_left, displ_speed_left, 0.0);
    func_R->SetSetpointAndDerivatives(displ_right, displ_speed_right, 0.0);
}

double ChSuspensionTestRigPlatform::GetActuatorDisp(VehicleSide side) {
    double time = GetSystem()->GetChTime();
    return m_post_linact[side]->GetMotionFunction()->Get_y(time);
}

double ChSuspensionTestRigPlatform::GetActuatorForce(VehicleSide side) {
    return m_post_linact[side]->Get_react_force().x();
}

double ChSuspensionTestRigPlatform::GetRideHeight() const {
    // Note: the chassis reference frame is constructed at a height of 0.
    return -(m_post[LEFT]->GetPos().z() + m_post[RIGHT]->GetPos().z()) / 2;
}

void ChSuspensionTestRigPlatform::CollectPlotData(double time) {
    // Suspension forces
    auto frc_left = ReportSuspensionForce(LEFT);
    auto frc_right = ReportSuspensionForce(RIGHT);

    // Tire forces
    auto tire_force_L = ReportTireForce(VehicleSide::LEFT);
    auto tire_force_R = ReportTireForce(VehicleSide::RIGHT);

    // Tire camber angles (flip sign of reported camber angle on the left to get common definition)
    double gamma_L = -m_tire[LEFT]->GetCamberAngle() * CH_C_RAD_TO_DEG;
    double gamma_R = m_tire[RIGHT]->GetCamberAngle() * CH_C_RAD_TO_DEG;

    *m_csv << time;

    *m_csv << GetLeftInput() << GetSpindlePos(LEFT).z() << GetSpindleLinVel(LEFT).z() << GetWheelTravel(LEFT);
    *m_csv << frc_left.spring_force << frc_left.shock_force;

    *m_csv << GetRightInput() << GetSpindlePos(RIGHT).z() << GetSpindleLinVel(RIGHT).z() << GetWheelTravel(RIGHT);
    *m_csv << frc_right.spring_force << frc_right.shock_force;

    *m_csv << GetRideHeight() << gamma_L << gamma_R;

    *m_csv << tire_force_L.point << tire_force_L.force << tire_force_L.moment;
    *m_csv << tire_force_R.point << tire_force_R.force << tire_force_R.moment;

    *m_csv << std::endl;
}

void ChSuspensionTestRigPlatform::PlotOutput(const std::string& out_dir, const std::string& out_name) {
    if (!m_plot_output)
        return;

    std::string out_file = out_dir + "/" + out_name + ".txt";
    m_csv->write_to_file(out_file);

#ifdef CHRONO_POSTPROCESS
    std::string gplfile = out_dir + "/tmp.gpl";
    postprocess::ChGnuPlot mplot(gplfile.c_str());

    std::string title = "Suspension test rig - Spring forces";
    mplot.OutputWindow(0);
    mplot.SetTitle(title.c_str());
    mplot.SetLabelX("wheel travel [m]");
    mplot.SetLabelY("spring force [N]");
    mplot.SetCommand("set format y '%4.1e'");
    mplot.SetCommand("set terminal wxt size 800, 600");
    mplot.Plot(out_file.c_str(), 5, 6, "left", " with lines lw 2");
    mplot.Plot(out_file.c_str(), 11, 12, "right", " with lines lw 2");

    title = "Suspension test rig - Shock forces";
    mplot.OutputWindow(1);
    mplot.SetTitle(title.c_str());
    mplot.SetLabelX("wheel veertical speed [m/s]");
    mplot.SetLabelY("shock force [N]");
    mplot.SetCommand("set format y '%4.1e'");
    mplot.SetCommand("set terminal wxt size 800, 600");
    mplot.Plot(out_file.c_str(), 4, 7, "left", " with lines lw 2");
    mplot.Plot(out_file.c_str(), 10, 13, "right", " with lines lw 2");

    title = "Suspension test rig - Camber angle";
    mplot.OutputWindow(2);
    mplot.SetTitle(title.c_str());
    mplot.SetLabelX("wheel travel [m]");
    mplot.SetLabelY("camber angle [deg]");
    mplot.SetCommand("set format y '%4.1f'");
    mplot.SetCommand("set terminal wxt size 800, 600");
    mplot.Plot(out_file.c_str(), 5, 15, "left", " with lines lw 2");
    mplot.Plot(out_file.c_str(), 11, 16, "right", " with lines lw 2");
#endif
}

// =============================================================================
// ChSuspensionTestRigPushrod class implementation
// =============================================================================
ChSuspensionTestRigPushrod::ChSuspensionTestRigPushrod(ChWheeledVehicle& vehicle,  // vehicle source
                                                       int axle_index,             // index of test suspension
                                                       double displ_limit,         // limits for post displacement
                                                       std::shared_ptr<ChTire> tire_left,   // left tire
                                                       std::shared_ptr<ChTire> tire_right,  // right tire
                                                       ChContactMethod contact_method       // contact method
                                                       )
    : ChSuspensionTestRig(vehicle, axle_index, displ_limit, tire_left, tire_right, contact_method) {
    InitializeSubsystems();
    Create();
}

ChSuspensionTestRigPushrod::ChSuspensionTestRigPushrod(
    const std::string& filename,         // JSON file with vehicle specification
    int axle_index,                      // index of the suspension to be tested
    double displ_limit,                  // limits for post displacement
    std::shared_ptr<ChTire> tire_left,   // left tire
    std::shared_ptr<ChTire> tire_right,  // right tire
    ChContactMethod contact_method       // contact method
    )
    : ChSuspensionTestRig(filename, axle_index, displ_limit, tire_left, tire_right, contact_method) {
    InitializeSubsystems();
    Create();
}

ChSuspensionTestRigPushrod::ChSuspensionTestRigPushrod(
    const std::string& filename,         // JSON file with test rig specification
    std::shared_ptr<ChTire> tire_left,   // left tire
    std::shared_ptr<ChTire> tire_right,  // right tire
    ChContactMethod contact_method       // contact method
    )
    : ChSuspensionTestRig(filename, tire_left, tire_right, contact_method) {
    InitializeSubsystems();
    Create();
}

void ChSuspensionTestRigPushrod::Create() {
    // Create and initialize the linear actuators.
    // These connect the spindle centers with ground points directly below the spindles at the initial configuration.
    auto func_L = chrono_types::make_shared<ChFunction_Setpoint>();
    auto func_R = chrono_types::make_shared<ChFunction_Setpoint>();

    auto pos_sL = m_suspension->GetSpindle(LEFT)->GetCoord();
    auto pos_sR = m_suspension->GetSpindle(RIGHT)->GetCoord();

    auto pos_gL = pos_sL;
    auto pos_gR = pos_sR;
    pos_gL.pos.z() = -m_rod_length;
    pos_gR.pos.z() = -m_rod_length;

    m_rod_linact[LEFT] = chrono_types::make_shared<ChLinkLinActuator>();
    m_rod_linact[LEFT]->SetNameString("L_rod_actuator");
    m_rod_linact[LEFT]->Set_dist_funct(func_L);
    m_rod_linact[LEFT]->Initialize(m_chassis->GetBody(), m_suspension->GetSpindle(LEFT), false, pos_gL, pos_sL);
    m_system->AddLink(m_rod_linact[LEFT]);

    m_rod_linact[RIGHT] = chrono_types::make_shared<ChLinkLinActuator>();
    m_rod_linact[RIGHT]->SetNameString("R_rod_actuator");
    m_rod_linact[RIGHT]->Set_dist_funct(func_R);
    m_rod_linact[RIGHT]->Initialize(m_chassis->GetBody(), m_suspension->GetSpindle(RIGHT), false, pos_gR, pos_sR);
    m_system->AddLink(m_rod_linact[RIGHT]);

    m_rod_linact[LEFT]->Set_lin_offset(m_rod_length);
    m_rod_linact[RIGHT]->Set_lin_offset(m_rod_length);

    // Create the two rod bodies (used only for visualization)
    m_rod[LEFT] = std::shared_ptr<ChBody>(m_system->NewBody());
    m_rod[LEFT]->SetPos(pos_sL.pos);
    m_rod[LEFT]->SetBodyFixed(true);
    m_system->Add(m_rod[LEFT]);
    AddRodVisualization(LEFT, ChColor(0.1f, 0.8f, 0.15f));

    m_rod[RIGHT] = std::shared_ptr<ChBody>(m_system->NewBody());
    m_rod[RIGHT]->SetPos(pos_sR.pos);
    m_rod[RIGHT]->SetBodyFixed(true);
    m_system->Add(m_rod[RIGHT]);
    AddRodVisualization(RIGHT, ChColor(0.8f, 0.1f, 0.1f));
}

void ChSuspensionTestRigPushrod::AddRodVisualization(VehicleSide side, const ChColor& color) {
    auto cyl = chrono_types::make_shared<ChCylinderShape>();
    cyl->GetCylinderGeometry().p1 = ChVector<>(0, 0, 0);
    cyl->GetCylinderGeometry().p2 = ChVector<>(0, 0, -m_rod_length);
    cyl->GetCylinderGeometry().rad = m_rod_radius;
    m_rod[side]->AddAsset(cyl);
    m_rod[side]->AddAsset(chrono_types::make_shared<ChColorAsset>(color));
}

double ChSuspensionTestRigPushrod::CalcDisplacementOffset() {
    // Set initial spindle position based on tire radius (note: tire assumed rigid here)
    return m_tire[LEFT]->GetRadius() - m_ride_height;
}

double ChSuspensionTestRigPushrod::CalcTerrainHeight(VehicleSide side) {
    // No terrain used here
    return -1000;
}

void ChSuspensionTestRigPushrod::UpdateActuators(double displ_left,
                                                 double displ_speed_left,
                                                 double displ_right,
                                                 double displ_speed_right) {
    double time = GetSystem()->GetChTime();
    auto func_L = std::static_pointer_cast<ChFunction_Setpoint>(m_rod_linact[LEFT]->Get_dist_funct());
    auto func_R = std::static_pointer_cast<ChFunction_Setpoint>(m_rod_linact[RIGHT]->Get_dist_funct());
    func_L->SetSetpointAndDerivatives(displ_left, displ_speed_left, 0.0);
    func_R->SetSetpointAndDerivatives(displ_right, displ_speed_right, 0.0);

    // Move the rod visualization bodies
    m_rod[LEFT]->SetPos(m_suspension->GetSpindle(LEFT)->GetPos());
    m_rod[RIGHT]->SetPos(m_suspension->GetSpindle(RIGHT)->GetPos());
}

double ChSuspensionTestRigPushrod::GetActuatorDisp(VehicleSide side) {
    double time = GetSystem()->GetChTime();
    return m_rod_linact[side]->Get_dist_funct()->Get_y(time);
}

double ChSuspensionTestRigPushrod::GetActuatorForce(VehicleSide side) {
    //// TODO
    ////ChVector<> react = m_rod_linact[side]->Get_react_force();
    return 0;
}

double ChSuspensionTestRigPushrod::GetRideHeight() const {
    // Estimated from average spindle positions and tire radius (note: tire assumed rigid here)
    auto spindle_avg =
        (m_suspension->GetSpindle(LEFT)->GetPos().z() + m_suspension->GetSpindle(RIGHT)->GetPos().z()) / 2;
    return m_tire[LEFT]->GetRadius() - spindle_avg;
}

void ChSuspensionTestRigPushrod::CollectPlotData(double time) {
    // Suspension forces
    auto frc_left = ReportSuspensionForce(LEFT);
    auto frc_right = ReportSuspensionForce(RIGHT);

    // Tire camber angles (flip sign of reported camber angle on the left to get common definition)
    double gamma_L = -m_tire[LEFT]->GetCamberAngle() * CH_C_RAD_TO_DEG;
    double gamma_R = m_tire[RIGHT]->GetCamberAngle() * CH_C_RAD_TO_DEG;

    *m_csv << time;

    *m_csv << GetLeftInput() << GetSpindlePos(LEFT).z() << GetSpindleLinVel(LEFT).z() << GetWheelTravel(LEFT);
    *m_csv << frc_left.spring_force << frc_left.shock_force;

    *m_csv << GetRightInput() << GetSpindlePos(RIGHT).z() << GetSpindleLinVel(RIGHT).z() << GetWheelTravel(RIGHT);
    *m_csv << frc_right.spring_force << frc_right.shock_force;

    *m_csv << GetRideHeight() << gamma_L << gamma_R;

    *m_csv << std::endl;
}

void ChSuspensionTestRigPushrod::PlotOutput(const std::string& out_dir, const std::string& out_name) {
    if (!m_plot_output)
        return;

    std::string out_file = out_dir + "/" + out_name + ".txt";
    m_csv->write_to_file(out_file);

#ifdef CHRONO_POSTPROCESS
    std::string gplfile = out_dir + "/tmp.gpl";
    postprocess::ChGnuPlot mplot(gplfile.c_str());

    std::string title = "Suspension test rig - Spring forces";
    mplot.OutputWindow(0);
    mplot.SetTitle(title.c_str());
    mplot.SetLabelX("wheel travel [m]");
    mplot.SetLabelY("spring force [N]");
    mplot.SetCommand("set format y '%4.1e'");
    mplot.SetCommand("set terminal wxt size 800, 600");
    mplot.Plot(out_file.c_str(), 5, 6, "left", " with lines lw 2");
    mplot.Plot(out_file.c_str(), 11, 12, "right", " with lines lw 2");

    title = "Suspension test rig - Shock forces";
    mplot.OutputWindow(1);
    mplot.SetTitle(title.c_str());
    mplot.SetLabelX("wheel veertical speed [m/s]");
    mplot.SetLabelY("shock force [N]");
    mplot.SetCommand("set format y '%4.1e'");
    mplot.SetCommand("set terminal wxt size 800, 600");
    mplot.Plot(out_file.c_str(), 4, 7, "left", " with lines lw 2");
    mplot.Plot(out_file.c_str(), 10, 13, "right", " with lines lw 2");

    title = "Suspension test rig - Camber angle";
    mplot.OutputWindow(2);
    mplot.SetTitle(title.c_str());
    mplot.SetLabelX("wheel travel [m]");
    mplot.SetLabelY("camber angle [deg]");
    mplot.SetCommand("set format y '%4.1f'");
    mplot.SetCommand("set terminal wxt size 800, 600");
    mplot.Plot(out_file.c_str(), 5, 15, "left", " with lines lw 2");
    mplot.Plot(out_file.c_str(), 11, 16, "right", " with lines lw 2");
#endif
}

}  // end namespace vehicle
}  // end namespace chrono
