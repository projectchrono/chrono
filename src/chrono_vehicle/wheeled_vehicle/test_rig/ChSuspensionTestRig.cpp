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
// Implementation of a suspension testing mechanism for a wheeld vehicle.
//
// The reference frame follows the ISO standard: Z-axis up, X-axis
// pointing forward, and Y-axis towards the left of the vehicle.
//
// =============================================================================

#include "chrono_vehicle/wheeled_vehicle/test_rig/ChSuspensionTestRig.h"

#include <algorithm>
#include <cstdio>

#include "chrono/assets/ChCylinderShape.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"

#ifdef CHRONO_POSTPROCESS
    #include "chrono_postprocess/ChGnuPlot.h"
#endif

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// =============================================================================
// Definition of a terrain object for use by a suspension test rig.
// Note that this assumes an ISO world frame.
class TestRigTerrain : public ChTerrain {
  public:
    TestRigTerrain(int naxles, const std::vector<double>& x);
    virtual double GetHeight(const ChVector<>& loc) const override;
    virtual ChVector<> GetNormal(const ChVector<>& loc) const override;
    virtual float GetCoefficientFriction(const ChVector<>& loc) const override;
    int m_naxles;
    std::vector<double> m_x;
    std::vector<double> m_height_L;
    std::vector<double> m_height_R;
};

TestRigTerrain::TestRigTerrain(int naxles, const std::vector<double>& x) : m_naxles(naxles), m_x(x) {
    m_height_L.resize(naxles, -1000);
    m_height_R.resize(naxles, -1000);
}

double TestRigTerrain::GetHeight(const ChVector<>& loc) const {
    double min_delta = 1000;
    int axle = 0;
    for (int ia = 0; ia < m_naxles; ia++) {
        auto delta = std::abs(loc.x() - m_x[ia]);
        if (delta < min_delta) {
            axle = ia;
            min_delta = delta;
        }
    }
    return (loc.y() < 0) ? m_height_R[axle] : m_height_L[axle];
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
ChSuspensionTestRig::ChSuspensionTestRig(std::shared_ptr<ChWheeledVehicle> vehicle,
                                         std::vector<int> axle_index,
                                         double displ_limit)
    : m_vehicle(vehicle),
      m_axle_index(axle_index),
      m_displ_limit(displ_limit),
      m_ride_height(-1),
      m_vis_suspension(VisualizationType::PRIMITIVES),
      m_vis_subchassis(VisualizationType::PRIMITIVES),
      m_vis_steering(VisualizationType::PRIMITIVES),
      m_vis_wheel(VisualizationType::NONE),
      m_vis_tire(VisualizationType::PRIMITIVES),
      m_plot_output(false),
      m_plot_output_step(0),
      m_next_plot_output_time(0),
      m_csv(nullptr) {
    // Cache number of tested axles and steering mechanisms
    m_naxles = (int)axle_index.size();

    // Initialize the vehicle
    m_vehicle->Initialize(ChCoordsys<>());

    // Fix chassis to ground
    m_vehicle->GetChassis()->SetFixed(true);

    // Disconnect driveline
    m_vehicle->DisconnectDriveline();
}

ChSuspensionTestRig::ChSuspensionTestRig(const std::string& spec_filename)
    : m_vis_suspension(VisualizationType::PRIMITIVES),
      m_vis_subchassis(VisualizationType::PRIMITIVES),
      m_vis_steering(VisualizationType::PRIMITIVES),
      m_vis_wheel(VisualizationType::NONE),
      m_vis_tire(VisualizationType::PRIMITIVES),
      m_plot_output(false),
      m_plot_output_step(0),
      m_next_plot_output_time(0),
      m_csv(nullptr) {
    // Open and parse the input file (rig JSON specification file)
    Document d;
    ReadFileJSON(spec_filename, d);
    if (d.IsNull())
        return;

    // Read top-level data
    assert(d.HasMember("Type"));
    std::string type = d["Type"].GetString();
    assert(type.compare("SuspensionTestRig") == 0);

    assert(d.HasMember("Vehicle Input File"));
    m_vehicle = chrono_types::make_shared<WheeledVehicle>(vehicle::GetDataFile(d["Vehicle Input File"].GetString()),
                                                          ChContactMethod::SMC);

    assert(d.HasMember("Test Axle Indices"));
    m_naxles = (int)d["Test Axle Indices"].Size();
    m_axle_index.resize(m_naxles);
    for (int ia = 0; ia < m_naxles; ia++)
        m_axle_index[ia] = d["Test Axle Indices"][ia].GetInt();

    if (d.HasMember("Test Subchassis Indices")) {
        int nsubchassis = (int)d["Test Subchassis Indices"].Size();
        m_subchassis_index.resize(nsubchassis);
        for (int is = 0; is < nsubchassis; is++)
            m_subchassis_index[is] = d["Test Subchassis Indices"][is].GetInt();
    }

    if (d.HasMember("Test Steering Indices")) {
        int nsteerings = (int)d["Test Steering Indices"].Size();
        m_steering_index.resize(nsteerings);
        for (int is = 0; is < nsteerings; is++)
            m_steering_index[is] = d["Test Steering Indices"][is].GetInt();
    }

    assert(d.HasMember("Displacement Limit"));
    m_displ_limit = d["Displacement Limit"].GetDouble();

    if (d.HasMember("Initial Ride Height")) {
        m_ride_height = d["Initial Ride Height"].GetDouble();
    } else {
        m_ride_height = -1;
    }

    // Initialize the vehicle
    m_vehicle->Initialize(ChCoordsys<>());

    // Fix chassis to ground
    m_vehicle->GetChassis()->SetFixed(true);

    // Disconnect driveline
    m_vehicle->DisconnectDriveline();
}

ChSuspensionTestRig::~ChSuspensionTestRig() {
    delete m_csv;
}

void ChSuspensionTestRig::IncludeSubchassis(int index) {
    m_subchassis_index.push_back(index);
}

void ChSuspensionTestRig::IncludeSteeringMechanism(int index) {
    m_steering_index.push_back(index);
}

void ChSuspensionTestRig::Initialize() {
    for (auto ia : m_axle_index) {
        if (ia < 0 || ia >= m_vehicle->GetNumberAxles()) {
            throw ChException("Incorrect axle index " + std::to_string(ia) + " for the given vehicle");
        }
        for (const auto& wheel : m_vehicle->GetAxle(ia)->GetWheels()) {
            if (!wheel->GetTire()) {
                throw ChException("No tires attached to axle " + std::to_string(ia) + " for the given vehicle");
            }
        }
    }

    for (auto is : m_steering_index) {
        if (is < 0 || is >= (int)m_vehicle->GetSteerings().size()) {
            throw ChException("Incorrect steering index " + std::to_string(is) + " for the given vehicle");
        }
    }

    if (!m_driver) {
        throw ChException("No driver system provided");
    }

    // Initialize visualization for all vehicle subsystems
    m_vehicle->SetChassisVisualizationType(VisualizationType::NONE);
    m_vehicle->SetSubchassisVisualizationType(VisualizationType::NONE);
    m_vehicle->SetSteeringVisualizationType(VisualizationType::NONE);
    m_vehicle->SetSuspensionVisualizationType(VisualizationType::NONE);
    m_vehicle->SetWheelVisualizationType(VisualizationType::NONE);
    m_vehicle->SetTireVisualizationType(VisualizationType::NONE);

    // Process axles
    for (auto ia : m_axle_index) {
        const auto& axle = m_vehicle->GetAxle(ia);
        // Overwrite visualization setting
        axle->m_suspension->SetVisualizationType(m_vis_suspension);
        if (axle->m_antirollbar) {
            axle->m_antirollbar->SetVisualizationType(m_vis_suspension);
        }
        for (const auto& wheel : axle->GetWheels()) {
            wheel->SetVisualizationType(m_vis_wheel);
            wheel->GetTire()->SetVisualizationType(m_vis_tire);
        }
        // Enable output
        m_vehicle->SetSuspensionOutput(ia, true);
        if (axle->m_antirollbar) {
            m_vehicle->SetAntirollbarOutput(ia, true);
        }

        // Initialize reference spindle vertical positions at design configuration.
        m_spindle_ref_L.push_back(axle->m_suspension->GetSpindlePos(LEFT).z());
        m_spindle_ref_R.push_back(axle->m_suspension->GetSpindlePos(RIGHT).z());
    }
    
    // Process subchassis mechanisms
    for (auto is : m_subchassis_index) {
        // Overwrite visualization setting
        m_vehicle->GetSubchassis(is)->SetVisualizationType(m_vis_subchassis);
        // Enable output
        m_vehicle->SetSubchassisOutput(is, true);
    }

    // Process steering mechanisms
    for (auto is : m_steering_index) {
        // Overwrite visualization setting
        m_vehicle->GetSteering(is)->SetVisualizationType(m_vis_steering);
        // Enable output
        m_vehicle->SetSteeringOutput(is, true);
    }

    // Let derived classes construct their rig mechanism
    InitializeRig();

    // Calculate displacement offset to set rig at specified ride height (if any).
    // The rig will be moved dynamically to this configuration over a time interval displ_delay.
    double displ_delay = 0;
    m_displ_offset.resize(m_naxles, 0.0);
    if (m_ride_height > 0) {
        displ_delay = 0.5;
        for (int ia = 0; ia < m_naxles; ia++) {
            m_displ_offset[ia] = CalcDisplacementOffset(ia);
        }
    }

    // Create the terrain system; pass spindle X positions
    std::vector<double> xS;
    for (auto ia : m_axle_index) {
        const auto& suspension = m_vehicle->GetAxle(ia)->m_suspension;
        xS.push_back(suspension->GetSpindlePos(LEFT).x());
    }
    m_terrain = std::unique_ptr<ChTerrain>(new TestRigTerrain(m_naxles, xS));

    // Initialize the driver system
    m_driver->m_delay = displ_delay;
    m_driver->Initialize(m_naxles);
    m_steering_input = 0;
    m_left_inputs.resize(m_naxles, 0.0);
    m_right_inputs.resize(m_naxles, 0.0);
}

// -----------------------------------------------------------------------------

void ChSuspensionTestRig::SetDriver(std::shared_ptr<ChDriverSTR> driver) {
    m_driver = driver;
}

// -----------------------------------------------------------------------------

const ChVector<>& ChSuspensionTestRig::GetSpindlePos(int axle, VehicleSide side) const {
    return m_vehicle->GetSpindlePos(m_axle_index[axle], side);
}

ChQuaternion<> ChSuspensionTestRig::GetSpindleRot(int axle, VehicleSide side) const {
    return m_vehicle->GetSpindleRot(m_axle_index[axle], side);
}

const ChVector<>& ChSuspensionTestRig::GetSpindleLinVel(int axle, VehicleSide side) const {
    return m_vehicle->GetSpindleLinVel(m_axle_index[axle], side);
}

ChVector<> ChSuspensionTestRig::GetSpindleAngVel(int axle, VehicleSide side) const {
    return m_vehicle->GetSpindleAngVel(m_axle_index[axle], side);
}

double ChSuspensionTestRig::GetWheelTravel(int axle, VehicleSide side) const {
    if (m_vehicle->GetChTime() < m_driver->m_delay)
        return 0;
    const auto& suspension = m_vehicle->GetAxle(m_axle_index[axle])->m_suspension;
    return (side == LEFT) ? suspension->GetSpindlePos(LEFT).z() - m_spindle_ref_L[axle]
                          : suspension->GetSpindlePos(RIGHT).z() - m_spindle_ref_R[axle];
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChSuspensionTestRig::Advance(double step) {
    double time = m_vehicle->GetChTime();

    // Actuation inputs
    std::vector<double> displ_left(m_naxles, 0.0);
    std::vector<double> displ_right(m_naxles, 0.0);
    std::vector<double> displ_speed_left(m_naxles, 0.0);
    std::vector<double> displ_speed_right(m_naxles, 0.0);

    if (time < m_driver->m_delay) {
        // Automatic phase to bring rig at specified initial ride height
        for (int ia = 0; ia < m_naxles; ia++) {
            displ_left[ia] = m_displ_offset[ia] * time / m_driver->m_delay;
            displ_right[ia] = m_displ_offset[ia] * time / m_driver->m_delay;
            // Update spindle vertical reference positions
            const auto& suspension = m_vehicle->GetAxle(m_axle_index[ia])->m_suspension;
            m_spindle_ref_L.push_back(suspension->GetSpindlePos(LEFT).z());
            m_spindle_ref_R.push_back(suspension->GetSpindlePos(RIGHT).z());
        }
    } else {
        // Use actual driver inputs to set current actuator displacements
        m_steering_input = m_driver->GetSteering();
        m_left_inputs = m_driver->GetDisplacementLeft();
        m_right_inputs = m_driver->GetDisplacementRight();
        const auto& left_input_speed = m_driver->GetDisplacementSpeedLeft();
        const auto& right_input_speed = m_driver->GetDisplacementSpeedRight();
        for (int ia = 0; ia < m_naxles; ia++) {
            displ_left[ia] = m_displ_offset[ia] + m_displ_limit * m_left_inputs[ia];
            displ_right[ia] = m_displ_offset[ia] + m_displ_limit * m_right_inputs[ia];
            displ_speed_left[ia] = m_displ_limit * left_input_speed[ia];
            displ_speed_right[ia] = m_displ_limit * right_input_speed[ia];
        }
    }

    ////std::cout << time << " " << m_steering_input << "   ";
    ////std::cout << m_left_inputs[0] << " " << m_right_inputs[0] << "   ";
    ////std::cout << displ_left[0] << " " << displ_right[0] << std::endl;

    // Synchronize vehicle system
    DriverInputs driver_inputs = {m_steering_input, 0, 0};
    m_vehicle->Synchronize(time, driver_inputs, *m_terrain);

    // Synchronize driver system
    m_driver->Synchronize(time);

    // Update actuators
    UpdateActuators(displ_left, displ_speed_left, displ_right, displ_speed_right);

    // Update the terrain "height" under each spindle.
    auto terrain = static_cast<TestRigTerrain*>(m_terrain.get());
    for (int ia = 0; ia < m_naxles; ia++) {
        terrain->m_height_L[ia] = CalcTerrainHeight(ia, LEFT);
        terrain->m_height_R[ia] = CalcTerrainHeight(ia, RIGHT);
    }

    // Advance vehicle state
    m_vehicle->Advance(step);

    // Generate output for plotting if requested
    time = m_vehicle->GetChTime();
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
    for (auto ia : m_axle_index) {
        const auto& axle = m_vehicle->GetAxle(ia);
        GetLog() << "\n---- LEFT side suspension constraint violations\n\n";
        axle->m_suspension->LogConstraintViolations(LEFT);
        GetLog() << "\n---- RIGHT side suspension constraint violations\n\n";
        axle->m_suspension->LogConstraintViolations(RIGHT);
    }

    // Report constraint violations for the steering joints
    for (auto is : m_steering_index) {
        GetLog() << "\n---- STEERING constrain violations\n\n";
        m_vehicle->GetSteering(is)->LogConstraintViolations();
    }

    GetLog().SetNumFormat("%g");
}

// -----------------------------------------------------------------------------

void ChSuspensionTestRig::SetOutput(ChVehicleOutput::Type type,
                                    const std::string& out_dir,
                                    const std::string& out_name,
                                    double output_step) {
    m_vehicle->SetOutput(type, out_dir, out_name, output_step);
}

void ChSuspensionTestRig::SetPlotOutput(double output_step) {
    m_plot_output = true;
    m_plot_output_step = output_step;
    m_csv = new utils::CSV_writer(" ");
}

void ChSuspensionTestRig::CollectPlotData(double time) {
    *m_csv << time;

    for (int ia = 0; ia < m_naxles; ia++) {
        const auto& axle = m_vehicle->GetAxle(m_axle_index[ia]);

        // Suspension spring and shock forces
        auto frc_left = axle->m_suspension->ReportSuspensionForce(LEFT);
        auto frc_right = axle->m_suspension->ReportSuspensionForce(RIGHT);

        // Tire camber angle (flip sign of reported camber angle on the left to get common definition)
        double gamma_left = -axle->m_wheels[0]->GetTire()->GetCamberAngle() * CH_C_RAD_TO_DEG;
        double gamma_right = axle->m_wheels[1]->GetTire()->GetCamberAngle() * CH_C_RAD_TO_DEG;

        *m_csv << m_left_inputs[ia] << m_right_inputs[ia];                    // 1          2
        *m_csv << GetSpindlePos(ia, LEFT) << GetSpindlePos(ia, RIGHT);        // 3 4 5      6 7 8
        *m_csv << GetSpindleLinVel(ia, LEFT) << GetSpindleLinVel(ia, RIGHT);  // 9 10 11    12 13 14
        *m_csv << GetWheelTravel(ia, LEFT) << GetWheelTravel(ia, RIGHT);      // 15         16
        *m_csv << frc_left.spring_force << frc_right.spring_force;            // 17         18
        *m_csv << frc_left.shock_force << frc_right.shock_force;              // 19         20
        *m_csv << gamma_left << gamma_right;                                  // 21         22
        *m_csv << GetRideHeight(ia);                                          // 23
    }

    *m_csv << std::endl;
}

void ChSuspensionTestRig::PlotOutput(const std::string& out_dir, const std::string& out_name) {
    if (!m_plot_output)
        return;

    std::string out_file = out_dir + "/" + out_name + ".txt";
    m_csv->write_to_file(out_file);

#ifdef CHRONO_POSTPROCESS
    std::string gplfile = out_dir + "/tmp.gpl";
    postprocess::ChGnuPlot mplot(gplfile.c_str());

    for (int ia = 0; ia < m_naxles; ia++) {
        std::string title = "Suspension test rig - Axle " + std::to_string(ia) + " - Spring forces ";
        mplot.OutputWindow(3 * ia + 0);
        mplot.SetTitle(title.c_str());
        mplot.SetLabelX("wheel travel [m]");
        mplot.SetLabelY("spring force [N]");
        mplot.SetCommand("set format y '%4.1e'");
        mplot.SetCommand("set terminal wxt size 800, 600");
        mplot.Plot(out_file.c_str(), 1 + 23 * ia + 15, 1 + 23 * ia + 17, "left", " with lines lw 2");
        mplot.Plot(out_file.c_str(), 1 + 23 * ia + 16, 1 + 23 * ia + 18, "right", " with lines lw 2");

        title = "Suspension test rig - Axle " + std::to_string(ia) + " - Shock forces";
        mplot.OutputWindow(3 * ia + 1);
        mplot.SetTitle(title.c_str());
        mplot.SetLabelX("wheel vertical speed [m/s]");
        mplot.SetLabelY("shock force [N]");
        mplot.SetCommand("set format y '%4.1e'");
        mplot.SetCommand("set terminal wxt size 800, 600");
        mplot.Plot(out_file.c_str(), 1 + 23 * ia + 11, 1 + 23 * ia + 19, "left", " with lines lw 2");
        mplot.Plot(out_file.c_str(), 1 + 23 * ia + 14, 1 + 23 * ia + 20, "right", " with lines lw 2");

        title = "Suspension test rig - Axle " + std::to_string(ia) + " - Camber angle";
        mplot.OutputWindow(3 * ia + 2);
        mplot.SetTitle(title.c_str());
        mplot.SetLabelX("wheel travel [m]");
        mplot.SetLabelY("camber angle [deg]");
        mplot.SetCommand("set format y '%4.1f'");
        mplot.SetCommand("set terminal wxt size 800, 600");
        mplot.Plot(out_file.c_str(), 1 + 23 * ia + 15, 1 + 23 * ia + 21, "left", " with lines lw 2");
        mplot.Plot(out_file.c_str(), 1 + 23 * ia + 16, 1 + 23 * ia + 22, "right", " with lines lw 2");
    }
#endif
}

// =============================================================================
// ChSuspensionTestRigPlatform class implementation
// =============================================================================

ChSuspensionTestRigPlatform::ChSuspensionTestRigPlatform(std::shared_ptr<ChWheeledVehicle> vehicle,
                                                         std::vector<int> axle_index,
                                                         double displ_limit)
    : ChSuspensionTestRig(vehicle, axle_index, displ_limit) {}

ChSuspensionTestRigPlatform::ChSuspensionTestRigPlatform(const std::string& spec_filename)
    : ChSuspensionTestRig(spec_filename) {}

ChSuspensionTestRigPlatform::~ChSuspensionTestRigPlatform() {
    auto sys = m_vehicle->GetSystem();
    if (sys) {
        for (int ia = 0; ia < m_naxles; ia++) {
            sys->Remove(m_post_L[ia]);
            sys->Remove(m_post_R[ia]);
            sys->Remove(m_linact_L[ia]);
            sys->Remove(m_linact_R[ia]);
        }
    }
}

void ChSuspensionTestRigPlatform::InitializeRig() {
    auto sys = m_vehicle->GetSystem();

    // Create a contact material for the posts (shared)
    //// TODO: are default material properties ok?
    ChContactMaterialData minfo;
    auto post_mat = minfo.CreateMaterial(sys->GetContactMethod());

    for (int ia = 0; ia < m_naxles; ia++) {
        const auto& axle = m_vehicle->GetAxle(m_axle_index[ia]);
        const auto& suspension = axle->m_suspension;
        auto tire_radius = axle->m_wheels[0]->GetTire()->GetRadius();

        // Create the left post body (green)
        ChVector<> spindle_L_pos = suspension->GetSpindlePos(LEFT);
        ChVector<> post_L_pos = spindle_L_pos - ChVector<>(0, 0, tire_radius);

        auto post_L = std::shared_ptr<ChBody>(sys->NewBody());
        post_L->SetPos(post_L_pos);
        post_L->SetMass(100);
        post_L->SetCollide(true);
        sys->Add(post_L);
        AddPostVisualization(post_L, ChColor(0.1f, 0.8f, 0.15f));

        post_L->GetCollisionModel()->ClearModel();
        post_L->GetCollisionModel()->AddCylinder(post_mat, m_post_radius, m_post_radius, m_post_hheight,
                                                 ChVector<>(0, 0, -m_post_hheight),
                                                 ChMatrix33<>(Q_from_AngX(CH_C_PI / 2)));
        post_L->GetCollisionModel()->BuildModel();

        // Create the right post body (red)
        ChVector<> spindle_R_pos = suspension->GetSpindlePos(RIGHT);
        ChVector<> post_R_pos = spindle_R_pos - ChVector<>(0, 0, tire_radius);

        auto post_R = std::shared_ptr<ChBody>(sys->NewBody());
        post_R->SetPos(post_R_pos);
        post_R->SetMass(100);
        post_R->SetCollide(true);
        sys->Add(post_R);
        AddPostVisualization(post_R, ChColor(0.8f, 0.1f, 0.1f));

        post_R->GetCollisionModel()->ClearModel();
        post_R->GetCollisionModel()->AddCylinder(post_mat, m_post_radius, m_post_radius, m_post_hheight,
                                                 ChVector<>(0, 0, -m_post_hheight),
                                                 ChMatrix33<>(Q_from_AngX(CH_C_PI / 2)));
        post_R->GetCollisionModel()->BuildModel();

        // Create and initialize actuators
        auto func_L = chrono_types::make_shared<ChFunction_Setpoint>();
        auto func_R = chrono_types::make_shared<ChFunction_Setpoint>();

        auto linact_L = chrono_types::make_shared<ChLinkMotorLinearPosition>();
        linact_L->SetNameString("L_post_linActuator");
        linact_L->SetMotionFunction(func_L);
        linact_L->Initialize(m_vehicle->GetChassisBody(), post_L,
                             ChFrame<>(ChVector<>(post_L_pos), Q_from_AngY(CH_C_PI_2)));
        sys->AddLink(linact_L);

        auto linact_R = chrono_types::make_shared<ChLinkMotorLinearPosition>();
        linact_R->SetNameString("R_post_linActuator");
        linact_R->SetMotionFunction(func_R);
        linact_R->Initialize(m_vehicle->GetChassisBody(), post_R,
                             ChFrame<>(ChVector<>(post_R_pos), Q_from_AngY(CH_C_PI_2)));
        sys->AddLink(linact_R);

        // Cache bodies and actuators
        m_post_L.push_back(post_L);
        m_post_R.push_back(post_R);
        m_linact_L.push_back(linact_L);
        m_linact_R.push_back(linact_R);
    }
}

void ChSuspensionTestRigPlatform::AddPostVisualization(std::shared_ptr<ChBody> post, const ChColor& color) {
    // Platform (on post body)
    auto mat = chrono_types::make_shared<ChVisualMaterial>();
    mat->SetDiffuseColor({color.R, color.G, color.B});

    auto base_cyl = chrono_types::make_shared<ChCylinderShape>();
    base_cyl->GetCylinderGeometry().rad = m_post_radius;
    base_cyl->GetCylinderGeometry().p1 = ChVector<>(0, 0, 0);
    base_cyl->GetCylinderGeometry().p2 = ChVector<>(0, 0, -2 * m_post_hheight);
    base_cyl->AddMaterial(mat);
    post->AddVisualShape(base_cyl);

    // Piston (on post body)
    auto piston = chrono_types::make_shared<ChCylinderShape>();
    piston->GetCylinderGeometry().rad = m_post_radius / 6.0;
    piston->GetCylinderGeometry().p1 = ChVector<>(0, 0, -2 * m_post_hheight);
    piston->GetCylinderGeometry().p2 = ChVector<>(0, 0, -m_post_hheight * 20.0);
    piston->AddMaterial(mat);
    post->AddVisualShape(piston);

    // Post sleeve (on chassis/ground body)
    auto cyl = chrono_types::make_shared<ChCylinderShape>();
    cyl->GetCylinderGeometry().rad = m_post_radius / 4.0;
    cyl->GetCylinderGeometry().p1 = post->GetPos() - ChVector<>(0, 0, 16 * m_post_hheight);
    cyl->GetCylinderGeometry().p2 = post->GetPos() - ChVector<>(0, 0, 32 * m_post_hheight);
    cyl->AddMaterial(mat);
    m_vehicle->GetChassisBody()->AddVisualShape(cyl);
}

double ChSuspensionTestRigPlatform::CalcDisplacementOffset(int axle) {
    return -m_ride_height - m_post_L[axle]->GetPos().z();
}

double ChSuspensionTestRigPlatform::CalcTerrainHeight(int axle, VehicleSide side) {
    // Update the height of the underlying terrain object, using the current z positions of the post bodies.
    return (side == LEFT) ? m_post_L[axle]->GetPos().z() : m_post_R[axle]->GetPos().z();
}

void ChSuspensionTestRigPlatform::UpdateActuators(std::vector<double> displ_left,
                                                  std::vector<double> displ_speed_left,
                                                  std::vector<double> displ_right,
                                                  std::vector<double> displ_speed_right) {
    for (int ia = 0; ia < m_naxles; ia++) {
        auto func_L = std::static_pointer_cast<ChFunction_Setpoint>(m_linact_L[ia]->GetMotionFunction());
        auto func_R = std::static_pointer_cast<ChFunction_Setpoint>(m_linact_R[ia]->GetMotionFunction());
        func_L->SetSetpointAndDerivatives(displ_left[ia], displ_speed_left[ia], 0.0);
        func_R->SetSetpointAndDerivatives(displ_right[ia], displ_speed_right[ia], 0.0);
    }
}

double ChSuspensionTestRigPlatform::GetActuatorDisp(int axle, VehicleSide side) {
    double time = m_vehicle->GetChTime();
    return (side == LEFT) ? m_linact_L[axle]->GetMotionFunction()->Get_y(time)
                          : m_linact_R[axle]->GetMotionFunction()->Get_y(time);
}

double ChSuspensionTestRigPlatform::GetActuatorForce(int axle, VehicleSide side) {
    return (side == LEFT) ? m_linact_L[axle]->Get_react_force().x() : m_linact_R[axle]->Get_react_force().x();
}

double ChSuspensionTestRigPlatform::GetRideHeight(int axle) const {
    // Note: the chassis reference frame is constructed at a height of 0.
    return -(m_post_L[axle]->GetPos().z() + m_post_R[axle]->GetPos().z()) / 2;
}

// =============================================================================
// ChSuspensionTestRigPushrod class implementation
// =============================================================================

ChSuspensionTestRigPushrod::ChSuspensionTestRigPushrod(std::shared_ptr<ChWheeledVehicle> vehicle,
                                                       std::vector<int> axle_index,
                                                       double displ_limit)
    : ChSuspensionTestRig(vehicle, axle_index, displ_limit) {}

ChSuspensionTestRigPushrod::ChSuspensionTestRigPushrod(const std::string& spec_filename)
    : ChSuspensionTestRig(spec_filename) {}

ChSuspensionTestRigPushrod::~ChSuspensionTestRigPushrod() {
    auto sys = m_vehicle->GetSystem();
    if (sys) {
        for (int ia = 0; ia < m_naxles; ia++) {
            sys->Remove(m_rod_L[ia]);
            sys->Remove(m_rod_R[ia]);
            sys->Remove(m_linact_L[ia]);
            sys->Remove(m_linact_R[ia]);
        }
    }
}

void ChSuspensionTestRigPushrod::InitializeRig() {
    auto sys = m_vehicle->GetSystem();

    for (int ia = 0; ia < m_naxles; ia++) {
        const auto& axle = m_vehicle->GetAxle(m_axle_index[ia]);
        const auto& suspension = axle->m_suspension;

        // Create and initialize the linear actuators.
        // These connect the spindle centers with ground points directly below the spindles at the initial
        // configuration.
        auto func_L = chrono_types::make_shared<ChFunction_Setpoint>();
        auto func_R = chrono_types::make_shared<ChFunction_Setpoint>();

        const auto& pos_sL = suspension->GetSpindle(LEFT)->GetCoord();
        const auto& pos_sR = suspension->GetSpindle(RIGHT)->GetCoord();

        auto pos_gL = pos_sL;
        auto pos_gR = pos_sR;
        pos_gL.pos.z() = -m_rod_length;
        pos_gR.pos.z() = -m_rod_length;

        auto linact_L = chrono_types::make_shared<ChLinkLinActuator>();
        linact_L->SetNameString("L_rod_actuator");
        linact_L->SetActuatorFunction(func_L);
        linact_L->Initialize(m_vehicle->GetChassisBody(), suspension->GetSpindle(LEFT), false, pos_gL, pos_sL);
        linact_L->SetDistanceOffset(m_rod_length);
        sys->AddLink(linact_L);

        auto linact_R = chrono_types::make_shared<ChLinkLinActuator>();
        linact_R->SetNameString("R_rod_actuator");
        linact_R->SetActuatorFunction(func_R);
        linact_R->Initialize(m_vehicle->GetChassisBody(), suspension->GetSpindle(RIGHT), false, pos_gR, pos_sR);
        linact_R->SetDistanceOffset(m_rod_length);
        sys->AddLink(linact_R);

        // Create the two rod bodies (used only for visualization)
        auto rod_L = std::shared_ptr<ChBody>(sys->NewBody());
        rod_L->SetPos(pos_sL.pos);
        rod_L->SetBodyFixed(true);
        sys->Add(rod_L);
        AddRodVisualization(rod_L, ChColor(0.1f, 0.8f, 0.15f));

        auto rod_R = std::shared_ptr<ChBody>(sys->NewBody());
        rod_R->SetPos(pos_sR.pos);
        rod_R->SetBodyFixed(true);
        sys->Add(rod_R);
        AddRodVisualization(rod_R, ChColor(0.8f, 0.1f, 0.1f));

        // Cache bodies and actuators
        m_rod_L.push_back(rod_L);
        m_rod_R.push_back(rod_R);
        m_linact_L.push_back(linact_L);
        m_linact_R.push_back(linact_R);
    }
}

void ChSuspensionTestRigPushrod::AddRodVisualization(std::shared_ptr<ChBody> rod, const ChColor& color) {
    auto cyl = chrono_types::make_shared<ChCylinderShape>();
    cyl->GetCylinderGeometry().p1 = ChVector<>(0, 0, 0);
    cyl->GetCylinderGeometry().p2 = ChVector<>(0, 0, -m_rod_length);
    cyl->GetCylinderGeometry().rad = m_rod_radius;
    cyl->SetColor(color);
    rod->AddVisualShape(cyl);
}

double ChSuspensionTestRigPushrod::CalcDisplacementOffset(int axle) {
    // Set initial spindle position based on tire radius (note: tire assumed rigid here)
    auto tire_radius = m_vehicle->GetAxle(m_axle_index[axle])->m_wheels[0]->GetTire()->GetRadius();
    return tire_radius - m_ride_height;
}

double ChSuspensionTestRigPushrod::CalcTerrainHeight(int axle, VehicleSide side) {
    // No terrain used here
    return -1000;
}

void ChSuspensionTestRigPushrod::UpdateActuators(std::vector<double> displ_left,
                                                 std::vector<double> displ_speed_left,
                                                 std::vector<double> displ_right,
                                                 std::vector<double> displ_speed_right) {
    for (int ia = 0; ia < m_naxles; ia++) {
        auto func_L = std::static_pointer_cast<ChFunction_Setpoint>(m_linact_L[ia]->GetActuatorFunction());
        auto func_R = std::static_pointer_cast<ChFunction_Setpoint>(m_linact_R[ia]->GetActuatorFunction());
        func_L->SetSetpointAndDerivatives(displ_left[ia], displ_speed_left[ia], 0.0);
        func_R->SetSetpointAndDerivatives(displ_right[ia], displ_speed_right[ia], 0.0);

        // Move the rod visualization bodies
        const auto& axle = m_vehicle->GetAxle(m_axle_index[ia]);
        m_rod_L[ia]->SetPos(axle->m_suspension->GetSpindle(LEFT)->GetPos());
        m_rod_R[ia]->SetPos(axle->m_suspension->GetSpindle(RIGHT)->GetPos());
    }
}

double ChSuspensionTestRigPushrod::GetActuatorDisp(int axle, VehicleSide side) {
    double time = m_vehicle->GetChTime();
    return (side == LEFT) ? m_linact_L[axle]->GetActuatorFunction()->Get_y(time)
                          : m_linact_R[axle]->GetActuatorFunction()->Get_y(time);
}

double ChSuspensionTestRigPushrod::GetActuatorForce(int axle, VehicleSide side) {
    //// TODO
    ////ChVector<> react = m_linact_L[ia]->Get_react_force();
    return 0;
}

double ChSuspensionTestRigPushrod::GetRideHeight(int axle) const {
    // Estimated from average spindle positions and tire radius (note: tire assumed rigid here)
    const auto& suspension = m_vehicle->GetAxle(m_axle_index[axle])->m_suspension;
    auto tire_radius = m_vehicle->GetAxle(m_axle_index[axle])->m_wheels[0]->GetTire()->GetRadius();
    auto spindle_avg = (suspension->GetSpindle(LEFT)->GetPos().z() + suspension->GetSpindle(RIGHT)->GetPos().z()) / 2;
    return tire_radius - spindle_avg;
}

}  // end namespace vehicle
}  // end namespace chrono
