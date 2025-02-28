// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2019 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Rainer Gericke
// =============================================================================
//
// HMMWV double lane change test ISO 3888-1 or ISO 3888-2.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// Call options:
//    1) demo_VEH_HMMWV_DoubleLaneChange
//        -> standard demo velkmh= 30km/h, ISO3888-1, left turn
//
//    2) demo_VEH_HMMWV_DoubleLaneChange VelKMH
//        -> velkmh= VelKMH, ISO3888-1, left turn
//
//    3) demo_VEH_HMMWV_DoubleLaneChange VelKMH [1|2]
//        -> velkmh= VelKMH, ISO variant set to 1 or 2, left turn
//
//    4) demo_VEH_HMMWV_DoubleLaneChange VelKMH [1|2] [L|R]
//        -> velkmh= VelKMH, ISO variant set to 1 or 2, turn set to left or right
//
// =============================================================================

#include <cmath>

#include "chrono/ChConfig.h"
#include "chrono/utils/ChFilters.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/utils/ChUtils.h"
#include "chrono/core/ChTimer.h"
#include "chrono/assets/ChVisualShapeBox.h"
#include "chrono/assets/ChVisualShapeCone.h"
#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/utils/ChVehiclePath.h"

#include "chrono_models/vehicle/hmmwv/HMMWV.h"

#ifdef CHRONO_POSTPROCESS
    #include "chrono_postprocess/ChGnuPlot.h"
#endif

#ifdef CHRONO_IRRLICHT
    #include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"
using namespace chrono::irrlicht;
#endif

#ifdef CHRONO_VSG
    #include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemVSG.h"
using namespace chrono::vsg3d;
#endif

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;

using std::cout;
using std::endl;

typedef enum { ISO3888_1, ISO3888_2 } DLC_Variant;

// =============================================================================
// Wrapper class for an ISO 3888 double lane change maneuver
class ISO3888_Wrapper {
  public:
    ISO3888_Wrapper(double xmin, double acc_length, double vehicle_width, DLC_Variant variant, bool left_turn);
    ~ISO3888_Wrapper() {}

    bool GateTestLeft(ChVector3d& p);
    bool GateTestRight(ChVector3d& p);

    const std::vector<ChVector3d>& GetLeftConePositions() const { return m_leftCones; }
    const std::vector<ChVector3d>& GetRightConePositions() const { return m_rightCones; }

    double GetManeuverLength() { return m_lineL[5].x() - m_lineL[0].x(); }
    double GetXmax() { return m_lineL[5].x(); }
    std::shared_ptr<ChBezierCurve> GetPath() { return m_path; }

  private:
    std::vector<ChVector3d> m_lineL;
    std::vector<ChVector3d> m_lineC;
    std::vector<ChVector3d> m_lineR;
    std::vector<ChVector3d> m_leftCones;
    std::vector<ChVector3d> m_rightCones;
    double m_widthA;
    double m_lengthA;
    double m_widthB;
    double m_lengthB;
    double m_widthC;
    double m_lengthC;
    double m_lengthAB;
    double m_lengthBC;
    double m_ofsB;
    double m_ofsC;
    std::vector<ChVector3d> m_inCV;
    std::vector<ChVector3d> m_outCV;
    std::shared_ptr<ChBezierCurve> m_path;
};

// =============================================================================

// Run-time visualization system (IRRLICHT or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
VisualizationType chassis_vis_type = VisualizationType::NONE;
VisualizationType suspension_vis_type = VisualizationType::PRIMITIVES;
VisualizationType steering_vis_type = VisualizationType::PRIMITIVES;
VisualizationType wheel_vis_type = VisualizationType::NONE;
VisualizationType tire_vis_type = VisualizationType::MESH;

// Type of engine model (SHAFTS, SIMPLE, SIMPLE_MAP)
EngineModelType engine_model = EngineModelType::SHAFTS;

// Type of transmission model (SHAFTS, SIMPLE_MAP)
TransmissionModelType transmission_model = TransmissionModelType::AUTOMATIC_SHAFTS;

// Drive type (FWD, RWD, or AWD)
DrivelineTypeWV drive_type = DrivelineTypeWV::AWD;

// Type of tire model (RIGID, RIGID_MESH, FIALA, PAC89, PAC02, TMEASY, TMSIMPLE)
TireModelType tire_model = TireModelType::TMEASY;

// Terrain length (X direction)
double terrainLength = 300.0;
double accelerationLength = 200.0;
double terrainWidth = 16.0;

// Simulation step sizes
double step_size = 1e-3;
double tire_step_size = 1e-3;

// =============================================================================

void CreateSceneObjects(std::shared_ptr<ChVehicleVisualSystem> vis,
                        const ISO3888_Wrapper& dlc,
                        int& sentinelID,
                        int& targetID) {
    // Add visualization of controller points (sentinel & target)
    auto ballS = chrono_types::make_shared<ChVisualShapeSphere>(0.1);
    auto ballT = chrono_types::make_shared<ChVisualShapeSphere>(0.1);
    ballS->SetColor(ChColor(1, 0, 0));
    ballT->SetColor(ChColor(0, 1, 0));
    sentinelID = vis->AddVisualModel(ballS, ChFrame<>());
    targetID = vis->AddVisualModel(ballT, ChFrame<>());

    // Add the road cones
    ChVector3d cone_offset(0, 0.21, 0);
    auto cone = chrono_types::make_shared<ChVisualShapeModelFile>();
    cone->SetFilename(GetChronoDataFile("models/traffic_cone/trafficCone750mm.obj"));
    cone->SetColor(ChColor(0.8f, 0.8f, 0.8f));
    for (const auto& pos : dlc.GetLeftConePositions())
        vis->AddVisualModel(cone, ChFrame<>(pos + cone_offset));
    for (const auto& pos : dlc.GetRightConePositions())
        vis->AddVisualModel(cone, ChFrame<>(pos - cone_offset));
}

// =============================================================================

int main(int argc, char* argv[]) {
    cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n" << endl;

    DLC_Variant dlc_mode = ISO3888_1;
    int velkmh = 30;
    bool left_turn = true;

    switch (argc) {
        case 1:
        default:
            break;
        case 2:
            velkmh = ChClamp(atoi(argv[1]), 10, 100);
            break;
        case 3:
            velkmh = ChClamp(atoi(argv[1]), 10, 100);
            dlc_mode = atoi(argv[2]) == 1 ? ISO3888_1 : ISO3888_2;
            break;
        case 4:
            velkmh = ChClamp(atoi(argv[1]), 10, 100);
            dlc_mode = atoi(argv[2]) == 1 ? ISO3888_1 : ISO3888_2;
            left_turn = (argv[3][0] != '2') && (argv[3][0] != 'r') && (argv[3][0] != 'R');
            break;
    }

    std::string title = dlc_mode ? "ISO 3888-1 Double Lane Change Test" : "ISO 3888-2 Moose Test";
    title += " - v = " + std::to_string(velkmh) + " km/h";
    title += left_turn ? " - Left Turn" : " - Right Turn";

    double target_speed = velkmh / 3.6;  // [m/s]

    // --------------
    // Create systems
    // --------------

    // Create the HMMWV vehicle, set parameters, and initialize.
    // Typical aerodynamic drag for HMMWV: Cd = 0.5 and area ~5 m2
    HMMWV_Full hmmwv;
    hmmwv.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    hmmwv.SetContactMethod(ChContactMethod::SMC);
    hmmwv.SetChassisFixed(false);
    hmmwv.SetInitPosition(ChCoordsys<>(ChVector3d(-terrainLength / 2 + 5, 0, 0.7), ChQuaternion<>(1, 0, 0, 0)));
    hmmwv.SetEngineType(engine_model);
    hmmwv.SetTransmissionType(transmission_model);
    hmmwv.SetDriveType(drive_type);
    hmmwv.SetTireType(tire_model);
    hmmwv.SetTireStepSize(tire_step_size);
    hmmwv.SetAerodynamicDrag(0.5, 5.0, 1.2);
    hmmwv.Initialize();

    // important vehicle data
    double wheel_base = hmmwv.GetVehicle().GetWheelbase();
    double vehicle_width = 2.16;
    double steering_gear_ratio = 3.5 * 360.0 / 2;  // caution: estimated value 3.5 revolutions from left to right

    // Set subsystem visualization mode
    if (tire_model == TireModelType::RIGID_MESH)
        tire_vis_type = VisualizationType::MESH;

    hmmwv.SetChassisVisualizationType(chassis_vis_type);
    hmmwv.SetSuspensionVisualizationType(suspension_vis_type);
    hmmwv.SetSteeringVisualizationType(steering_vis_type);
    hmmwv.SetWheelVisualizationType(wheel_vis_type);
    hmmwv.SetTireVisualizationType(tire_vis_type);

    ISO3888_Wrapper dlc(-accelerationLength + 5.0, accelerationLength, vehicle_width, dlc_mode, left_turn);
    ////cout << "Maneuver Length = " << helper.GetManeuverLength() << " m" << endl;

    // Create the terrain
    RigidTerrain terrain(hmmwv.GetSystem());
    auto patch_mat = chrono_types::make_shared<ChContactMaterialSMC>();
    patch_mat->SetFriction(0.9f);
    patch_mat->SetRestitution(0.01f);
    patch_mat->SetYoungModulus(2e7f);
    patch_mat->SetPoissonRatio(0.3f);
    auto patch = terrain.AddPatch(patch_mat, CSYSNORM, terrainLength, terrainWidth);
    patch->SetColor(ChColor(0.8f, 0.8f, 0.5f));
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), (float)terrainLength, (float)terrainWidth);
    terrain.Initialize();

    // ---------------------------------------------------
    // Create the lane change path and the driver system
    // ---------------------------------------------------

    auto path = dlc.GetPath();
    ChPathFollowerDriver driver(hmmwv.GetVehicle(), path, "my_path", target_speed);
    driver.GetSteeringController().SetLookAheadDistance(5.0);
    driver.GetSteeringController().SetGains(0.5, 0, 0);
    driver.GetSpeedController().SetGains(0.4, 0, 0);
    driver.Initialize();

    // -------------------------------------------
    // Create the run-time visualization interface
    // -------------------------------------------

#ifndef CHRONO_IRRLICHT
    if (vis_type == ChVisualSystem::Type::IRRLICHT)
        vis_type = ChVisualSystem::Type::VSG;
#endif
#ifndef CHRONO_VSG
    if (vis_type == ChVisualSystem::Type::VSG)
        vis_type = ChVisualSystem::Type::IRRLICHT;
#endif

    int sentinelID = -1;
    int targetID = -1;
    std::shared_ptr<ChVehicleVisualSystem> vis;
    switch (vis_type) {
        case ChVisualSystem::Type::IRRLICHT: {
#ifdef CHRONO_IRRLICHT
            auto vis_irr = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
            vis_irr->SetWindowTitle(title);
            vis_irr->SetChaseCamera(ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5);
            vis_irr->Initialize();
            vis_irr->AddLightDirectional();
            vis_irr->AddSkyBox();
            vis_irr->AddLogo();
            vis_irr->AttachVehicle(&hmmwv.GetVehicle());
            CreateSceneObjects(vis_irr, dlc, sentinelID, targetID);

            vis = vis_irr;
#endif
            break;
        }
        default:
        case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
            auto vis_vsg = chrono_types::make_shared<ChWheeledVehicleVisualSystemVSG>();
            vis_vsg->SetWindowTitle(title);
            vis_vsg->SetChaseCamera(ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5);
            vis_vsg->AttachVehicle(&hmmwv.GetVehicle());
            CreateSceneObjects(vis_vsg, dlc, sentinelID, targetID);
            vis_vsg->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
            vis_vsg->SetShadows(true);
            vis_vsg->Initialize();

            vis = vis_vsg;
#endif
            break;
        }
    }

    // ---------------
    // Simulation loop
    // ---------------

    // Running average of vehicle speed
    utils::ChRunningAverage speed_filter(500);

    // Running average of vehicle lateral acceleration
    utils::ChButterworthLowpass accel_filter(4, step_size, 2.0);

    // Running average of vehicle steering wheel angle
    utils::ChButterworthLowpass steer_filter(4, step_size, 2.0);

    // Differentiate steering signal
    utils::ChFilterD ang_diff(step_size);

    // Record vehicle speed
    ChFunctionInterp speed_recorder;

    // Record lateral vehicle acceleration
    ChFunctionInterp accel_recorder;

    // Record lateral vehicle steering wheel angle
    ChFunctionInterp steer_recorder;

    // Record lateral vehicle steering wheel angular speed
    ChFunctionInterp angspeed_recorder;

    // Initialize simulation frame counter and simulation time
    int step_number = 0;

    while (vis->Run()) {
        double time = hmmwv.GetSystem()->GetChTime();
        double speed = speed_filter.Add(hmmwv.GetVehicle().GetSpeed());
        double accel =
            accel_filter.Filter(hmmwv.GetVehicle().GetPointAcceleration(ChVector3d(-wheel_base / 2, 0, 0)).y());

        speed_recorder.AddPoint(time, speed);
        accel_recorder.AddPoint(time, accel);

        ChVector3d pFrontLeft = hmmwv.GetVehicle().GetPointLocation(ChVector3d(0, vehicle_width / 2, 1));
        ChVector3d pRearLeft = hmmwv.GetVehicle().GetPointLocation(ChVector3d(-wheel_base, vehicle_width / 2, 1));
        ChVector3d pFrontRight = hmmwv.GetVehicle().GetPointLocation(ChVector3d(0, -vehicle_width / 2, 1));
        ChVector3d pRearRight = hmmwv.GetVehicle().GetPointLocation(ChVector3d(-wheel_base, -vehicle_width / 2, 1));
        if (!dlc.GateTestLeft(pFrontLeft)) {
            cout << "Test Failure: vehicle left the course with the front left wheel." << endl;
            break;
        }
        if (!dlc.GateTestLeft(pRearLeft)) {
            cout << "Test Failure: vehicle left the course with the rear left wheel." << endl;
            break;
        }
        if (!dlc.GateTestRight(pFrontRight)) {
            cout << "Test Failure: vehicle left the course with the front right wheel." << endl;
            cout << pFrontRight << endl;
            break;
        }
        if (!dlc.GateTestRight(pRearRight)) {
            cout << "Test Failure: vehicle left the course with the rear right wheel." << endl;
            break;
        }

        if (time >= 100 || hmmwv.GetVehicle().GetPos().x() > dlc.GetXmax()) {
            cout << "Test Run: terminated normally." << endl;
            break;
        }

        // Driver inputs
        DriverInputs driver_inputs = driver.GetInputs();

        double steer = steering_gear_ratio * steer_filter.Filter(driver_inputs.m_steering);
        steer_recorder.AddPoint(time, steer);
        double angspeed = ang_diff.Filter(steer);
        angspeed_recorder.AddPoint(time, angspeed);

        // Update sentinel and target location markers for the path-follower controller.
        vis->UpdateVisualModel(sentinelID, ChFrame<>(driver.GetSteeringController().GetSentinelLocation()));
        vis->UpdateVisualModel(targetID, ChFrame<>(driver.GetSteeringController().GetTargetLocation()));

        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        // Update modules (process inputs from other modules)
        driver.Synchronize(time);
        terrain.Synchronize(time);
        hmmwv.Synchronize(time, driver_inputs, terrain);
        vis->Synchronize(time, driver_inputs);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        terrain.Advance(step_size);
        hmmwv.Advance(step_size);
        vis->Advance(step_size);

        // Increment frame number
        step_number++;
    }

#ifdef CHRONO_POSTPROCESS
    const std::string out_dir = GetChronoOutputPath() + "DEMO_CONTROLLER_DLC";
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }


    std::string test_title = std::string(dlc_mode == ISO3888_1 ? "1" : "2") +
                             std::string(left_turn ? " left turn test" : " right turn test");

    postprocess::ChGnuPlot gplot(out_dir + "/speed.gpl");
    gplot.SetGrid();
    std::string speed_title = "Speed at ISO3888-" + test_title;
    gplot.SetTitle(speed_title);
    gplot.SetLabelX("time (s)");
    gplot.SetLabelY("speed (m/s)");
    gplot.Plot(speed_recorder, "", " with lines lt -1 lc rgb'#00AAEE' ");

    postprocess::ChGnuPlot gplot_acc(out_dir + "/lateral_acceleration.gpl");
    gplot_acc.SetGrid();
    std::string accel_title = "Lateral Acceleration at ISO3888-" + test_title;
    gplot_acc.SetTitle(accel_title);
    gplot_acc.SetLabelX("time (s)");
    gplot_acc.SetLabelY("lateral acceleration (m/s^2)");
    gplot_acc.Plot(accel_recorder, "", " with lines lt -1 lc rgb'#00AAEE' ");

    postprocess::ChGnuPlot gplot_steer(out_dir + "/steering_angle.gpl");
    gplot_steer.SetGrid();
    std::string steer_title = "Steering Wheel Angle at ISO3888-" + test_title;
    gplot_steer.SetTitle(steer_title);
    gplot_steer.SetLabelX("time (s)");
    gplot_steer.SetLabelY("steering wheel angle (degrees)");
    gplot_steer.Plot(steer_recorder, "", " with lines lt -1 lc rgb'#00AAEE' ");

    postprocess::ChGnuPlot gplot_angspeed(out_dir + "/steering_angular_vel.gpl");
    gplot_angspeed.SetGrid();
    std::string angspeed_title = "Steering Wheel Angular Speed at ISO3888-" + test_title;
    gplot_angspeed.SetTitle(angspeed_title);
    gplot_angspeed.SetLabelX("time (s)");
    gplot_angspeed.SetLabelY("steering wheel angle (degrees/s)");
    gplot_angspeed.Plot(angspeed_recorder, "", " with lines lt -1 lc rgb'#00AAEE' ");
#endif

    return 0;
}

// =============================================================================
// Implementation of ISO3888_Wrapper

ISO3888_Wrapper::ISO3888_Wrapper(double xmin,
                                 double acc_length,
                                 double vehicle_width,
                                 DLC_Variant variant,
                                 bool left_turn) {
    switch (variant) {
        default:
        case ISO3888_1:
            // gate A
            m_lengthA = 15.0;
            m_widthA = 1.1 * vehicle_width + 0.25;
            // gate B
            m_lengthAB = 30.0;
            m_lengthB = 25.0;
            m_widthB = 1.2 * vehicle_width + 0.25;
            m_ofsB = left_turn ? m_widthB / 2.0 + 3.5 : -(m_widthB / 2.0 + 3.5);
            // gate C
            m_lengthBC = 25.0;
            m_lengthC = 15.0;
            m_widthC = 1.3 * vehicle_width + 0.25;
            m_ofsC = left_turn ? (m_widthC - m_widthA) / 2 : (m_widthA - m_widthC) / 2;
            break;
        case ISO3888_2:
            // gate A
            m_lengthA = 12.0;
            m_widthA = 1.1 * vehicle_width + 0.25;
            // gate B
            m_lengthAB = 13.5;
            m_lengthB = 11.0;
            m_widthB = vehicle_width + 1.0;
            m_ofsB = left_turn ? m_widthB / 2.0 + m_widthA / 2.0 + 1.0 : -(m_widthB / 2.0 + m_widthA / 2.0 + 1.0);
            // gate C
            m_lengthBC = 12.5;
            m_lengthC = 12.0;
            m_widthC = 3.0;
            m_ofsC = left_turn ? (m_widthC - m_widthA) / 2 : (m_widthA - m_widthC) / 2;
            break;
    }

    // Set up definition points
    double zl = 0.1;
    // P1
    m_lineL.push_back({xmin + acc_length, m_widthA / 2.0, 0});
    m_lineC.push_back({xmin, 0, 0});
    m_lineR.push_back({xmin + acc_length, -m_widthA / 2.0, 0});
    // P2
    m_lineL.push_back({xmin + acc_length + m_lengthA, m_widthA / 2.0, 0});
    m_lineC.push_back({xmin + acc_length + m_lengthA, 0, zl});
    m_lineR.push_back({xmin + acc_length + m_lengthA, -m_widthA / 2.0, 0});
    // P3
    m_lineL.push_back({xmin + acc_length + m_lengthA + m_lengthAB, m_widthB / 2.0, 0});
    m_lineC.push_back({xmin + acc_length + m_lengthA + m_lengthAB, 0, zl});
    m_lineR.push_back({xmin + acc_length + m_lengthA + m_lengthAB, -m_widthB / 2.0, 0});
    m_lineL.back().y() += m_ofsB;
    m_lineC.back().y() += m_ofsB;
    m_lineR.back().y() += m_ofsB;
    // P4
    m_lineL.push_back({xmin + acc_length + m_lengthA + m_lengthAB + m_lengthB, m_widthB / 2.0, 0});
    m_lineC.push_back({xmin + acc_length + m_lengthA + m_lengthAB + m_lengthB, 0, zl});
    m_lineR.push_back({xmin + acc_length + m_lengthA + m_lengthAB + m_lengthB, -m_widthB / 2.0, 0});
    m_lineL.back().y() += m_ofsB;
    m_lineC.back().y() += m_ofsB;
    m_lineR.back().y() += m_ofsB;
    // P5
    m_lineL.push_back({xmin + acc_length + m_lengthA + m_lengthAB + m_lengthB + m_lengthBC, m_widthC / 2.0, 0});
    m_lineC.push_back({xmin + acc_length + m_lengthA + m_lengthAB + m_lengthB + m_lengthBC, 0, zl});
    m_lineR.push_back({xmin + acc_length + m_lengthA + m_lengthAB + m_lengthB + m_lengthBC, -m_widthC / 2.0, 0});
    m_lineL.back().y() += m_ofsC;
    m_lineC.back().y() += m_ofsC;
    m_lineR.back().y() += m_ofsC;
    // P6
    m_lineL.push_back(
        {xmin + acc_length + m_lengthA + m_lengthAB + m_lengthB + m_lengthBC + m_lengthC, m_widthC / 2.0, 0});
    m_lineC.push_back({xmin + acc_length + m_lengthA + m_lengthAB + m_lengthB + m_lengthBC + m_lengthC, 0, zl});
    m_lineR.push_back(
        {xmin + acc_length + m_lengthA + m_lengthAB + m_lengthB + m_lengthBC + m_lengthC, -m_widthC / 2.0, 0});
    m_lineL.back().y() += m_ofsC;
    m_lineC.back().x() += 100.0;
    m_lineC.back().y() += m_ofsC;
    m_lineR.back().y() += m_ofsC;

    // Add some road cone positions like in the standard
    m_leftCones.push_back(m_lineL[0]);
    m_leftCones.push_back((m_lineL[0] + m_lineL[1]) / 2);
    m_leftCones.push_back(m_lineL[1]);
    m_leftCones.push_back(m_lineL[2]);
    m_leftCones.push_back((m_lineL[2] + m_lineL[3]) / 2);
    m_leftCones.push_back(m_lineL[3]);
    m_leftCones.push_back(m_lineL[4]);
    m_leftCones.push_back((m_lineL[4] + m_lineL[5]) / 2);
    m_leftCones.push_back(m_lineL[5]);

    m_rightCones.push_back(m_lineR[0]);
    m_rightCones.push_back((m_lineR[0] + m_lineR[1]) / 2);
    m_rightCones.push_back(m_lineR[1]);
    m_rightCones.push_back(m_lineR[2]);
    m_rightCones.push_back((m_lineR[2] + m_lineR[3]) / 2);
    m_rightCones.push_back(m_lineR[3]);
    m_rightCones.push_back(m_lineR[4]);
    m_rightCones.push_back((m_lineR[4] + m_lineR[5]) / 2);
    m_rightCones.push_back(m_lineR[5]);

    // Prepare path spline definition
    ChVector3d offset(m_lengthB * CH_1_3, 0, 0);
    for (size_t i = 0; i < m_lineC.size(); i++) {
        m_inCV.push_back(m_lineC[i] - offset);
        m_outCV.push_back(m_lineC[i] + offset);
    }
    m_path = chrono_types::make_shared<ChBezierCurve>(m_lineC, m_inCV, m_outCV);
}

bool ISO3888_Wrapper::GateTestLeft(ChVector3d& p) {
    if (p.x() >= m_lineL[0].x() && p.x() <= m_lineL[1].x())
        return p.y() <= m_lineL[0].y();
    if (p.x() >= m_lineL[2].x() && p.x() <= m_lineL[3].x())
        return p.y() <= m_lineL[2].y();
    if (p.x() >= m_lineL[4].x() && p.x() <= m_lineL[5].x())
        return p.y() <= m_lineL[4].y();
    return true;
}

bool ISO3888_Wrapper::GateTestRight(ChVector3d& p) {
    if (p.x() >= m_lineL[0].x() && p.x() <= m_lineL[1].x())
        return p.y() >= m_lineR[0].y();
    if (p.x() >= m_lineL[2].x() && p.x() <= m_lineL[3].x())
        return p.y() >= m_lineR[2].y();
    if (p.x() >= m_lineL[4].x() && p.x() <= m_lineL[5].x())
        return p.y() >= m_lineR[4].y();
    return true;
}
