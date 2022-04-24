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
// =============================================================================

#include <cmath>

#include "chrono/ChConfig.h"
#include "chrono/utils/ChFilters.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/core/ChTimer.h"
#include "chrono/assets/ChBoxShape.h"
#include "chrono/assets/ChConeShape.h"
#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/utils/ChVehiclePath.h"
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h"

#include "chrono_models/vehicle/hmmwv/HMMWV.h"

#ifdef CHRONO_POSTPROCESS
    #include "chrono_postprocess/ChGnuPlot.h"
#endif

using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::irrlicht;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;

typedef enum { ISO3888_1, ISO3888_2 } DLC_Variant;

// =============================================================================
// ISO 3888 helper class
class ISO3888_Helper {
  public:
    ISO3888_Helper(double xmin, double acc_length, double vehicle_width, DLC_Variant variant, bool left_turn = true) {
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
                if (left_turn) {
                    m_ofsB = m_widthB / 2.0 + 3.5;
                } else {
                    m_ofsB = -(m_widthB / 2.0 + 3.5);
                }
                // gate C
                m_lengthBC = 25.0;
                m_lengthC = 15.0;
                m_widthC = 1.3 * vehicle_width + 0.25;
                if (left_turn) {
                    m_ofsC = (m_widthC - m_widthA) / 2;
                } else {
                    m_ofsC = (m_widthA - m_widthC) / 2;
                }
                break;
            case ISO3888_2:
                // gate A
                m_lengthA = 12.0;
                m_widthA = 1.1 * vehicle_width + 0.25;
                // gate B
                m_lengthAB = 13.5;
                m_lengthB = 11.0;
                m_widthB = vehicle_width + 1.0;
                if (left_turn) {
                    m_ofsB = m_widthB / 2.0 + m_widthA / 2.0 + 1.0;
                } else {
                    m_ofsB = -(m_widthB / 2.0 + m_widthA / 2.0 + 1.0);
                }
                // gate C
                m_lengthBC = 12.5;
                m_lengthC = 12.0;
                m_widthC = 3.0;
                if (left_turn) {
                    m_ofsC = (m_widthC - m_widthA) / 2;
                } else {
                    m_ofsC = (m_widthA - m_widthC) / 2;
                }
                break;
        }
        //================== setup definition points ========================================
        double zl = 0.1;
        // P1
        m_leftLine.push_back(ChVector<>(xmin + acc_length, m_widthA / 2.0, 0));
        m_centerLine.push_back(ChVector<>(xmin, 0, 0));
        m_rightLine.push_back(ChVector<>(xmin + acc_length, -m_widthA / 2.0, 0));
        // P2
        m_leftLine.push_back(ChVector<>(xmin + acc_length + m_lengthA, m_widthA / 2.0, 0));
        m_centerLine.push_back(ChVector<>(xmin + acc_length + m_lengthA, 0, zl));
        m_rightLine.push_back(ChVector<>(xmin + acc_length + m_lengthA, -m_widthA / 2.0, 0));
        // P3
        m_leftLine.push_back(ChVector<>(xmin + acc_length + m_lengthA + m_lengthAB, m_widthB / 2.0, 0));
        m_centerLine.push_back(ChVector<>(xmin + acc_length + m_lengthA + m_lengthAB, 0, zl));
        m_rightLine.push_back(ChVector<>(xmin + acc_length + m_lengthA + m_lengthAB, -m_widthB / 2.0, 0));
        m_leftLine.back().y() += m_ofsB;
        m_centerLine.back().y() += m_ofsB;
        m_rightLine.back().y() += m_ofsB;
        // P4
        m_leftLine.push_back(ChVector<>(xmin + acc_length + m_lengthA + m_lengthAB + m_lengthB, m_widthB / 2.0, 0));
        m_centerLine.push_back(ChVector<>(xmin + acc_length + m_lengthA + m_lengthAB + m_lengthB, 0, zl));
        m_rightLine.push_back(ChVector<>(xmin + acc_length + m_lengthA + m_lengthAB + m_lengthB, -m_widthB / 2.0, 0));
        m_leftLine.back().y() += m_ofsB;
        m_centerLine.back().y() += m_ofsB;
        m_rightLine.back().y() += m_ofsB;
        // P5
        m_leftLine.push_back(
            ChVector<>(xmin + acc_length + m_lengthA + m_lengthAB + m_lengthB + m_lengthBC, m_widthC / 2.0, 0));
        m_centerLine.push_back(ChVector<>(xmin + acc_length + m_lengthA + m_lengthAB + m_lengthB + m_lengthBC, 0, zl));
        m_rightLine.push_back(
            ChVector<>(xmin + acc_length + m_lengthA + m_lengthAB + m_lengthB + m_lengthBC, -m_widthC / 2.0, 0));
        m_leftLine.back().y() += m_ofsC;
        m_centerLine.back().y() += m_ofsC;
        m_rightLine.back().y() += m_ofsC;
        // P6
        m_leftLine.push_back(ChVector<>(xmin + acc_length + m_lengthA + m_lengthAB + m_lengthB + m_lengthBC + m_lengthC,
                                        m_widthC / 2.0, 0));
        m_centerLine.push_back(
            ChVector<>(xmin + acc_length + m_lengthA + m_lengthAB + m_lengthB + m_lengthBC + m_lengthC, 0, zl));
        m_rightLine.push_back(ChVector<>(
            xmin + acc_length + m_lengthA + m_lengthAB + m_lengthB + m_lengthBC + m_lengthC, -m_widthC / 2.0, 0));
        m_leftLine.back().y() += m_ofsC;
        m_centerLine.back().x() += 100.0;
        m_centerLine.back().y() += m_ofsC;
        m_rightLine.back().y() += m_ofsC;
        // luxury: add some road cone positions like in the standard
        m_leftCones.push_back(m_leftLine[0]);
        m_leftCones.push_back((m_leftLine[0] + m_leftLine[1]) / 2);
        m_leftCones.push_back(m_leftLine[1]);
        m_leftCones.push_back(m_leftLine[2]);
        m_leftCones.push_back((m_leftLine[2] + m_leftLine[3]) / 2);
        m_leftCones.push_back(m_leftLine[3]);
        m_leftCones.push_back(m_leftLine[4]);
        m_leftCones.push_back((m_leftLine[4] + m_leftLine[5]) / 2);
        m_leftCones.push_back(m_leftLine[5]);

        m_rightCones.push_back(m_rightLine[0]);
        m_rightCones.push_back((m_rightLine[0] + m_rightLine[1]) / 2);
        m_rightCones.push_back(m_rightLine[1]);
        m_rightCones.push_back(m_rightLine[2]);
        m_rightCones.push_back((m_rightLine[2] + m_rightLine[3]) / 2);
        m_rightCones.push_back(m_rightLine[3]);
        m_rightCones.push_back(m_rightLine[4]);
        m_rightCones.push_back((m_rightLine[4] + m_rightLine[5]) / 2);
        m_rightCones.push_back(m_rightLine[5]);

        ////std::ofstream tst("bla.txt");
        ////for (int i = 0; i < m_leftLine.size(); i++) {
        ////    tst << m_leftLine[i].x() << "\t" << centerLine[i].x() << "\t" << m_leftLine[i].y() << "\t" <<
        ///centerLine[i].y() /        << "\t" << m_rightLine[i].y() << std::endl;
        ////}
        ////tst.close();

        // prepare path spline definition
        ChVector<> offset(m_lengthB / 3, 0, 0);
        for (size_t i = 0; i < m_centerLine.size(); i++) {
            m_inCV.push_back(m_centerLine[i] - offset);
            m_outCV.push_back(m_centerLine[i] + offset);
        }
        m_path = chrono_types::make_shared<ChBezierCurve>(m_centerLine, m_inCV, m_outCV);
    }

    bool GateTestLeft(ChVector<>& p) {
        if (p.x() >= m_leftLine[0].x() && p.x() <= m_leftLine[1].x()) {
            if (p.y() > m_leftLine[0].y())
                return false;
        }
        if (p.x() >= m_leftLine[2].x() && p.x() <= m_leftLine[3].x()) {
            if (p.y() > m_leftLine[2].y())
                return false;
        }
        if (p.x() >= m_leftLine[4].x() && p.x() <= m_leftLine[5].x()) {
            if (p.y() > m_leftLine[4].y())
                return false;
        }
        return true;
    }

    bool GateTestRight(ChVector<>& p) {
        if (p.x() >= m_leftLine[0].x() && p.x() <= m_leftLine[1].x()) {
            if (p.y() < m_rightLine[0].y())
                return false;
        }
        if (p.x() >= m_leftLine[2].x() && p.x() <= m_leftLine[3].x()) {
            if (p.y() < m_rightLine[2].y())
                return false;
        }
        if (p.x() >= m_leftLine[4].x() && p.x() <= m_leftLine[5].x()) {
            if (p.y() < m_rightLine[4].y())
                return false;
        }
        return true;
    }

    size_t GetConePositions() { return m_leftCones.size(); }
    ChVector<>& GetConePosition(size_t idx, bool left) {
        if (left)
            return m_leftCones[idx];
        else
            return m_rightCones[idx];
    }

    ~ISO3888_Helper() {}

    double GetManeuverLength() { return m_leftLine[5].x() - m_leftLine[0].x(); }
    double GetXmax() { return m_leftLine[5].x(); }
    std::shared_ptr<ChBezierCurve> GetPath() { return m_path; }

  private:
    std::vector<ChVector<>> m_leftLine;
    std::vector<ChVector<>> m_centerLine;
    std::vector<ChVector<>> m_rightLine;
    std::vector<ChVector<>> m_leftCones;
    std::vector<ChVector<>> m_rightCones;
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
    std::vector<ChVector<>> m_inCV;
    std::vector<ChVector<>> m_outCV;
    std::shared_ptr<ChBezierCurve> m_path;
};
// =============================================================================

// Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
VisualizationType chassis_vis_type = VisualizationType::PRIMITIVES;
VisualizationType suspension_vis_type = VisualizationType::PRIMITIVES;
VisualizationType steering_vis_type = VisualizationType::PRIMITIVES;
VisualizationType wheel_vis_type = VisualizationType::NONE;
VisualizationType tire_vis_type = VisualizationType::PRIMITIVES;

// Type of powertrain model (SHAFTS, SIMPLE, SIMPLE_CVT)
PowertrainModelType powertrain_model = PowertrainModelType::SHAFTS;

// Drive type (FWD, RWD, or AWD)
DrivelineTypeWV drive_type = DrivelineTypeWV::AWD;

// Type of tire model (RIGID, RIGID_MESH, PACEJKA, LUGRE, FIALA, PAC89, PAC02, TMEASY)
TireModelType tire_model = TireModelType::PAC02;

// Terrain length (X direction)
double terrainLength = 300.0;
double accelerationLength = 200.0;
double terrainWidth = 16.0;

// Simulation step sizes
double step_size = 1e-3;
double tire_step_size = 1e-3;

// =============================================================================
// call variants:
//                          1) demo_VEH_HMMWV_DoubleLaneChange
//                              -> standard demo velkmh= 30km/h, ISO3888-1, left turn
//
//                          2) demo_VEH_HMMWV_DoubleLaneChange VelKMH
//                              -> velkmh= VelKMH, ISO3888-1, left turn
//
//                          3) demo_VEH_HMMWV_DoubleLaneChange VelKMH [1|2]
//                              -> velkmh= VelKMH, ISO variant set to 1 or 2, left turn
//
//                          4) demo_VEH_HMMWV_DoubleLaneChange VelKMH [1|2] [L|R]
//                              -> velkmh= VelKMH, ISO variant set to 1 or 2, turn set to left or right
//  on the Mac:
//  instead of 'demo_VEH_HMMWV_DoubleLaneChange' use 'run_app.sh demo_VEH_HMMWV_DoubleLaneChange.app'

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    std::wstring wtitle;
    DLC_Variant dlc_mode = ISO3888_1;
    int velkmh = 30;
    bool left_turn = true;
    switch (argc) {
        case 1:
        default:
            // no parameters given
            if (left_turn) {
                wtitle = L"ISO 3888-1 Double Lane Change Test v = " + std::to_wstring(velkmh) + L" km/h - Left Turn";

            } else {
                wtitle = L"ISO 3888-1 Double Lane Change Test v = " + std::to_wstring(velkmh) + L" km/h - Right Turn";
            }
            break;
        case 2:
            // one parameter given (velkmh)
            velkmh = ChClamp(atoi(argv[1]), 10, 100);
            if (left_turn) {
                wtitle = L"ISO 3888-1 Double Lane Change Test v = " + std::to_wstring(velkmh) + L" km/h - Left Turn";

            } else {
                wtitle = L"ISO 3888-1 Double Lane Change Test v = " + std::to_wstring(velkmh) + L" km/h - Right Turn";
            }
            break;
        case 3:
            // two parameters given (velkmh,dlc_mode)
            velkmh = ChClamp(atoi(argv[1]), 10, 100);
            switch (ChClamp(atoi(argv[2]), 1, 2)) {
                default:
                case 1:
                    dlc_mode = ISO3888_1;
                    break;
                case 2:
                    dlc_mode = ISO3888_2;
                    break;
            }
            if (dlc_mode == ISO3888_1) {
                wtitle = L"ISO 3888-1 Double Lane Change Test v = " + std::to_wstring(velkmh) + L" km/h - ";
            } else {
                wtitle = L"ISO 3888-2 Moose Test v = " + std::to_wstring(velkmh) + L" km/h - ";
            }
            if (left_turn) {
                wtitle += L" Left Turn";

            } else {
                wtitle += L" Right Turn";
            }
            break;
        case 4:
            // three parameters given (velkmh,dlc_mode,turn_mode)
            velkmh = ChClamp(atoi(argv[1]), 10, 100);
            switch (ChClamp(atoi(argv[2]), 1, 2)) {
                default:
                case 1:
                    dlc_mode = ISO3888_1;
                    break;
                case 2:
                    dlc_mode = ISO3888_2;
                    break;
            }
            switch (argv[3][0]) {
                case '2':
                case 'r':
                case 'R':
                    left_turn = false;
                    break;
                default:
                    left_turn = true;
            }
            if (dlc_mode == ISO3888_1) {
                wtitle = L"ISO 3888-1 Double Lane Change Test v = " + std::to_wstring(velkmh) + L" km/h - ";
            } else {
                wtitle = L"ISO 3888-2 Moose Test v = " + std::to_wstring(velkmh) + L" km/h - ";
            }
            if (left_turn) {
                wtitle += L" Left Turn";

            } else {
                wtitle += L" Right Turn";
            }
            break;
    }
    double target_speed = velkmh / 3.6;
    // --------------
    // Create systems
    // --------------

    // Create the HMMWV vehicle, set parameters, and initialize.
    // Typical aerodynamic drag for HMMWV: Cd = 0.5 and area ~5 m2
    HMMWV_Full my_hmmwv;
    my_hmmwv.SetContactMethod(ChContactMethod::SMC);
    my_hmmwv.SetChassisFixed(false);
    my_hmmwv.SetInitPosition(ChCoordsys<>(ChVector<>(-terrainLength / 2 + 5, 0, 0.7), ChQuaternion<>(1, 0, 0, 0)));
    my_hmmwv.SetPowertrainType(powertrain_model);
    my_hmmwv.SetDriveType(drive_type);
    my_hmmwv.SetTireType(tire_model);
    my_hmmwv.SetTireStepSize(tire_step_size);
    my_hmmwv.SetAerodynamicDrag(0.5, 5.0, 1.2);
    my_hmmwv.Initialize();

    // important vehicle data
    double wheel_base = my_hmmwv.GetVehicle().GetWheelbase();
    double vehicle_width = 2.16;
    double steering_gear_ratio = 3.5 * 360.0 / 2;  // caution: estimated value 3.5 revolutions from left to right

    // Set subsystem visualization mode
    if (tire_model == TireModelType::RIGID_MESH)
        tire_vis_type = VisualizationType::MESH;

    my_hmmwv.SetChassisVisualizationType(chassis_vis_type);
    my_hmmwv.SetSuspensionVisualizationType(suspension_vis_type);
    my_hmmwv.SetSteeringVisualizationType(steering_vis_type);
    my_hmmwv.SetWheelVisualizationType(wheel_vis_type);
    my_hmmwv.SetTireVisualizationType(tire_vis_type);

    ISO3888_Helper helper(-accelerationLength + 5.0, accelerationLength, vehicle_width, dlc_mode, left_turn);
    ////GetLog() << "Maneuver Length = " << helper.GetManeuverLength() << " m\n";

    // Create the terrain
    RigidTerrain terrain(my_hmmwv.GetSystem());
    auto patch_mat = chrono_types::make_shared<ChMaterialSurfaceSMC>();
    patch_mat->SetFriction(0.9f);
    patch_mat->SetRestitution(0.01f);
    patch_mat->SetYoungModulus(2e7f);
    patch_mat->SetPoissonRatio(0.3f);
    auto patch = terrain.AddPatch(patch_mat, ChVector<>(0, 0, 0), ChVector<>(0, 0, 1), terrainLength, terrainWidth);
    patch->SetColor(ChColor(0.8f, 0.8f, 0.5f));
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), (float)terrainLength, (float)terrainWidth);
    terrain.Initialize();

    // Create the vehicle Irrlicht interface
    ChWheeledVehicleIrrApp app(&my_hmmwv.GetVehicle(), wtitle.c_str());
    app.AddTypicalLights();
    app.SetChaseCamera(ChVector<>(0.0, 0.0, 1.75), 6.0, 0.5);
    app.SetTimestep(step_size);

    // ---------------------------------------------------
    // Create the lane change path and the driver system
    // ---------------------------------------------------

    auto path = helper.GetPath();
    ChPathFollowerDriver driver(my_hmmwv.GetVehicle(), path, "my_path", target_speed);
    driver.GetSteeringController().SetLookAheadDistance(5.0);
    driver.GetSteeringController().SetGains(0.5, 0, 0);
    driver.GetSpeedController().SetGains(0.4, 0, 0);
    driver.Initialize();

    // now we can add the road cones
    double coneBaseWidth = 0.415;
    for (size_t i = 0; i < helper.GetConePositions(); i++) {
        ChVector<> pL = helper.GetConePosition(i, true) + ChVector<>(0, coneBaseWidth / 2, 0);
        irr::scene::IAnimatedMesh* mesh_coneL =
            app.GetSceneManager()->getMesh(GetChronoDataFile("models/traffic_cone/trafficCone750mm.obj").c_str());
        irr::scene::IAnimatedMeshSceneNode* node_coneL = app.GetSceneManager()->addAnimatedMeshSceneNode(mesh_coneL);
        node_coneL->getMaterial(0).EmissiveColor = irr::video::SColor(0, 100, 0, 0);
        node_coneL->setPosition(irr::core::vector3dfCH(pL));

        ChVector<> pR = helper.GetConePosition(i, false) + ChVector<>(0, -coneBaseWidth / 2, 0);
        irr::scene::IAnimatedMesh* mesh_coneR =
            app.GetSceneManager()->getMesh(GetChronoDataFile("models/traffic_cone/trafficCone750mm.obj").c_str());
        irr::scene::IAnimatedMeshSceneNode* node_coneR = app.GetSceneManager()->addAnimatedMeshSceneNode(mesh_coneR);
        node_coneR->getMaterial(0).EmissiveColor = irr::video::SColor(0, 0, 100, 0);
        node_coneR->setPosition(irr::core::vector3dfCH(pR));
    }

    // ---------------------------------------------
    // Finalize construction of visualization assets
    // ---------------------------------------------

    app.AssetBindAll();
    app.AssetUpdateAll();

    // ---------------
    // Simulation loop
    // ---------------

    // Running average of vehicle speed
    utils::ChRunningAverage speed_filter(500);

    // Running average of vehicle lateral acceleration
    utils::ChButterworth_Lowpass accel_filter(4, step_size, 2.0);

    // Running average of vehicle steering wheel angle
    utils::ChButterworth_Lowpass steer_filter(4, step_size, 2.0);

    // Differentiate steering signal
    utils::ChFilterD ang_diff(step_size);

    // Record vehicle speed
    ChFunction_Recorder speed_recorder;

    // Record lateral vehicle acceleration
    ChFunction_Recorder accel_recorder;

    // Record lateral vehicle steering wheel angle
    ChFunction_Recorder steer_recorder;

    // Record lateral vehicle steering wheel angular speed
    ChFunction_Recorder angspeed_recorder;

    // Initialize simulation frame counter and simulation time
    int step_number = 0;
    double time = 0;
    double xpos = 0;
    double xend = helper.GetXmax();
    while (app.GetDevice()->run()) {
        time = my_hmmwv.GetSystem()->GetChTime();
        double speed = speed_filter.Add(my_hmmwv.GetVehicle().GetSpeed());
        double accel = accel_filter.Filter(
            my_hmmwv.GetVehicle().GetPointAcceleration(ChVector<>(-wheel_base / 2, 0, 0)).y());

        speed_recorder.AddPoint(time, speed);
        accel_recorder.AddPoint(time, accel);
        xpos = my_hmmwv.GetVehicle().GetPos().x();
        ChVector<> pFrontLeft = my_hmmwv.GetVehicle().GetPointLocation(ChVector<>(0, vehicle_width / 2, 1));
        ChVector<> pRearLeft = my_hmmwv.GetVehicle().GetPointLocation(ChVector<>(-wheel_base, vehicle_width / 2, 1));
        ChVector<> pFrontRight = my_hmmwv.GetVehicle().GetPointLocation(ChVector<>(0, -vehicle_width / 2, 1));
        ChVector<> pRearRight = my_hmmwv.GetVehicle().GetPointLocation(ChVector<>(-wheel_base, -vehicle_width / 2, 1));
        if (!helper.GateTestLeft(pFrontLeft)) {
            GetLog() << "Test Failure: vehicle left the course with the front left wheel.\n";
            break;
        }
        if (!helper.GateTestLeft(pRearLeft)) {
            GetLog() << "Test Failure: vehicle left the course with the rear left wheel.\n";
            break;
        }
        if (!helper.GateTestRight(pFrontRight)) {
            GetLog() << "Test Failure: vehicle left the course with the front right wheel.\n";
            break;
        }
        if (!helper.GateTestRight(pRearRight)) {
            GetLog() << "Test Failure: vehicle left the course with the rear right wheel.\n";
            break;
        }
        // End simulation
        if (time >= 100 || xpos > xend) {
            GetLog() << "Test Run: terminated normally.\n";
            break;
        }

        app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
        app.DrawAll();

        // Driver inputs
        ChDriver::Inputs driver_inputs = driver.GetInputs();

        double steer = steering_gear_ratio * steer_filter.Filter(driver_inputs.m_steering);
        steer_recorder.AddPoint(time, steer);
        double angspeed = ang_diff.Filter(steer);
        angspeed_recorder.AddPoint(time, angspeed);

        // Update modules (process inputs from other modules)
        driver.Synchronize(time);
        terrain.Synchronize(time);
        my_hmmwv.Synchronize(time, driver_inputs, terrain);
        app.Synchronize("Double lane change test", driver_inputs);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        terrain.Advance(step_size);
        my_hmmwv.Advance(step_size);
        app.Advance(step_size);

        // Increment frame number
        step_number++;

        app.EndScene();
    }

#ifdef CHRONO_POSTPROCESS

    postprocess::ChGnuPlot gplot;
    gplot.SetGrid();
    std::string speed_title = "Speed at ISO3888-";
    if (dlc_mode == ISO3888_1) {
        speed_title.append("1");
    } else {
        speed_title.append("2");
    }
    if (left_turn) {
        speed_title.append(" left turn test");
    } else {
        speed_title.append(" right turn test");
    }
    gplot.SetTitle(speed_title.c_str());
    gplot.SetLabelX("time (s)");
    gplot.SetLabelY("speed (m/s)");
    gplot.Plot(speed_recorder, "", " with lines lt -1 lc rgb'#00AAEE' ");

    postprocess::ChGnuPlot gplot_acc("_tmp2_gnuplot.gpl");
    gplot_acc.SetGrid();
    std::string accel_title = "Lateral Acceleration at ISO3888-";
    if (dlc_mode == ISO3888_1) {
        accel_title.append("1");
    } else {
        accel_title.append("2");
    }
    if (left_turn) {
        accel_title.append(" left turn test");
    } else {
        accel_title.append(" right turn test");
    }
    gplot_acc.SetTitle(accel_title.c_str());
    gplot_acc.SetLabelX("time (s)");
    gplot_acc.SetLabelY("lateral acceleration (m/s^2)");
    gplot_acc.Plot(accel_recorder, "", " with lines lt -1 lc rgb'#00AAEE' ");

    postprocess::ChGnuPlot gplot_steer("_tmp3_gnuplot.gpl");
    gplot_steer.SetGrid();
    std::string steer_title = "Steering Wheel Angle at ISO3888-";
    if (dlc_mode == ISO3888_1) {
        steer_title.append("1");
    } else {
        steer_title.append("2");
    }
    if (left_turn) {
        steer_title.append(" left turn test");
    } else {
        steer_title.append(" right turn test");
    }
    gplot_steer.SetTitle(steer_title.c_str());
    gplot_steer.SetLabelX("time (s)");
    gplot_steer.SetLabelY("steering wheel angle (degrees)");
    gplot_steer.Plot(steer_recorder, "", " with lines lt -1 lc rgb'#00AAEE' ");

    postprocess::ChGnuPlot gplot_angspeed("_tmp4_gnuplot.gpl");
    gplot_angspeed.SetGrid();
    std::string angspeed_title = "Steering Wheel Angular Speed at ISO3888-";
    if (dlc_mode == ISO3888_1) {
        angspeed_title.append("1");
    } else {
        angspeed_title.append("2");
    }
    if (left_turn) {
        angspeed_title.append(" left turn test");
    } else {
        angspeed_title.append(" right turn test");
    }
    gplot_angspeed.SetTitle(angspeed_title.c_str());
    gplot_angspeed.SetLabelX("time (s)");
    gplot_angspeed.SetLabelY("steering wheel angle (degrees/s)");
    gplot_angspeed.Plot(angspeed_recorder, "", " with lines lt -1 lc rgb'#00AAEE' ");
#endif

    return 0;
}
