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
// Authors: Radu Serban, Rainer Gericke
// =============================================================================
//
// Tool to estimate the 6W absorbed power speed for a vehicle.
//
// The default world frame is ISO (Z up, X forward, Y to the left).
//
// =============================================================================

#include <iomanip>

#include "chrono/physics/ChSystemSMC.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/terrain/CRGTerrain.h"

#include "chrono_models/vehicle/feda/FEDA.h"

#include "chrono_postprocess/ChGnuPlot.h"

#include "chrono_thirdparty/cxxopts/ChCLI.h"
#include "chrono_thirdparty/filesystem/path.h"

#ifdef CHRONO_IRRLICHT
    #include "chrono_vehicle/driver/ChInteractiveDriverIRR.h"
    #include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"
using namespace chrono::irrlicht;
#endif

#ifdef CHRONO_VSG
    #include "chrono_vehicle/driver/ChInteractiveDriverVSG.h"
    #include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemVSG.h"
using namespace chrono::vsg3d;
#endif

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::feda;

using std::cout;
using std::endl;

// =============================================================================
// Problem parameters

// Run-time visualization system (IRRLICHT, VSG, or NONE)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::NONE;

enum class DriverModelType {
    PID,      // pure PID lateral controller with constant speed controller
    STANLEY,  // geometrical P heading and PID lateral controller with constant speed controller
    SR        // alternative PID lateral controller with constant speed controller
};

// Type of tire model (RIGID, PAC02, TMSIMPLE, TMEASY)
TireModelType tire_model = TireModelType::PAC02;

// Tire pressure (PSI)
double tire_pressure_psi = 35.0;

// Road visualization (mesh or boundary lines)
bool useMesh = true;

// Simulation step size
double step_size = 1e-3;
double tire_step_size = 1e-3;

// Output directory
const std::string out_dir = GetChronoOutputPath() + "RIDETOOL_DEMO";

// Rendering frequency (FPS)
double fps = 60;

// =============================================================================

const double mph_to_mps = 0.44704;
const double mps_to_mph = 1.0 / mph_to_mps;
const double psi_to_pascal = 6894.76;

// =============================================================================

// Wrapper around a driver system of specified type
class MyDriver {
  public:
    MyDriver(DriverModelType type, ChWheeledVehicle& vehicle, double target_speed, std::shared_ptr<ChBezierCurve> path)
        : m_steering_controller(nullptr) {
        switch (type) {
            case DriverModelType::PID: {
                auto driverPID = chrono_types::make_shared<ChPathFollowerDriver>(vehicle, path, "path", target_speed);
                driverPID->GetSteeringController().SetLookAheadDistance(5);
                driverPID->GetSteeringController().SetGains(0.5, 0, 0);
                driverPID->GetSpeedController().SetGains(0.4, 0, 0);

                m_driver_type = "PID";
                m_driver = driverPID;
                m_steering_controller = &driverPID->GetSteeringController();
                break;
            }
            case DriverModelType::STANLEY: {
                auto driverStanley = chrono_types::make_shared<ChPathFollowerDriverStanley>(
                    vehicle, path, "path", target_speed, vehicle.GetMaxSteeringAngle());
                driverStanley->GetSteeringController().SetLookAheadDistance(5.0);
                driverStanley->GetSteeringController().SetGains(0.5, 0.0, 0.0);
                driverStanley->GetSpeedController().SetGains(0.4, 0, 0);

                m_driver_type = "STANLEY";
                m_driver = driverStanley;
                m_steering_controller = &driverStanley->GetSteeringController();
                break;
            }
            case DriverModelType::SR: {
                auto driverSR = chrono_types::make_shared<ChPathFollowerDriverSR>(vehicle, path, "path", target_speed,
                                                                                  vehicle.GetMaxSteeringAngle(), 3.2);
                driverSR->GetSteeringController().SetGains(0.1, 5);
                driverSR->GetSteeringController().SetPreviewTime(0.5);
                driverSR->GetSpeedController().SetGains(0.4, 0, 0);

                m_driver_type = "SR";
                m_driver = driverSR;
                m_steering_controller = &driverSR->GetSteeringController();
                break;
            }
        }
    }

    DriverInputs GetInputs() { return m_driver->GetInputs(); }
    void Initialize() { m_driver->Initialize(); }
    void Synchronize(double time) { m_driver->Synchronize(time); }
    void Advance(double step) { m_driver->Advance(step); }

    const std::string& GetDriverType() { return m_driver_type; }

    ChVector3d GetTargetLocation() { return m_steering_controller->GetTargetLocation(); }

    ChVector3d GetSentinelLocation() { return m_steering_controller->GetSentinelLocation(); }

  private:
    std::string m_driver_type;
    std::shared_ptr<ChDriver> m_driver;
    ChSteeringController* m_steering_controller;
};

// =============================================================================
// Run a single test at specified speed.
// Return the average speed and absorbed power.

void RunTest(double speed,
             DriverModelType driver_type,
             const std::string& crg_road_file,
             double rms,
             double& avg_vel,
             double& abs_pow) {
    // Create the containing system
    ChSystemSMC sys;
    sys.SetGravitationalAcceleration({0, 0, -9.81});
    sys.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
    sys.GetSolver()->AsIterative()->SetMaxIterations(150);
    sys.SetMaxPenetrationRecoverySpeed(4.0);

    // Create the terrain
    CRGTerrain terrain(&sys);
    terrain.UseMeshVisualization(useMesh);
    terrain.SetContactFrictionCoefficient(0.8f);
    terrain.SetRoadsidePostDistance(50.0);
    terrain.SetRoadDiffuseTextureFile("vehicle/terrain/textures/Concrete002_2K-JPG/Concrete002_2K_Color.jpg");
    terrain.SetRoadNormalTextureFile("vehicle/terrain/textures/Concrete002_2K-JPG/Concrete002_2K_NormalGL.jpg");
    terrain.SetRoadRoughnessTextureFile("vehicle/terrain/textures/Concrete002_2K-JPG/Concrete002_2K_Roughness.jpg");
    terrain.Initialize(crg_road_file);

    // Get the vehicle path (middle of the road)
    auto path = terrain.GetRoadCenterLine();

    // Initial location and orientation from CRG terrain (create vehicle 0.5 m above road)
    auto init_csys = terrain.GetStartPosition();
    init_csys.pos = init_csys.TransformPointLocalToParent(ChVector3d(4, 0, 0.5));

    cout << "\n----------------------------------------------\n" << endl;
    cout << "Target speed = " << speed << " mph" << endl;
    cout << "Road length  = " << terrain.GetLength() << " m" << endl;
    cout << "Road width   = " << terrain.GetWidth() << " m" << endl;

    // Create the vehicle
    FEDA my_feda(&sys);
    my_feda.SetContactMethod(ChContactMethod::SMC);
    my_feda.SetTireCollisionType(ChTire::CollisionType::ENVELOPE);
    my_feda.SetTirePressure(tire_pressure_psi * psi_to_pascal);
    my_feda.SetTireType(tire_model);
    my_feda.SetTireStepSize(tire_step_size);
    my_feda.SetChassisFixed(false);
    my_feda.SetInitPosition(init_csys);
    my_feda.SetEngineType(EngineModelType::SIMPLE_MAP);
    my_feda.SetTransmissionType(TransmissionModelType::AUTOMATIC_SIMPLE_MAP);
    my_feda.SetDamperMode(FEDA::DamperMode::PASSIVE_LOW);  // use semiactive dampers
    my_feda.SetRideHeight_ObstacleCrossing();              // high static height
    my_feda.Initialize();

    my_feda.SetChassisVisualizationType(VisualizationType::PRIMITIVES);
    my_feda.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    my_feda.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    my_feda.SetWheelVisualizationType(VisualizationType::NONE);
    my_feda.SetTireVisualizationType(VisualizationType::MESH);

    // Create driver system
    MyDriver driver(driver_type, my_feda.GetVehicle(), mph_to_mps * speed, path);
    driver.Initialize();

    cout << "Driver model: " << driver.GetDriverType() << endl << endl;

    // Create the visualization system
#ifndef CHRONO_IRRLICHT
    if (vis_type == ChVisualSystem::Type::IRRLICHT)
        vis_type = ChVisualSystem::Type::VSG;
#endif
#ifndef CHRONO_VSG
    if (vis_type == ChVisualSystem::Type::VSG)
        vis_type = ChVisualSystem::Type::IRRLICHT;
#endif

    char cval[10];
    char sval[10];
    snprintf(cval, 9, "%.1f", rms);
    snprintf(sval, 9, "%.1f", speed);
    std::string winTitle =
        "FED Alpha Ride Test (RMS = " + std::string(cval) + " in | Speed = " + std::string(sval) + " mph)";

    std::shared_ptr<ChVehicleVisualSystem> vis;
    int sentinelID = 0;
    int targetID = 0;
    switch (vis_type) {
        case ChVisualSystem::Type::NONE:
            break;
        case ChVisualSystem::Type::IRRLICHT: {
#ifdef CHRONO_IRRLICHT
            // Create the vehicle Irrlicht interface
            auto vis_irr = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
            vis_irr->SetWindowTitle(winTitle);
            vis_irr->SetChaseCamera(ChVector3d(0.0, 0.0, 1.75), 12.0, 0.5);
            vis_irr->Initialize();
            vis_irr->AddLightDirectional();
            vis_irr->AddSkyBox();
            vis_irr->AddLogo();
            vis_irr->AttachVehicle(&my_feda.GetVehicle());

            vis = vis_irr;
#endif
            break;
        }
        default:
        case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
            // Create the vehicle VSG interface
            auto vis_vsg = chrono_types::make_shared<ChWheeledVehicleVisualSystemVSG>();
            vis_vsg->SetWindowTitle(winTitle);
            vis_vsg->AttachVehicle(&my_feda.GetVehicle());
            vis_vsg->SetChaseCamera(ChVector3d(0.0, 0.0, 1.75), 12.0, 0.5);
            vis_vsg->SetWindowSize(1200, 800);
            vis_vsg->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
            vis_vsg->SetShadows(true);

            vis = vis_vsg;
#endif
            break;
        }
    }

    if (vis) {
        auto sentinel = chrono_types::make_shared<ChVisualShapeSphere>(0.1);
        auto target = chrono_types::make_shared<ChVisualShapeSphere>(0.1);
        sentinel->SetColor(ChColor(1, 0, 0));
        target->SetColor(ChColor(0, 1, 0));
        sentinelID = vis->AddVisualModel(sentinel, ChFrame<>());
        targetID = vis->AddVisualModel(target, ChFrame<>());

        vis->Initialize();
    }

    // ---------------
    // Simulation loop
    // ---------------

    // Number of simulation steps between image outputs
    double render_step_size = 1 / fps;
    int render_steps = (int)std::ceil(render_step_size / step_size);

    // Initialize frame counters
    int sim_frame = 0;

    double max_travel = 404.0;

    chrono::utils::ChISO2631_Vibration_SeatCushionLogger cushion(step_size);

    while (true) {
        double time = my_feda.GetSystem()->GetChTime();

        ChVector3d xpos = my_feda.GetVehicle().GetPos();
        ChVector3d sacc = my_feda.GetVehicle().GetPointAcceleration(ChVector3d(-1.0, 1.0, 0.5));
        double vel = my_feda.GetVehicle().GetSpeed();
        if (xpos.x() > 100.0) {
            cushion.AddData(vel, sacc);
        }

        std::cout << std::fixed << std::setw(5) << std::setprecision(1) << xpos.x();
        std::cout << "\r" << std::flush;

        // Driver inputs
        DriverInputs driver_inputs = driver.GetInputs();

        // Render scene and output images
        if (vis && sim_frame % render_steps == 0) {
            if (!vis->Run())
                break;

            // Update sentinel and target location markers for the path-follower controller.
            vis->UpdateVisualModel(sentinelID, ChFrame<>(driver.GetSentinelLocation()));
            vis->UpdateVisualModel(targetID, ChFrame<>(driver.GetTargetLocation()));

            vis->BeginScene();
            vis->Render();
            vis->EndScene();
        }

        // Update modules (process inputs from other modules)
        driver.Synchronize(time);
        terrain.Synchronize(time);
        my_feda.Synchronize(time, driver_inputs, terrain);
        if (vis)
            vis->Synchronize(time, driver_inputs);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        terrain.Advance(step_size);
        my_feda.Advance(step_size);
        if (vis)
            vis->Advance(step_size);
        sys.DoStepDynamics(step_size);

        // Increment simulation frame number
        sim_frame++;

        // Test end of track
        if (xpos.x() > max_travel) {
            cout << "Front wheels at x = " << xpos.x() << " m" << endl;
            cout << "End of rms surface reached. Regular simulation end." << endl;
            break;
        }
    }

    avg_vel = mps_to_mph * cushion.GetAVGSpeed();
    abs_pow = cushion.GetAbsorbedPowerVertical();

    cout << "Average Velocity = " << avg_vel << " miles/h" << endl;
    cout << "Absorbed Power = " << abs_pow << " W" << endl;
}

// =============================================================================

int main(int argc, char* argv[]) {
    cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << endl;

    ChCLI cli(argv[0]);

    // Set up parameter defaults and command-line arguments
    DriverModelType driver_type = DriverModelType::PID;
    std::string crg_road_file = "terrain/crg_roads/detrended_rms_course_1in.crg";

    cli.AddOption<std::string>("Demo", "f,roadfile", "CRG road filename", crg_road_file);
    cli.AddOption<int>("Demo", "c,controller", "Controller type (1:PID, 2:STANLEY, 3:SR", "1");
    cli.AddOption<double>("Demo", "s,speed", "Start speed (mph)", "4.0");
    cli.AddOption<double>("Demo", "i,increment", "Speed increment (mph)", "2.0");
    cli.AddOption<double>("Demo", "e,endspeed", "Max. Speed (mph)", "20.0");

    if (!cli.Parse(argc, argv, true)) {
        cli.Help();
        return 1;
    }

    switch (cli.GetAsType<int>("controller")) {
        case 1:
            driver_type = DriverModelType::PID;
            break;
        case 2:
            driver_type = DriverModelType::STANLEY;
            break;
        case 3:
            driver_type = DriverModelType::SR;
            break;
        default:
            cli.Help();
            return 1;
    }
    crg_road_file = vehicle::GetDataFile(cli.GetAsType<std::string>("roadfile"));

    double speed_start = cli.GetAsType<double>("speed");
    double speed_inc = cli.GetAsType<double>("increment");
    double speed_end = cli.GetAsType<double>("endspeed");

    std::string ride_data_file;
    ride_data_file.assign(crg_road_file);
    ride_data_file.erase(ride_data_file.end() - 4, ride_data_file.end());
    size_t pos_us = ride_data_file.find_last_of("_");  // find last underscore
    std::string rmsvstr = ride_data_file.substr(pos_us + 1);
    rmsvstr.erase(rmsvstr.end() - 2, rmsvstr.end());  // remove unit string "in"
    size_t pos_p = rmsvstr.find("p");                 // place holder for decimal dot?
    if (pos_p != std::string::npos)
        rmsvstr[pos_p] = '.';
    double rmsValue = std::stod(rmsvstr);

    // ----------------
    // Output directory
    // ----------------

    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        cout << "Error creating directory " << out_dir << endl;
        return 1;
    }

    ChFunctionInterp powRec;

    std::string datafile = out_dir + "/test_gnuplot_data.dat";
    std::ofstream mdatafile(datafile);

    bool power_limit_reached = false;

    for (double speed = speed_start; speed <= speed_end; speed += speed_inc) {
        double avg_vel;
        double abs_pow;
        RunTest(speed, driver_type, crg_road_file, rmsValue, avg_vel, abs_pow);

        powRec.AddPoint(abs_pow, avg_vel);
        mdatafile << avg_vel << "\t" << abs_pow << "\t6.0" << endl;
        if (abs_pow >= 6.0) {
            power_limit_reached = true;
            cout << "\n\nPower limit reached at target speed = " << speed << " mph" << endl;
            break;
        }
    }

    cout << "\n----------------------------------------------\n" << endl;

    chrono::postprocess::ChGnuPlot mplot(out_dir + "/tmp_gnuplot_3.gpl");
    mplot.SetTitle("FED Alpha Ride Test");
    mplot.SetLabelX("Vehicle Speed (mph)");
    mplot.SetLabelY("Absorbed Power (W)");
    std::string tstr = "RMS ";
    tstr.append(rmsvstr);
    tstr.append(" in");
    mplot.Plot(datafile, 1, 2, tstr, " with linespoints");
    mplot.Plot(datafile, 1, 3, "Absorbed Power Limit", " with lines");

    if (!power_limit_reached) {
        cout << "Could not find Absorbed Power limit!\n";
        cout << "Speed 6W above " << powRec.GetTable().rbegin()->first << " mph\n";
    } else {
        double speed_6W = powRec.GetVal(6.0);
        cout << "Speed 6W = " << speed_6W << " mph\n";
    }
    return 0;
}
