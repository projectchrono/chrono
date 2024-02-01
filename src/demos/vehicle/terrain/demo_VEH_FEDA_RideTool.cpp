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
// Demonstration of an OpenCRG terrain and different driver controllers.
//
// The default world frame is ISO (Z up, X forward, Y to the left).
// This demo can be set up to work with a non-ISO frame by uncommenting the line
//         ChWorldFrame::SetYUP();
// at the top of the main function.  This will use a world frame with Y up, X
// forward, and Z to the right.
//
// NOTES:
// (1) changing the world frame from the ISO default must be done *before* any
//     other Chrono::Vehicle library calls.
// (2) modifications to user code to use a different world frame are minimal:
//     - set the desired world frame
//     - properly set vehicle initial position (e.g. initial height above terrain)
//     - adjust light locations in the run-time visualization system
//
// =============================================================================

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/assets/ChVisualShapeBox.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/ChWorldFrame.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/driver/ChHumanDriver.h"
#include "chrono_vehicle/terrain/CRGTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemVSG.h"

#include "chrono_models/vehicle/feda/FEDA.h"

#include "chrono_thirdparty/cxxopts/ChCLI.h"
#include "chrono_postprocess/ChGnuPlot.h"
#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::feda;

// =============================================================================
// Problem parameters

enum class DriverModelType {
    PID,      // pure PID lateral controller with constant speed controller
    STANLEY,  // geometrical P heading and PID lateral controller with constant speed controller
    XT,       // alternative PID lateral controller with constant speed controller
    SR,       // alternative PID lateral controller with constant speed controller
    HUMAN     // simple realistic human driver
};

// Type of tire model (RIGID, PAC02, TMSIMPLE, TMEASY)
TireModelType tire_model = TireModelType::PAC02;

// Road visualization (mesh or boundary lines)
bool useMesh = true;

// Desired vehicle speed (m/s)
double target_speed = 12;

// Simulation step size
double step_size = 1e-3;
double tire_step_size = 1e-3;

// Output frame images
bool output_images = false;
double fps = 60;
const std::string out_dir = GetChronoOutputPath() + "RIDETOOL_DEMO";

DriverModelType DriverModelFromString(const std::string& str) {
    if (str == "PID")
        return DriverModelType::PID;
    if (str == "STANLEY")
        return DriverModelType::STANLEY;
    if (str == "SR")
        return DriverModelType::SR;
    std::cerr << "String \"" + str +
                     "\" does not represent a valid DriverModelType (PID/SR/STANLEY) - returned DriverModelType::PID"
              << std::endl;
    return DriverModelType::PID;
}

// =============================================================================

// Wrapper around a driver system of specified type
class MyDriver {
   public:
    MyDriver(DriverModelType type, ChWheeledVehicle& vehicle, std::shared_ptr<ChBezierCurve> path, double road_width)
        : m_type(type), m_steering_controller(nullptr) {
        switch (type) {
            case DriverModelType::PID: {
                m_driver_type = "PID";

                auto driverPID =
                    chrono_types::make_shared<ChPathFollowerDriver>(vehicle, path, "my_path", target_speed);
                driverPID->GetSteeringController().SetLookAheadDistance(5);
                driverPID->GetSteeringController().SetGains(0.5, 0, 0);
                driverPID->GetSpeedController().SetGains(0.4, 0, 0);

                m_driver = driverPID;
                m_steering_controller = &driverPID->GetSteeringController();
                break;
            }
            case DriverModelType::STANLEY: {
                m_driver_type = "STANLEY";

                auto driverStanley =
                    chrono_types::make_shared<ChPathFollowerDriver>(vehicle, path, "my_path", target_speed);
                driverStanley->GetSteeringController().SetLookAheadDistance(5.0);
                driverStanley->GetSteeringController().SetGains(0.5, 0.0, 0.0);
                driverStanley->GetSpeedController().SetGains(0.4, 0, 0);

                m_driver = driverStanley;
                m_steering_controller = &driverStanley->GetSteeringController();
                break;
            }
            case DriverModelType::SR: {
                m_driver_type = "SR";

                auto driverSR = chrono_types::make_shared<ChPathFollowerDriverSR>(
                    vehicle, path, "my_path", target_speed, vehicle.GetMaxSteeringAngle(), 3.2);
                driverSR->GetSteeringController().SetGains(0.1, 5);
                driverSR->GetSteeringController().SetPreviewTime(0.5);
                driverSR->GetSpeedController().SetGains(0.4, 0, 0);

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

    ChVector<> GetTargetLocation() { return m_steering_controller->GetTargetLocation(); }

    ChVector<> GetSentinelLocation() { return m_steering_controller->GetSentinelLocation(); }

   private:
    DriverModelType m_type;
    std::string m_driver_type;
    std::shared_ptr<ChDriver> m_driver;
    ChSteeringController* m_steering_controller;
};

// =============================================================================

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    const double mph_to_mps = 0.44704;
    const double mps_to_mph = 1.0 / mph_to_mps;
    const double psi_to_pascal = 6894.76;

    ChCLI cli(argv[0]);

    // Set up parameter defaults and command-line arguments
    DriverModelType driver_type = DriverModelType::PID;
    std::string crg_road_file = "terrain/crg_roads/detrended_rms_course_1in.crg";
    bool yup = false;

    cli.AddOption<std::string>("Demo", "m,model", "Controller model type - PID, STANLEY, SR", "PID");
    cli.AddOption<std::string>("Demo", "f,roadfile", "CRG road filename", crg_road_file);
    cli.AddOption<std::string>("Demo", "s,speed", "Start speed (mph)", "4.0");
    cli.AddOption<std::string>("Demo", "i,increment", "Speed increment (mph)", "2.0");
    cli.AddOption<std::string>("Demo", "e,endspeed", "Max. Speed (mph)", "20.0");
    cli.AddOption<bool>("Demo", "y,yup", "Use YUP world frame", std::to_string(yup));
    if (!cli.Parse(argc, argv, true))
        return 1;

    driver_type = DriverModelFromString(cli.GetAsType<std::string>("model"));
    crg_road_file = vehicle::GetDataFile(cli.GetAsType<std::string>("roadfile"));
    std::string speed_s = cli.GetAsType<std::string>("speed");
    double speed_start = std::stod(speed_s);
    std::string speed_is = cli.GetAsType<std::string>("increment");
    double speed_inc = std::stod(speed_is);
    std::string speed_es = cli.GetAsType<std::string>("endspeed");
    double speed_end = std::stod(speed_es);
    yup = cli.GetAsType<bool>("yup");

    std::string ride_data_file;
    ride_data_file.assign(crg_road_file);
    ride_data_file.erase(ride_data_file.end()-4,ride_data_file.end());
    size_t pos_us = ride_data_file.find_last_of("_"); // find last underscore
    std::string rmsvstr = ride_data_file.substr(pos_us+1);
    rmsvstr.erase(rmsvstr.end()-2,rmsvstr.end()); // remove unit string "in"
    size_t pos_p = rmsvstr.find("p"); // place holder for decimal dot?
    if(pos_p != std::string::npos)
        rmsvstr[pos_p] = '.';
    double rmsValue = std::stod(rmsvstr);
    GetLog() << "Data file: " << ride_data_file << "\n";
    GetLog() << "Value string: " << rmsvstr << "\n";
    GetLog() << "RMS Value: " << rmsValue << " in\n";
    
    
    // ----------------
    // Output directory
    // ----------------

    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    // ---------------
    // Set World Frame
    // ---------------

    if (yup)
        ChWorldFrame::SetYUP();

    std::cout << "World Frame\n" << ChWorldFrame::Rotation() << std::endl;
    std::cout << "Vertical direction: " << ChWorldFrame::Vertical() << std::endl;
    std::cout << "Forward direction:  " << ChWorldFrame::Forward() << std::endl;

    // ----------------------------
    // Create the containing system
    // ----------------------------

    ChSystemSMC sys;
    sys.Set_G_acc(-9.81 * ChWorldFrame::Vertical());
    sys.SetSolverMaxIterations(150);
    sys.SetMaxPenetrationRecoverySpeed(4.0);

    // ------------------
    // Create the terrain
    // ------------------

    // For a crg terrain with arbitrary start heading the terrain class must be initialized before the vehicle class

    std::cout << std::endl;
    std::cout << "CRG road file: " << crg_road_file << std::endl;

    
    ChFunction_Recorder powRec;
    
    std::string datafile = out_dir + "/test_gnuplot_data.dat";
    ChStreamOutAsciiFile mdatafile(datafile.c_str());

    bool power_limit_reached = 0;
    double speed = speed_start;
    while (speed <= speed_end) {
        CRGTerrain terrain(&sys);
        terrain.UseMeshVisualization(useMesh);
        terrain.SetContactFrictionCoefficient(0.8f);
        terrain.SetRoadsidePostDistance(50.0);
        // bright concrete
        terrain.SetRoadDiffuseTextureFile("vehicle/terrain/textures/Concrete002_2K-JPG/Concrete002_2K_Color.jpg");
        terrain.SetRoadNormalTextureFile("vehicle/terrain/textures/Concrete002_2K-JPG/Concrete002_2K_NormalGL.jpg");
        terrain.SetRoadRoughnessTextureFile("vehicle/terrain/textures/Concrete002_2K-JPG/Concrete002_2K_Roughness.jpg");
        terrain.Initialize(crg_road_file);

        // Get the vehicle path (middle of the road)
        auto path = terrain.GetRoadCenterLine();
        bool path_is_closed = terrain.IsPathClosed();
        double road_length = terrain.GetLength();
        double road_width = terrain.GetWidth();
        auto init_csys = terrain.GetStartPosition();

        std::cout << "Road length = " << road_length << std::endl;
        std::cout << "Road width  = " << road_width << std::endl;
        std::cout << std::boolalpha << "Closed loop?  " << path_is_closed << std::endl << std::endl;

        terrain.GetGround()->AddVisualShape(chrono_types::make_shared<ChVisualShapeBox>(geometry::ChBox(1, road_width, 1)),
                                            ChFrame<>(init_csys.pos - 0.5 * ChWorldFrame::Vertical(), init_csys.rot));

        // Initial location and orientation from CRG terrain (create vehicle 0.5 m above road)
        init_csys.pos += 0.5 * ChWorldFrame::Vertical() + ChVector<>(1, 0, 0);

        path->write(out_dir + "/path.txt");

        target_speed = mph_to_mps * speed;
        // ------------------
        // Create the vehicle
        // ------------------

        double tire_pressure_psi = 35.0;

        // Create the HMMWV vehicle, set parameters, and initialize
        FEDA my_feda(&sys);
        my_feda.SetContactMethod(ChContactMethod::SMC);
        my_feda.SetTireCollisionType(ChTire::CollisionType::ENVELOPE);
        my_feda.SetTirePressure(tire_pressure_psi * psi_to_pascal);  // CDT / KRC Test conditions
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

        // --------------------
        // Create driver system
        // --------------------

        MyDriver driver(driver_type, my_feda.GetVehicle(), path, road_width);
        driver.Initialize();

        std::cout << "Driver model: " << driver.GetDriverType() << std::endl << std::endl;

        double vel = 0;
        // -------------------------------
        // Create the visualization system
        // -------------------------------

        char cval[10];
        char sval[10];
        snprintf(cval,9,"%.1f",rmsValue);
        snprintf(sval,9,"%.1f",speed);
        std::string winTitle = "FED Alpha Ride Test (RMS = ";
        winTitle.append(cval);
        winTitle.append(" in) Speed = ");
        winTitle.append(sval);
        winTitle.append(" miles/hour");
        auto vis = chrono_types::make_shared<ChWheeledVehicleVisualSystemVSG>();
        vis->SetWindowTitle(winTitle);
        vis->SetWindowSize(1200, 800);
        vis->SetChaseCamera(ChVector<>(0.0, 0.0, 1.75), 10.0, 0.5);
        vis->AttachVehicle(&my_feda.GetVehicle());

        auto sentinel = chrono_types::make_shared<ChVisualShapeSphere>(0.1);
        auto target = chrono_types::make_shared<ChVisualShapeSphere>(0.1);
        sentinel->SetColor(ChColor(1, 0, 0));
        target->SetColor(ChColor(0, 1, 0));
        int sentinelID = vis->AddVisualModel(sentinel, ChFrame<>());
        int targetID = vis->AddVisualModel(target, ChFrame<>());

        vis->Initialize();

        // ---------------
        // Simulation loop
        // ---------------

        // Number of simulation steps between image outputs
        double render_step_size = 1 / fps;
        int render_steps = (int)std::ceil(render_step_size / step_size);

        // Initialize frame counters
        int sim_frame = 0;
        int render_frame = 0;

        double max_travel = 404.0;

        chrono::utils::ChISO2631_Vibration_SeatCushionLogger cushion(step_size);

        while (vis->Run()) {
            double time = my_feda.GetSystem()->GetChTime();
            ChVector<> xpos = my_feda.GetVehicle().GetPos();
            ChVector<> sacc = my_feda.GetVehicle().GetPointAcceleration(ChVector<>(-1.0, 1.0, 0.5));
            vel = my_feda.GetVehicle().GetSpeed();
            if (xpos.x() > 100.0) {
                cushion.AddData(vel, sacc);
            }
            // Driver inputs
            DriverInputs driver_inputs = driver.GetInputs();

            // Update sentinel and target location markers for the path-follower controller.
            vis->UpdateVisualModel(sentinelID, ChFrame<>(driver.GetSentinelLocation()));
            vis->UpdateVisualModel(targetID, ChFrame<>(driver.GetTargetLocation()));

            // Render scene and output images
            if (sim_frame % render_steps == 0) {
                vis->Render();

                if (output_images) {
                    char filename[200];
                    int nstr = sizeof(filename) - 1;
                    snprintf(filename, nstr, "%s/image_%05d.bmp", out_dir.c_str(), render_frame);
                    vis->WriteImageToFile(filename);
                    render_frame++;
                }
            }

            // Update modules (process inputs from other modules)
            driver.Synchronize(time);
            terrain.Synchronize(time);
            my_feda.Synchronize(time, driver_inputs, terrain);
            vis->Synchronize(time, driver_inputs);

            // Advance simulation for one timestep for all modules
            driver.Advance(step_size);
            terrain.Advance(step_size);
            my_feda.Advance(step_size);
            vis->Advance(step_size);
            sys.DoStepDynamics(step_size);

            // Increment simulation frame number
            sim_frame++;
            if (xpos.x() > max_travel) {
                GetLog() << "Front wheels at x = " << xpos.x() << " m\n";
                GetLog() << "End of rms surface reached. Regular simulation end.\n";
                vis->Quit();
            }
        }

        double velAvg = mps_to_mph * cushion.GetAVGSpeed();
        double absPow = cushion.GetAbsorbedPowerVertical();
        GetLog() << "Average Velocity = " << velAvg << " miles/h\n";
        GetLog() << "Absorbed Power = " << absPow << " W\n";
        powRec.AddPoint(absPow, velAvg);
        mdatafile << velAvg << "\t" << absPow << "\t6.0" "\n";
        if(absPow >= 6.0) {
            power_limit_reached = true;
            break;
        }
        speed += speed_inc;
    }  // while
    
    chrono::postprocess::ChGnuPlot mplot(out_dir + "/tmp_gnuplot_3.gpl");
    mplot.SetTitle("FED Alpha Ride Test");
    mplot.SetLabelX("Vehicle Speed (miles/hour)");
    mplot.SetLabelY("Absorbed Power (W)");
    std::string tstr = "RMS ";
    tstr.append(rmsvstr);
    tstr.append(" in");
    mplot.Plot(datafile, 1, 2, tstr, " with linespoints");
    mplot.Plot(datafile, 1, 3, "Absorbed Power Limit", " with lines");

    if(!power_limit_reached) {
        GetLog() << "Could not find Absorbed Power limit!\n";
        double a, b;
        powRec.Estimate_x_range(a, b);
        GetLog() << "Speed 6W beyond " << b << " miles/hour\n";
    } else {
        double speed_6W = powRec.Get_y(6.0);
        GetLog() << "Speed 6W = " << speed_6W << " miles/hour\n";
    }
    return 0;
}
