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
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/ChWorldFrame.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/driver/ChHumanDriver.h"
#include "chrono_vehicle/terrain/CRGTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"

#include "chrono_models/vehicle/hmmwv/HMMWV.h"

#include "chrono_thirdparty/cxxopts/ChCLI.h"
#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;

// =============================================================================
// Problem parameters

enum class DriverModelType {
    PID,      // pure PID lateral controller with constant speed controller
    STANLEY,  // geometrical P heading and PID lateral controller with constant speed controller
    XT,       // alternative PID lateral controller with constant speed controller
    SR,       // alternative PID lateral controller with constant speed controller
    HUMAN     // simple realistic human driver
};

// Type of tire model (FIALA, PAC89, PAC02, or TMEASY)
TireModelType tire_model = TireModelType::TMEASY;

// Road visualization (mesh or boundary lines)
bool useMesh = false;

// Desired vehicle speed (m/s)
double target_speed = 12;

// Minimum / maximum speed (m/s) for Human driver type
double minimum_speed = 12;
double maximum_speed = 30;

// Simulation step size
double step_size = 3e-3;
double tire_step_size = 1e-3;

// Output frame images
bool output_images = false;
double fps = 60;
const std::string out_dir = GetChronoOutputPath() + "OPENCRG_DEMO";

DriverModelType DriverModelFromString(const std::string& str) {
    if (str == "HUMAN")
        return DriverModelType::HUMAN;
    if (str == "PID")
        return DriverModelType::PID;
    if (str == "STANLEY")
        return DriverModelType::STANLEY;
    if (str == "SR")
        return DriverModelType::SR;
    if (str == "XT")
        return DriverModelType::XT;
    std::cerr << "String \"" + str +
                     "\" does not represent a valid DriverModelType (HUMAN/PID/SR/XT) - returned DriverModelType::HUMAN"
              << std::endl;
    return DriverModelType::HUMAN;
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
            case DriverModelType::XT: {
                m_driver_type = "XT";

                auto driverXT = chrono_types::make_shared<ChPathFollowerDriverXT>(
                    vehicle, path, "my_path", target_speed, vehicle.GetMaxSteeringAngle());
                driverXT->GetSteeringController().SetLookAheadDistance(5);
                driverXT->GetSteeringController().SetGains(0.4, 1, 1, 1);
                driverXT->GetSpeedController().SetGains(0.4, 0, 0);

                m_driver = driverXT;
                m_steering_controller = &driverXT->GetSteeringController();
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
            case DriverModelType::HUMAN: {
                m_driver_type = "HUMAN";

                // Driver model read from JSON file
                ////auto driverHUMAN = chrono_types::make_shared<ChHumanDriver>(
                ////    vehicle::GetDataFile("hmmwv/driver/HumanController.json"), vehicle, path, "my_path",
                ////    road_width, vehicle.GetMaxSteeringAngle(), 3.2);

                auto driverHUMAN = chrono_types::make_shared<ChHumanDriver>(vehicle, path, "my_path", road_width,
                                                                            vehicle.GetMaxSteeringAngle(), 3.2);
                driverHUMAN->SetPreviewTime(0.5);
                driverHUMAN->SetLateralGains(0.1, 2);
                driverHUMAN->SetLongitudinalGains(0.1, 0.1, 0.2);
                driverHUMAN->SetSpeedRange(minimum_speed, maximum_speed);

                m_driver = driverHUMAN;
                break;
            }
        }
    }

    DriverInputs GetInputs() { return m_driver->GetInputs(); }
    void Initialize() { m_driver->Initialize(); }
    void Synchronize(double time) { m_driver->Synchronize(time); }
    void Advance(double step) { m_driver->Advance(step); }

    const std::string& GetDriverType() { return m_driver_type; }

    ChVector<> GetTargetLocation() {
        if (m_type == DriverModelType::HUMAN)
            return std::static_pointer_cast<ChHumanDriver>(m_driver)->GetTargetLocation();
        else
            return m_steering_controller->GetTargetLocation();
    }

    ChVector<> GetSentinelLocation() {
        if (m_type == DriverModelType::HUMAN)
            return std::static_pointer_cast<ChHumanDriver>(m_driver)->GetSentinelLocation();
        else
            return m_steering_controller->GetSentinelLocation();
    }

    void PrintStats() {
        if (m_type != DriverModelType::HUMAN)
            return;

        auto driverHUMAN = std::static_pointer_cast<ChHumanDriver>(m_driver);
        std::cout << std::endl;
        std::cout << "Traveled Distance    = " << driverHUMAN->GetTraveledDistance() << " m" << std::endl;
        std::cout << "Average Speed        = " << driverHUMAN->GetAverageSpeed() << " m/s" << std::endl;
        std::cout << "Maximum Speed        = " << driverHUMAN->GetMaxSpeed() << " m/s" << std::endl;
        std::cout << "Minimum Speed        = " << driverHUMAN->GetMinSpeed() << " m/s" << std::endl;
        std::cout << "Maximum Lateral Acc. = " << driverHUMAN->GetMaxLatAcc() << " m^2/s" << std::endl;
        std::cout << "Minimum Lateral Acc. = " << driverHUMAN->GetMinLatAcc() << " m^2/s" << std::endl;
    }

  private:
    DriverModelType m_type;
    std::string m_driver_type;
    std::shared_ptr<ChDriver> m_driver;
    ChSteeringController* m_steering_controller;
};

// =============================================================================

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    ChCLI cli(argv[0]);

    // Set up parameter defaults and command-line arguments
    DriverModelType driver_type = DriverModelType::HUMAN;
    std::string crg_road_file = "terrain/crg_roads/RoadCourse.crg";
    bool yup = false;

    cli.AddOption<std::string>("Demo", "m,model", "Controller model type - PID, STANLEY, XT, SR, HUMAN", "HUMAN");
    cli.AddOption<std::string>("Demo", "f,roadfile", "CRG road filename", crg_road_file);
    cli.AddOption<bool>("Demo", "y,yup", "Use YUP world frame", std::to_string(yup));

    if (!cli.Parse(argc, argv, true))
        return 1;

    driver_type = DriverModelFromString(cli.GetAsType<std::string>("model"));
    crg_road_file = vehicle::GetDataFile(cli.GetAsType<std::string>("roadfile"));
    yup = cli.GetAsType<bool>("yup");

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
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    sys.Set_G_acc(-9.81 * ChWorldFrame::Vertical());
    sys.SetSolverMaxIterations(150);
    sys.SetMaxPenetrationRecoverySpeed(4.0);

    // ------------------
    // Create the terrain
    // ------------------

    // For a crg terrain with arbitrary start heading the terrain class must be initialized before the vehicle class

    std::cout << std::endl;
    std::cout << "CRG road file: " << crg_road_file << std::endl;

    CRGTerrain terrain(&sys);
    terrain.UseMeshVisualization(useMesh);
    terrain.SetContactFrictionCoefficient(0.8f);
    terrain.SetRoadsidePostDistance(100.0);
    terrain.SetRoadDiffuseTextureFile("textures/concrete.jpg");
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

    terrain.GetGround()->AddVisualShape(
        chrono_types::make_shared<ChVisualShapeBox>(geometry::ChBox(1, road_width, 0.1)),
        ChFrame<>(init_csys.pos - 0.05 * ChWorldFrame::Vertical(), init_csys.rot));

    path->write(out_dir + "/path.txt");

    // ------------------
    // Create the vehicle
    // ------------------

    // Initial location and orientation from CRG terrain (create vehicle 0.5 m above road)
    init_csys.pos += 0.5 * ChWorldFrame::Vertical();

    // Create the HMMWV vehicle, set parameters, and initialize
    HMMWV_Full hmmwv(&sys);
    hmmwv.SetContactMethod(ChContactMethod::SMC);
    hmmwv.SetChassisFixed(false);
    hmmwv.SetInitPosition(init_csys);
    hmmwv.SetEngineType(EngineModelType::SHAFTS);
    hmmwv.SetTransmissionType(TransmissionModelType::AUTOMATIC_SHAFTS);
    hmmwv.SetDriveType(DrivelineTypeWV::RWD);
    hmmwv.SetTireType(tire_model);
    hmmwv.SetTireStepSize(tire_step_size);
    hmmwv.Initialize();

    hmmwv.SetChassisVisualizationType(VisualizationType::PRIMITIVES);
    hmmwv.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    hmmwv.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    hmmwv.SetWheelVisualizationType(VisualizationType::NONE);
    hmmwv.SetTireVisualizationType(VisualizationType::PRIMITIVES);

    // --------------------
    // Create driver system
    // --------------------

    MyDriver driver(driver_type, hmmwv.GetVehicle(), path, road_width);
    driver.Initialize();

    std::cout << "Driver model: " << driver.GetDriverType() << std::endl << std::endl;

    // -------------------------------
    // Create the visualization system
    // -------------------------------

    auto vis = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
    vis->SetHUDLocation(500, 20);
    vis->SetWindowTitle("OpenCRG Steering");
    vis->SetChaseCamera(ChVector<>(0.0, 0.0, 1.75), 6.0, 0.5);
    vis->Initialize();
    vis->AddSkyBox();
    vis->AddLogo();
    vis->AddLightDirectional();
    vis->AttachVehicle(&hmmwv.GetVehicle());

    // Visualization of controller points (sentinel & target)
    auto sentinel = chrono_types::make_shared<ChVisualShapeSphere>(0.1);
    auto target = chrono_types::make_shared<ChVisualShapeSphere>(0.1);
    sentinel->SetColor(ChColor(1, 0, 0));
    target->SetColor(ChColor(0, 1, 0));
    int sentinelID = vis->AddVisualModel(sentinel, ChFrame<>());
    int targetID = vis->AddVisualModel(target, ChFrame<>());

    // ---------------
    // Simulation loop
    // ---------------

    // Number of simulation steps between image outputs
    double render_step_size = 1 / fps;
    int render_steps = (int)std::ceil(render_step_size / step_size);

    // Initialize frame counters
    int sim_frame = 0;
    int render_frame = 0;

    while (vis->Run()) {
        double time = hmmwv.GetSystem()->GetChTime();

        // Driver inputs
        DriverInputs driver_inputs = driver.GetInputs();

        // Update sentinel and target location markers for the path-follower controller.
        vis->UpdateVisualModel(sentinelID, ChFrame<>(driver.GetSentinelLocation()));
        vis->UpdateVisualModel(targetID, ChFrame<>(driver.GetTargetLocation()));

        // Render scene and output images
        vis->BeginScene();
        vis->Render();

        // Draw the world reference frame at the sentinel location
        vis->RenderFrame(ChFrame<>(driver.GetSentinelLocation()));

        if (output_images && sim_frame % render_steps == 0) {
            // Zero-pad frame numbers in file names for postprocessing
            std::ostringstream filename;
            filename << out_dir << "/image_" << std::setw(5) << std::setfill('0') << render_frame << ".bmp";
            vis->WriteImageToFile(filename.str());
            render_frame++;
        }

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
        sys.DoStepDynamics(step_size);

        // Increment simulation frame number
        sim_frame++;

        vis->EndScene();
    }

    driver.PrintStats();

    return 0;
}
