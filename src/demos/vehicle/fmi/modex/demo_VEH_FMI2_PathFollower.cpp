// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2025 projectchrono.org
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
// Demo illustrating the use of a model exchange FMU in a Chrono::Vehicle
// simulation. A full HMMWV vehicle is coupled to an FMU that implements a
// model for a lateral path-follower and a longitudinal cruise controller.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include <cmath>

#include "chrono/ChConfig.h"
#include "chrono/geometry/ChLineBezier.h"
#include "chrono/input_output/ChWriterCSV.h"
#include "chrono/utils/ChFilters.h"
#include "chrono/utils/ChUtils.h"

#include "chrono_vehicle/ChConfigVehicleFMI.h"
#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/terrain/FlatTerrain.h"

#include "chrono_models/vehicle/hmmwv/HMMWV.h"

#include "chrono_fmi/ChExternalFmu.h"

#ifdef CHRONO_IRRLICHT
    #include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"
using namespace chrono::irrlicht;
#endif

#ifdef CHRONO_VSG
    #include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemVSG.h"
using namespace chrono::vsg3d;
#endif

#ifdef CHRONO_POSTPROCESS
    #include "chrono_postprocess/ChGnuPlot.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::fmi2;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;

using std::cout;
using std::endl;

// =============================================================================

// Run-time visualization system (IRRLICHT or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// Simulation step sizes
double step_size = 2e-3;
double tire_step_size = 2e-3;

// Simulation end time (used only if render = false)
double t_end = 15;

// Output
bool output = true;

// Visualization settings
bool render = true;
double render_fps = 120;
bool save_img = false;

// =============================================================================

class ChExternalDriverFmu : public ChDriver, public ChExternalFmu {
  public:
    ChExternalDriverFmu(ChVehicle& vehicle, const std::string& fmu_filename, const std::string& path_filename)
        : ChDriver(vehicle), m_fmu_filename(fmu_filename), m_path_filename(path_filename) {}

    void SetUnpackDirectory(const std::string& unpack_dir) { m_unpack_dir = unpack_dir; }

    ChVector3d GetSentinelLocation() const { return GetVecVariable("sentinel_loc"); }
    ChVector3d GetTargetLocation() const { return GetVecVariable("target_loc"); }

    virtual void Initialize() override {
        // Load the FMU
        try {
            Load("driver_fmu", m_fmu_filename, m_unpack_dir);
        } catch (std::exception& e) {
            std::cerr << "ERROR loading FMU: " << e.what() << std::endl;
            throw e;
        }

        if (m_verbose)
            PrintInfo();

        //// TODO: provide setter functions to set parameter values from outside

        // Set FMU parameters
        double throttle_threshold = 0.2;  // [-]
        double look_ahead_dist = 5.0;     // [m]
        double target_speed = 12;         // [m/s]

        SetStringParameterValue("path_file", m_path_filename);
        SetRealParameterValue("look_ahead_dist", look_ahead_dist);
        SetRealParameterValue("throttle_threshold", throttle_threshold);
        SetRealParameterValue("target_speed", target_speed);

        // Set FMU initial conditions
        SetInitialCondition("err_lat", 0.0);
        SetInitialCondition("err_long", 0.0);

        // Initialize the base class
        ChExternalFmu::Initialize();
    }

    virtual void Synchronize(double time) override {
        SetFrameMovingVariable("ref_frame", m_vehicle.GetRefFrame());

        m_steering = GetRealVariable("steering");
        m_throttle = GetRealVariable("throttle");
        m_braking = GetRealVariable("braking");
        m_clutch = 0.0;
    }

    virtual void Advance(double step) override final {}

    void PrintInfo() {
        // Print all FMU variables
        PrintFmuVariables();

        // Print names of all FMU states
        auto s_list = GetStatesList();
        std::cout << "\nFMU states:  ";
        for (const auto& v : s_list)
            std::cout << v << "  ";
        std::cout << std::endl;

        // Print names of all real FMU parameters
        auto rp_list = GetRealParametersList();
        std::cout << "FMU real parameters:  ";
        for (const auto& v : rp_list)
            std::cout << v << "  ";
        std::cout << std::endl;

        // Print names of all real FMU parameters
        auto ip_list = GetIntParametersList();
        std::cout << "FMU integer parameters:  ";
        for (const auto& v : ip_list)
            std::cout << v << "  ";
        std::cout << std::endl;

        // Print names of all real FMU inputs
        auto ri_list = GetRealInputsList();
        std::cout << "FMU real inputs:  ";
        for (const auto& v : ri_list)
            std::cout << v << "  ";
        std::cout << "\n" << std::endl;
    }

  private:
    std::string m_fmu_filename;   ///< name of the FMU file
    std::string m_unpack_dir;     ///< name of directory where FMU file is unpacked
    std::string m_path_filename;  ///< name of Bezier path file
};

// =============================================================================

int main(int argc, char* argv[]) {
    cout << "Copyright (c) 2025 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n" << endl;

    // -----------------------
    // Model exchange FMU
    // -----------------------

    // Specify FMU and unpack directory
#ifdef FMU_EXPORT_SUPPORT
    std::string driver_fmu_model_identfier = "FMU2me_PathFollowerDriver";
    std::string driver_fmu_dir = CHRONO_VEHICLE_FMU_DIR + driver_fmu_model_identfier + std::string("/");
    std::string driver_fmu_filename = driver_fmu_dir + driver_fmu_model_identfier + std::string(".fmu");
#else
    if (argc != 2) {
        std::cout << "Usage: ./demo_VEH_FMI2_PathFollower [driver_FMU_filename]" << std::endl;
        return 1;
    }
    std::string driver_fmu_filename = argv[1];  // FMU (fully qualified filename) specified as program argument
#endif

    // FMU unpack directory
    std::string driver_unpack_dir = CHRONO_VEHICLE_FMU_DIR + std::string("tmp_unpack_driver/");

    // ---------------
    // Create the path
    // ---------------

    std::string path_filename = GetVehicleDataFile("paths/ISO_double_lane_change.txt");
    auto path = ChBezierCurve::Read(path_filename, false);

    // Find initial position on path
    auto num_path_points = static_cast<unsigned int>(path->GetNumPoints());
    const auto& point_0 = path->GetPoint(0);
    const auto& point_1 = path->GetPoint(1);
    const auto& point_n = path->GetPoint(num_path_points - 1);
    auto init_loc = point_0;
    auto last_loc = point_n;
    auto init_yaw = std::atan2(point_1.y() - point_0.y(), point_1.x() - point_0.x());

    std::cout << "Initial path location: " << init_loc << std::endl;
    std::cout << "Initial path yaw:      " << init_yaw << std::endl;

    // --------------
    // Create vehicle
    // --------------

    // Create the HMMWV vehicle, set parameters, and initialize.
    HMMWV_Full hmmwv;
    hmmwv.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    hmmwv.SetContactMethod(ChContactMethod::SMC);
    hmmwv.SetChassisFixed(false);
    hmmwv.SetInitPosition(ChCoordsys<>(init_loc + ChVector3d(0, 0, 0.4), QuatFromAngleZ(init_yaw)));
    hmmwv.SetEngineType(EngineModelType::SHAFTS);
    hmmwv.SetTransmissionType(TransmissionModelType::AUTOMATIC_SHAFTS);
    hmmwv.SetDriveType(DrivelineTypeWV::AWD);
    hmmwv.SetTireType(TireModelType::TMEASY);
    hmmwv.SetTireStepSize(tire_step_size);
    hmmwv.SetAerodynamicDrag(0.5, 5.0, 1.2);
    hmmwv.Initialize();

    hmmwv.SetChassisVisualizationType(VisualizationType::NONE);
    hmmwv.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    hmmwv.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    hmmwv.SetWheelVisualizationType(VisualizationType::NONE);
    hmmwv.SetTireVisualizationType(VisualizationType::MESH);

    auto& vehicle = hmmwv.GetVehicle();
    auto sys = hmmwv.GetSystem();

    // Vehicle data
    double wheel_base = vehicle.GetWheelbase();
    double steering_gear_ratio = 3.5 * 360.0 / 2;  // caution: estimated value 3.5 revolutions from left to right

    // -------------
    // Create driver
    // -------------

    // Create the driver Chrono FMU wrapper
    auto fmu_driver = chrono_types::make_shared<ChExternalDriverFmu>(vehicle, driver_fmu_filename, path_filename);
    fmu_driver->SetVerbose(true);
    fmu_driver->SetUnpackDirectory(driver_unpack_dir);

    // Initialize FMU wrapper
    fmu_driver->Initialize();

    // Attach driver FMU wrapper as a Chrono modeling component
    sys->Add(fmu_driver);

    // ------------------
    // Create the terrain
    // ------------------

    FlatTerrain terrain(-0.05, 0.8f);

    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetFixed(true);
    sys->AddBody(ground);

    // Add path visualization
    auto path_asset = chrono_types::make_shared<ChVisualShapeLine>();
    path_asset->SetLineGeometry(chrono_types::make_shared<ChLineBezier>(path));
    path_asset->SetColor(ChColor(0.8f, 0.8f, 0.0f));
    path_asset->SetNumRenderPoints(std::max<unsigned int>(2 * num_path_points, 400));
    ground->AddVisualShape(path_asset);

    // ----------------------------------------
    // Create the run-time visualization system
    // ----------------------------------------

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
    auto vis = chrono_types::make_shared<ChVehicleVisualSystem>();
    if (render) {
        switch (vis_type) {
            case ChVisualSystem::Type::IRRLICHT: {
#ifdef CHRONO_IRRLICHT
                auto vis_irr = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
                vis_irr->SetWindowTitle("Double lane change with driver FMU");
                vis_irr->SetBackgroundColor(ChColor(0.37f, 0.50f, 0.60f));
                vis_irr->SetChaseCamera(ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5);
                vis_irr->Initialize();
                vis_irr->AddLightDirectional();
                vis_irr->AddLogo();
                vis_irr->AttachVehicle(&vehicle);

                vis = vis_irr;
#endif
                break;
            }
            default:
            case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
                auto vis_vsg = chrono_types::make_shared<ChWheeledVehicleVisualSystemVSG>();
                vis_vsg->SetWindowTitle("Double lane change with driver FMU");
                vis_vsg->SetWindowSize(1280, 800);
                vis_vsg->SetWindowPosition(100, 100);
                vis_vsg->SetBackgroundColor(ChColor(0.37f, 0.50f, 0.60f));
                vis_vsg->SetCameraAngleDeg(40);
                vis_vsg->SetLightIntensity(1.0f);
                vis_vsg->SetChaseCamera(ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5);
                vis_vsg->AttachVehicle(&vehicle);
                vis_vsg->AttachTerrain(&terrain);
                vis_vsg->Initialize();

                vis = vis_vsg;
#endif
                break;
            }
        }

        // Add a visualization grid
        vis->AddGrid(0.5, 0.5, 2000, 400, ChCoordsys<>(init_loc + ChVector3d(0, 0, -0.05), QuatFromAngleZ(init_yaw)),
                     ChColor(0.31f, 0.43f, 0.43f));

        // Add visualization of controller points (sentinel & target)
        auto ballS = chrono_types::make_shared<ChVisualShapeSphere>(0.1);
        auto ballT = chrono_types::make_shared<ChVisualShapeSphere>(0.1);
        ballS->SetColor(ChColor(1, 0, 0));
        ballT->SetColor(ChColor(0, 1, 0));
        sentinelID = vis->AddVisualModel(ballS, ChFrame<>());
        targetID = vis->AddVisualModel(ballT, ChFrame<>());
    }

    // -------------------------
    // Create output directories
    // -------------------------

    const std::string out_dir = GetChronoOutputPath() + "DEMO_WHEELEDVEHICLE_FMI_MODEX";
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    if (save_img) {
        if (!filesystem::create_directory(filesystem::path(out_dir + "/img"))) {
            std::cout << "Error creating directory " << out_dir + "/img" << std::endl;
            return 1;
        }
    }

    // ---------------
    // Simulation loop
    // ---------------

    // Filtered vehicle outputs
    utils::ChRunningAverage speed_filter(500);
    utils::ChButterworthLowpass accel_filter(4, step_size, 2.0);
    utils::ChButterworthLowpass steer_filter(4, step_size, 2.0);
    utils::ChFilterD ang_diff(step_size);

    // Vehicle output recorders
    ChFunctionInterp speed_recorder;
    ChFunctionInterp accel_recorder;
    ChFunctionInterp steer_recorder;
    ChFunctionInterp angspeed_recorder;

    // Initialize simulation frame counter and simulation time
    double time = 0;
    int sim_frame = 0;
    int render_frame = 0;

    ChVector3d sentinel_loc = VNULL;
    ChVector3d target_loc = VNULL;

    ChTimer timer;
    timer.start();

    while (sentinel_loc.x() < last_loc.x()) {
        if (!render && time > t_end)
            break;

        // Driver inputs
        DriverInputs driver_inputs = fmu_driver->GetInputs();
        sentinel_loc = fmu_driver->GetSentinelLocation();
        target_loc = fmu_driver->GetTargetLocation();

        if (output) {
            double speed = speed_filter.Add(vehicle.GetSpeed());
            double accel = accel_filter.Filter(vehicle.GetPointAcceleration(ChVector3d(-wheel_base / 2, 0, 0)).y());

            speed_recorder.AddPoint(time, speed);
            accel_recorder.AddPoint(time, accel);

            double steer = steering_gear_ratio * steer_filter.Filter(driver_inputs.m_steering);
            steer_recorder.AddPoint(time, steer);
            double angspeed = ang_diff.Filter(steer);
            angspeed_recorder.AddPoint(time, angspeed);
        }

        if (render && time >= render_frame / render_fps) {
            if (!vis->Run())
                break;

            // Update sentinel and target location markers for the path-follower controller
            vis->UpdateVisualModel(sentinelID, ChFrame<>(sentinel_loc));
            vis->UpdateVisualModel(targetID, ChFrame<>(target_loc));

            vis->BeginScene();
            vis->Render();
            vis->EndScene();

            if (save_img) {
                std::ostringstream filename;
                filename << out_dir << "/img/img_" << std::setw(5) << std::setfill('0') << render_frame + 1 << ".bmp";
                vis->WriteImageToFile(filename.str());
            }

            render_frame++;
        }

        // Update modules (process inputs from other modules)
        fmu_driver->Synchronize(time);
        terrain.Synchronize(time);
        hmmwv.Synchronize(time, driver_inputs, terrain);
        vis->Synchronize(time, driver_inputs);

        // Advance simulation for one timestep for all modules
        fmu_driver->Advance(step_size);
        terrain.Advance(step_size);
        hmmwv.Advance(step_size);
        vis->Advance(step_size);

        // Increment frame number
        sim_frame++;
        time += step_size;
    }

    timer.stop();
    std::cout << "Sim time: " << time << std::endl;
    std::cout << "Run time: " << timer() << std::endl;
    std::cout << "RTF:      " << timer() / time << std::endl;

#ifdef CHRONO_POSTPROCESS
    if (output) {
        postprocess::ChGnuPlot gplot_speed(out_dir + "/speed.gpl");
        gplot_speed.SetGrid();
        gplot_speed.SetTitle("Speed");
        gplot_speed.SetLabelX("time (s)");
        gplot_speed.SetLabelY("speed (m/s)");
        gplot_speed.SetAspectRatio(0.25);
        gplot_speed.Plot(speed_recorder, "", " with lines lw 2 lt -1 lc rgb'#5E7F99' ");

        postprocess::ChGnuPlot gplot_acc(out_dir + "/lateral_acceleration.gpl");
        gplot_acc.SetGrid();
        gplot_acc.SetTitle("Lateral Acceleration");
        gplot_acc.SetLabelX("time (s)");
        gplot_acc.SetLabelY("lateral acceleration (m/s^2)");
        gplot_acc.SetAspectRatio(0.25);
        gplot_acc.Plot(accel_recorder, "", " with lines lw 2 lt -1 lc rgb'#5E7F99' ");

        postprocess::ChGnuPlot gplot_steer(out_dir + "/steering_angle.gpl");
        gplot_steer.SetGrid();
        gplot_steer.SetTitle("Steering Wheel Angle");
        gplot_steer.SetLabelX("time (s)");
        gplot_steer.SetLabelY("steering wheel angle (degrees)");
        gplot_steer.SetAspectRatio(0.25);
        gplot_steer.Plot(steer_recorder, "", " with lines lw 2 lt -1 lc rgb'#5E7F99' ");

        postprocess::ChGnuPlot gplot_angspeed(out_dir + "/steering_angular_vel.gpl");
        gplot_angspeed.SetGrid();
        gplot_angspeed.SetTitle("Steering Wheel Angular Speed");
        gplot_angspeed.SetLabelX("time (s)");
        gplot_angspeed.SetLabelY("steering wheel angle (degrees/s)");
        gplot_angspeed.SetAspectRatio(0.25);
        gplot_angspeed.Plot(angspeed_recorder, "", " with lines lw 2 lt -1 lc rgb'#5E7F99' ");
    }
#endif

    return 0;
}
