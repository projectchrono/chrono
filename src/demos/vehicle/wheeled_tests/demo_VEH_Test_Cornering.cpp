// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Rainer Gericke
// =============================================================================
//
// Steady-state cornering test for Chrono::Vehicle models.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/assets/ChVisualShapeBox.h"
#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/ChWorldFrame.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/driver/ChHumanDriver.h"
#include "chrono_vehicle/terrain/CRGTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemVSG.h"

#include "chrono_postprocess/ChGnuPlot.h"

#include "chrono_thirdparty/cxxopts/ChCLI.h"
#include "chrono_thirdparty/filesystem/path.h"

#include "../WheeledVehicleModels.h"

using namespace chrono;
using namespace chrono::postprocess;

int main(int argc, char** argv) {
    std::cout << "Copyright (c) 2024 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    ChCLI cli(argv[0]);

    cli.AddOption<double>("SSC", "d,duration", "Subtest duration (s)", "30.0");
    cli.AddOption<double>("SSC", "f,friction-coefficient", "Road surface friction coefficient ()", "0.8");
    cli.AddOption<int>("SSC", "n,nsubtests", "No. of subtests ()", "20");
    cli.AddOption<bool>("SSC", "r,right_turn", "Turn right instead of left", "false");
    cli.AddOption<bool>("SSC", "l,logged_data", "Plot logged data", "false");
    cli.AddOption<bool>("SSC", "c,charts", "Plot result charts", "false");
    cli.AddOption<bool>("Graphics", "show_car_body", "Show Car Body (default NO)", "false");
    cli.AddOption<bool>("SSC", "b,big_radius", "Use 100 m radius instead of 50 m", "false");
    cli.AddOption<double>("SSC", "s,speed_step", "Speed inrement after subtest (m/s)", "1");
    cli.AddOption<double>("Controller", "T,preview_distance", "Preview distance of course controller", "5");
    cli.AddOption<double>("Controller", "m,max_deviation", "Maximal course deviation (m)", "0.1");

    if (!cli.Parse(argc, argv, true)) {
        cli.Help();
        return 1;
    }

    bool show_car_body = cli.GetAsType<bool>("show_car_body");
    bool right_turn = cli.GetAsType<bool>("right_turn");
    bool big_radius = cli.GetAsType<bool>("big_radius");
    bool logged_data_plot = cli.GetAsType<bool>("logged_data");
    bool result_plot = cli.GetAsType<bool>("charts");
    bool output_images = false;
    double road_friction = std::max(0.1,cli.GetAsType<double>("friction-coefficient"));
    double target_speed = 5;
    double speed_step = std::max(1.0, cli.GetAsType<double>("speed_step"));
    double step_size = 1.0e-3;
    double max_dev = std::max(0.1, cli.GetAsType<double>("max_deviation"));  // maximal course deviation
    double fps = 60;
    double t_duration = std::max(15.0, cli.GetAsType<double>("duration"));  // length of subtest
    int n_subtests = std::max(5, cli.GetAsType<int>("nsubtests"));
    size_t switch_frame = floor(t_duration / step_size);  // length of subtest as frame number
    double t_end = double(n_subtests) * t_duration;
    double D_pre = std::max(1.0, cli.GetAsType<double>("preview_distance"));
    std::cout << "Big radius         = " << big_radius << std::endl;
    std::cout << "Turn right         = " << right_turn << std::endl;
    std::cout << "No. Subtests       = " << n_subtests << std::endl;
    std::cout << "Duration           = " << t_duration << std::endl;
    std::cout << "Road friction      = " << road_friction << std::endl;
    std::cout << "Plot logged data   = " << logged_data_plot << std::endl;
    std::cout << "Plot result charts = " << result_plot << std::endl;
    std::cout << "Preview distance   = " << D_pre << std::endl;

    std::string crg_road_file;
    if (big_radius) {
        if (right_turn)
            crg_road_file = GetVehicleDataFile("terrain/crg_roads/circle_100m_right.crg");
        else
            crg_road_file = GetVehicleDataFile("terrain/crg_roads/circle_100m_left.crg");
    } else {
        if (right_turn)
            crg_road_file = GetVehicleDataFile("terrain/crg_roads/circle_50m_right.crg");
        else
            crg_road_file = GetVehicleDataFile("terrain/crg_roads/circle_50m_left.crg");
    }

    // ----------------------------
    // Create the containing system
    // ----------------------------

    ChSystemSMC sys;
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    sys.SetGravitationalAcceleration(-9.81 * ChWorldFrame::Vertical());
    sys.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
    sys.GetSolver()->AsIterative()->SetMaxIterations(150);
    sys.SetMaxPenetrationRecoverySpeed(4.0);

    // ------------------
    // Create the terrain
    // ------------------

    // For a crg terrain with arbitrary start heading the terrain class must be initialized before the vehicle class

    std::cout << std::endl;
    std::cout << "CRG road file: " << crg_road_file << std::endl;

    CRGTerrain terrain(&sys);
    terrain.UseMeshVisualization(true);
    terrain.SetContactFrictionCoefficient(road_friction);
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

    // ------------------
    // Create the vehicle
    // ------------------

    // Select vehicle model (see VehicleModel.h)
    auto models = WheeledVehicleModel::List();

    int num_models = (int)models.size();
    int which = 0;
    std::cout << "Options:\n";
    for (int i = 0; i < num_models; i++)
        std::cout << std::setw(2) << i + 1 << "  " << models[i].second << std::endl;
    std::cout << "\nSelect vehicle: ";
    std::cin >> which;
    std::cout << std::endl;
    ChClampValue(which, 1, num_models);

    auto vehicle_model = models[which - 1].first;

    // Initial location and orientation from CRG terrain (create vehicle 0.5 m above road)
    init_csys.pos += 0.5 * ChWorldFrame::Vertical();

    // Create the vehicle model
    vehicle_model->Create(&sys, init_csys, show_car_body);
    auto& vehicle = vehicle_model->GetVehicle();

    ChPathFollowerDriverPP driver(vehicle, path, "my_path", target_speed);
    driver.GetSteeringController().SetLookAheadDistance(D_pre);
    driver.GetSteeringController().SetGain(0);
    driver.GetSteeringController().SetStartSpeed(target_speed);
    driver.GetSpeedController().SetGains(0.4, 0, 0);
    driver.Initialize();

    // ----------------
    // Output directory
    // ----------------

    std::string out_dir = GetChronoOutputPath() + "STEADY_STATE_CORNERING";
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    out_dir = out_dir + "/" + vehicle_model->ModelName();
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    ChWriterCSV csv("\t");
    csv << "#time speed acc_y deviation" << std::endl;

    std::string datafilename = out_dir + "/ssc_accy_data_";

    ChWriterCSV csv_res("\t");
    csv_res << "#acc_y steer" << std::endl;

    std::string resfilename = out_dir + "/ssc_result_data_";

    ChWriterCSV csv_angle("\t");
    csv_angle << "#acc_y roll pitch slip_angle" << std::endl;

    std::string anglefilename = out_dir + "/ssc_angle_data_";

    if (right_turn) {
        datafilename.append("_right.txt");
        resfilename.append("_right.txt");
        anglefilename.append("_right.txt");
    } else {
        datafilename.append("_left.txt");
        resfilename.append("_left.txt");
        anglefilename.append("_left.txt");
    }

    // -------------------------------
    // Create the visualization system
    // -------------------------------

    auto vis = chrono_types::make_shared<ChWheeledVehicleVisualSystemVSG>();
    vis->SetWindowTitle("OpenCRG Steering");
    vis->SetWindowSize(1200, 800);
    vis->SetChaseCamera(vehicle_model->TrackPoint(), vehicle_model->CameraDistance(), vehicle_model->CameraHeight());
    vis->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
    vis->EnableShadows();
    vis->AttachVehicle(&vehicle);
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

    // Running average of vehicle speed
    utils::ChRunningAverage dev_filter(500);

    // vehicle.EnableRealtime(true);

    // Number of simulation steps between image outputs
    double render_step_size = 1 / fps;
    int render_steps = (int)std::ceil(render_step_size / step_size);

    // Initialize frame counters
    size_t sim_frame = 0;
    size_t render_frame = 0;

    while (vis->Run()) {
        double time = vehicle.GetSystem()->GetChTime();
        double speed = vehicle.GetSpeed();
        double acc_y = std::pow(speed, 2) / 50.0;
        double roll = vehicle.GetRoll() * CH_RAD_TO_DEG;
        double pitch = vehicle.GetPitch() * CH_RAD_TO_DEG;
        double veh_slip_angle = vehicle.GetSlipAngle() * CH_RAD_TO_DEG;

        // Driver inputs
        DriverInputs driver_inputs = driver.GetInputs();

        // Update sentinel and target location markers for the path-follower controller
        auto sentinelPos = driver.GetSteeringController().GetSentinelLocation();
        auto targetPos = driver.GetSteeringController().GetTargetLocation();
        vis->UpdateVisualModel(sentinelID, ChFrame<>(sentinelPos));
        vis->UpdateVisualModel(targetID, ChFrame<>(targetPos));
        double deviation = dev_filter.Add(
            std::sqrt(std::pow(sentinelPos[0] - targetPos[0], 2) + std::pow(sentinelPos[1] - targetPos[1], 2)));
        if (time > 10)
            csv << time << speed << acc_y << deviation << std::endl;
        if (deviation > max_dev && time > 10) {
            std::cout << "Vehicle leaves turn circle! Test Stopped." << std::endl;
            break;
        }
        if (sim_frame % switch_frame == 0 && sim_frame > 0) {
            std::cout << "Actual course deviation = " << deviation << " m at Ay = " << floor(100.0*acc_y/9.81) << "% G" << std::endl;
            csv_res << acc_y << driver.GetSteering() << std::endl;
            csv_angle << acc_y << roll << pitch << veh_slip_angle << std::endl;
            target_speed += speed_step;
            driver.SetDesiredSpeed(target_speed);
            if (driver.GetInputs().m_throttle > 0.9 && time > 10) {
                std::cout << "Vehicle engine power exhausted! Test Stopped." << std::endl;
                break;
            }
        }

        // Render scene and output images
        if (sim_frame % render_steps == 0) {
            vis->Render();

            if (output_images) {
                std::ostringstream filename;
                filename << out_dir << "/img_" << std::setw(4) << std::setfill('0') << render_frame + 1 << ".png";
                vis->WriteImageToFile(filename.str());
                render_frame++;
            }
        }

        // Update modules (process inputs from other modules)
        driver.Synchronize(time);
        terrain.Synchronize(time);
        vehicle_model->Synchronize(time, driver_inputs, terrain);
        vis->Synchronize(time, driver_inputs);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        terrain.Advance(step_size);
        vehicle_model->Advance(step_size);
        vis->Advance(step_size);
        sys.DoStepDynamics(step_size);

        // Increment simulation frame number
        sim_frame++;

        if (time > t_end)
            break;
    }

    csv.WriteToFile(datafilename);
    csv_res.WriteToFile(resfilename);
    csv_angle.WriteToFile(anglefilename);

    std::string theTitle = "Steady State Cornering Test of '" + vehicle.GetName() + "' - Left Turn";
    std::string speedPlot = out_dir + "/plot_speed_";
    std::string accyPlot = out_dir + "/plot_accy_";
    std::string devPlot = out_dir + "/plot_dev_";
    std::string resPlot = out_dir + "/plot_res_";
    std::string anglePlot = out_dir + "/plot_ang_";
    if (right_turn) {
        theTitle.append("' - Right Turn");
        speedPlot.append("right.gpl");
        accyPlot.append("right.gpl");
        devPlot.append("right.gpl");
        anglePlot.append("right.gpl");
        resPlot.append("right.gpl");
    } else {
        theTitle.append("' - Left Turn");
        speedPlot.append("left.gpl");
        accyPlot.append("left.gpl");
        devPlot.append("left.gpl");
        anglePlot.append("left.gpl");
        resPlot.append("left.gpl");
    }
    if (logged_data_plot) {
        ChGnuPlot mplot_speed(speedPlot);
        mplot_speed.SetGrid();
        mplot_speed.SetTitle(theTitle);
        mplot_speed.SetLabelX("Time (s)");
        mplot_speed.SetLabelY("Speed (m/s)");
        mplot_speed.Plot(datafilename, 1, 2, "", " with lines");

        ChGnuPlot mplot_accy(accyPlot);
        mplot_accy.SetGrid();
        mplot_accy.SetTitle(theTitle);
        mplot_accy.SetLabelX("Time (s)");
        mplot_accy.SetLabelY("Lateral Acceleration (m/s)");
        mplot_accy.Plot(datafilename, 1, 3, "", " with lines");

        ChGnuPlot mplot_dev(devPlot);
        mplot_dev.SetGrid();
        mplot_dev.SetTitle(theTitle);
        mplot_dev.SetLabelX("Time (s)");
        mplot_dev.SetLabelY("Lateral Course Deviation (m)");
        mplot_dev.Plot(datafilename, 1, 4, "", " with lines");
    }
    if (result_plot) {
        ChGnuPlot mplot_res(resPlot);
        mplot_res.SetGrid();
        mplot_res.SetTitle(theTitle);
        mplot_res.SetRangeY(-1, 1);
        mplot_res.SetLabelX("Lateral Acceleration (s)");
        mplot_res.SetLabelY("Steering Signal ()");
        mplot_res.Plot(resfilename, 1, 2, "", " with linespoints");

        ChGnuPlot mplot_angle(anglePlot);
        mplot_angle.SetGrid();
        mplot_angle.SetTitle(theTitle);
        mplot_angle.SetLabelX("Lateral Acceleration (s)");
        mplot_angle.SetLabelY("Chassis Angles (deg)");
        mplot_angle.Plot(anglefilename, 1, 2, "Roll", " with linespoints");
        mplot_angle.Plot(anglefilename, 1, 3, "Pitch", " with linespoints");
        mplot_angle.Plot(anglefilename, 1, 4, "Vehicle Slip Angle", " with linespoints");
    }

    return 0;
}
