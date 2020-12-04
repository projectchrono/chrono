// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2020 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Jay Taves
// =============================================================================
//
// Demo code illustrating synchronization of the SCM semi-empirical model for
// deformable soil
//
// See also in chrono_vehicle:
// - demo_VEH_DeformableSoil
// - demo_VEH_DeformableSoilAndTire
// - demo_VEH_HMMWV_DefSoil
//
// =============================================================================

#include <chrono>

#include "chrono_models/vehicle/hmmwv/HMMWV.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"

#include "chrono_synchrono/communication/mpi/SynMPIManager.h"
#include "chrono_thirdparty/cxxopts/ChCLI.h"

#include "chrono_synchrono/agent/SynWheeledVehicleAgent.h"
#include "chrono_synchrono/vehicle/SynWheeledVehicle.h"

#include "chrono_synchrono/terrain/SynSCMTerrain.h"
#include "chrono_synchrono/terrain/SynRigidTerrain.h"

#include "chrono_synchrono/visualization/SynVisualizationManager.h"

#ifdef CHRONO_IRRLICHT
#include "chrono_synchrono/visualization/SynIrrVehicleVisualization.h"
#endif

#ifdef CHRONO_SENSOR
#include "chrono_sensor/ChCameraSensor.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterSave.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_synchrono/visualization/SynSensorVisualization.h"
using namespace chrono::sensor;
#endif

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::synchrono;
using namespace chrono::vehicle::hmmwv;

// =============================================================================

// Better conserve mass by displacing soil to the sides of a rut
const bool bulldozing = true;

ChContactMethod contact_method = ChContactMethod::SMC;
double end_time = 1000;
double step_size = 3e-3;

// Time interval between two render frames
double render_step_size = 1.0 / 50;  // FPS = 50

// How often SynChrono state messages are interchanged
float heartbeat = 1e-2;  // 100[Hz]

// Forward declares for straightforward helper functions
std::string StringFromContactMethod(ChContactMethod contact);
ChContactMethod ContactMethodFromString(std::string str);
void AddCommandLineOptions(ChCLI& cli);

// =============================================================================

int main(int argc, char* argv[]) {
    SynMPIConfig config = MPI_CONFIG_DEFAULT;

    // Need dynamic reserve as we don't know how many nodes the vehicles will hit
    config.memory_mode = SynMPIMemoryMode::DYNAMIC_RESERVE;

    SynMPIManager mpi_manager(argc, argv, config);
    int rank = mpi_manager.GetRank();
    int num_ranks = mpi_manager.GetNumRanks();

    // -----------------------------------------------------
    // CLI SETUP - Get most parameters from the command line
    // -----------------------------------------------------

    ChCLI cli(argv[0]);

    AddCommandLineOptions(cli);

    if (!cli.Parse(argc, argv, rank == 0))
        mpi_manager.Exit();

    // Normal simulation options
    step_size = cli.GetAsType<double>("step_size");
    end_time = cli.GetAsType<double>("end_time");
    heartbeat = cli.GetAsType<double>("heartbeat");
    contact_method = ContactMethodFromString(cli.GetAsType<std::string>("contact_method"));

    const double size_x = cli.GetAsType<double>("sizeX");
    const double size_y = cli.GetAsType<double>("sizeY");
    const double cam_x = cli.GetAsType<std::vector<double>>("c_pos")[0];
    const double cam_y = cli.GetAsType<std::vector<double>>("c_pos")[1];
    const double dpu = cli.GetAsType<double>("dpu");
    const int cam_res_width = cli.GetAsType<std::vector<int>>("res")[0];
    const int cam_res_height = cli.GetAsType<std::vector<int>>("res")[1];
    const bool using_scm_terrain = cli.GetAsType<std::string>("terrain_type") == "SCM";
    const bool use_sensor_vis = cli.HasValueInVector<int>("sens", rank);
    const bool use_irrlicht_vis = !use_sensor_vis && cli.HasValueInVector<int>("irr", rank);

    mpi_manager.SetHeartbeat(heartbeat);
    mpi_manager.SetEndTime(end_time);

    // --------------------
    // Agent Initialization
    // --------------------

    auto agent = chrono_types::make_shared<SynWheeledVehicleAgent>(rank);

    mpi_manager.AddAgent(agent, rank);

    // Use up more of the mesh by not placing vehicles in the middle
    ChVector<> offset(-size_x / 2 + 5, 0, 0);

    ChVector<> init_loc;
    ChQuaternion<> init_rot;
    std::vector<ChVector<>> curve_pts;
    if (rank % 2 == 0) {
        // Start even vehicles in a row on the south side, driving north
        init_loc = offset + ChVector<>(0, 2.0 * (rank + 1), 0.5);
        init_rot = Q_from_AngZ(0);
        curve_pts = {init_loc, init_loc + ChVector<>(100, 0, 0)};
    } else {
        // Start odd vehicles staggered going up the west edge, driving east
        init_loc = offset + ChVector<>(2.0 * (rank - 1), -5.0 - 2.0 * (rank - 1), 0.5);
        init_rot = Q_from_AngZ(CH_C_PI / 2);
        curve_pts = {init_loc, init_loc + ChVector<>(0, 100, 0)};
    }

    // -------------------------
    // Chrono::Vehicle specifics
    // -------------------------
    auto hmmwv = chrono_types::make_shared<HMMWV_Full>();

    hmmwv->SetContactMethod(contact_method);
    hmmwv->SetChassisCollisionType(CollisionType::NONE);
    hmmwv->SetChassisFixed(false);
    hmmwv->SetInitPosition(ChCoordsys<>(init_loc, init_rot));
    hmmwv->SetTireType(TireModelType::TMEASY);

    if (using_scm_terrain)
        hmmwv->SetTireType(TireModelType::RIGID);

    hmmwv->SetTireStepSize(step_size);
    hmmwv->Initialize();

    hmmwv->SetChassisVisualizationType(VisualizationType::MESH);
    hmmwv->SetSuspensionVisualizationType(VisualizationType::MESH);
    hmmwv->SetSteeringVisualizationType(VisualizationType::NONE);
    hmmwv->SetWheelVisualizationType(VisualizationType::MESH);
    hmmwv->SetTireVisualizationType(VisualizationType::MESH);

    auto vehicle = chrono_types::make_shared<SynCustomWheeledVehicle<HMMWV_Full>>(hmmwv);
    agent->SetVehicle(vehicle);

    vehicle->SetZombieVisualizationFiles("hmmwv/hmmwv_chassis.obj",     //
                                         "hmmwv/hmmwv_rim.obj",         //
                                         "hmmwv/hmmwv_tire_left.obj");  //
    vehicle->SetNumWheels(4);

    // ----------------------
    // Terrain specific setup
    // ----------------------
    if (using_scm_terrain) {
        auto scm = chrono_types::make_shared<SCMDeformableTerrain>(agent->GetSystem());

        // Configure the SCM terrain
        if (bulldozing) {
            scm->EnableBulldozing(bulldozing);
            scm->SetBulldozingParameters(
                55,   // angle of friction for erosion of displaced material at the border of the rut
                1,    // displaced material vs downward pressed material.
                5,    // number of erosion refinements per timestep
                10);  // number of concentric vertex selections subject to erosion
        }

        // Only relevant for Irrlicht visualization, gives some nice colors
        scm->SetPlotType(SCMDeformableTerrain::PLOT_SINKAGE, 0, 0.1);
        scm->GetMesh()->SetWireframe(true);

        // The physics do not change when you add a moving patch, you just make it much easier for the SCM
        // implementation to do its job by restricting where it has to look for contacts
        scm->AddMovingPatch(vehicle->GetVehicle().GetChassisBody(), ChVector<>(0, 0, 0), ChVector<>(5, 3, 1));

        scm->Initialize(size_x, size_y, 1. / dpu);

        auto terrain = chrono_types::make_shared<SynSCMTerrain>(scm, agent->GetSystem());
        agent->SetTerrain(terrain);

        // Choice of soft parameters is arbitrary
        SCMParameters params;
        params.InitializeParametersAsSoft();
        terrain->SetSoilParametersFromStruct(&params);

        // Add texture for the terrain
        auto vis_mat = chrono_types::make_shared<ChVisualMaterial>();
        vis_mat->SetSpecularColor({.1f, .1f, .1f});
        vis_mat->SetKdTexture(GetChronoDataFile("sensor/textures/grass_texture.jpg"));
        scm->GetMesh()->material_list.push_back(vis_mat);
    } else {
        MaterialInfo minfo;
        minfo.mu = 0.9f;
        minfo.cr = 0.01f;
        minfo.Y = 2e7f;
        auto patch_mat = minfo.CreateMaterial(contact_method);

        auto rigid = chrono_types::make_shared<RigidTerrain>(agent->GetSystem());
        auto patch = rigid->AddPatch(patch_mat, ChVector<>(0, 0, 0), ChVector<>(0, 0, 1), size_x, size_y);
        rigid->Initialize();

        // Terrain visualization
        // For irrlicht
        patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 200);
        // For sensor
        auto patch_asset = patch->GetGroundBody()->GetAssets()[0];
        if (auto visual_asset = std::dynamic_pointer_cast<ChVisualization>(patch_asset)) {
            auto box_texture = chrono_types::make_shared<ChVisualMaterial>();
            box_texture->SetKdTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"));
            // FresnelMax and SpecularColor should make it less shiny
            box_texture->SetFresnelMax(0.2);
            box_texture->SetSpecularColor({0.2, 0.2, 0.2});

            visual_asset->material_list.push_back(box_texture);
        }
        agent->SetTerrain(chrono_types::make_shared<SynRigidTerrain>(rigid));
    }

    // ---------------------------
    // Controller for the vehicles
    // ---------------------------

    // What we defined earlier, a straight line
    auto path = chrono_types::make_shared<ChBezierCurve>(curve_pts);
    auto driver = chrono_types::make_shared<ChPathFollowerDriver>(vehicle->GetVehicle(), path, "Box path", 10);

    // Reasonable defaults for the underlying PID
    driver->GetSpeedController().SetGains(0.4, 0, 0);
    driver->GetSteeringController().SetGains(0.4, 0.1, 0.2);
    driver->GetSteeringController().SetLookAheadDistance(5);

    // Wrap the ChDriver in a SynVehicleBrain and add it to our agent
    auto brain = chrono_types::make_shared<SynVehicleBrain>(rank, driver, agent->GetChVehicle());
    agent->SetBrain(brain);

    // ---------------------------
    // Visualization
    // ---------------------------

    // Visualization manager is a convenient way to bring Sensor and Irrlicht under one roof
    auto vis_manager = chrono_types::make_shared<SynVisualizationManager>();
    agent->SetVisualizationManager(vis_manager);

#ifdef CHRONO_IRRLICHT
    if (use_irrlicht_vis) {
        auto irr_vis = chrono_types::make_shared<SynIrrVehicleVisualization>(driver, step_size, render_step_size);
        irr_vis->InitializeAsDefaultChaseCamera(agent->GetVehicle());
        vis_manager->AddVisualization(irr_vis);
    }
#endif

#ifdef CHRONO_SENSOR
    if (use_sensor_vis) {
        auto sen_vis = chrono_types::make_shared<SynSensorVisualization>();
        sen_vis->InitializeDefaultSensorManager(agent->GetSystem());

        // Give the camera a fixed place to live
        auto origin = chrono_types::make_shared<ChBody>();
        origin->SetBodyFixed(true);
        agent->GetSystem()->AddBody(origin);

        // Happens to be a reasonable-looking height
        ChVector<> camera_loc(cam_x, cam_y, 25);

        // Rotations to get a nice angle
        ChQuaternion<> rotation = QUNIT;
        const bool USE_ISO_VIEW = true;
        if (USE_ISO_VIEW) {
            ChQuaternion<> qA = Q_from_AngAxis(35 * CH_C_DEG_TO_RAD, VECT_Y);
            ChQuaternion<> qB = Q_from_AngAxis(135 * CH_C_DEG_TO_RAD, VECT_Z);
            rotation = rotation >> qA >> qB;
        } else {
            // Top down view
            ChQuaternion<> qA = Q_from_AngAxis(90 * CH_C_DEG_TO_RAD, VECT_Y);
            ChQuaternion<> qB = Q_from_AngAxis(180 * CH_C_DEG_TO_RAD, VECT_Z);
            rotation = rotation >> qA >> qB;
        }

        auto overhead_camera = chrono_types::make_shared<chrono::sensor::ChCameraSensor>(
            origin,                                         // body camera is attached to
            30,                                             // update rate in Hz
            chrono::ChFrame<double>(camera_loc, rotation),  // offset pose
            cam_res_width,                                  // image width
            cam_res_height,                                 // image height
            CH_C_PI / 3);

        overhead_camera->SetName("Overhead Cam");
        overhead_camera->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());

        // Do we draw a window on the screen?
        if (cli.GetAsType<bool>("sens_vis"))
            overhead_camera->PushFilter(chrono_types::make_shared<ChFilterVisualize>(cam_res_width, cam_res_height));

        // Do we save images to disc?
        std::string file_path = std::string("SENSOR_OUTPUT/scm") + std::to_string(rank) + std::string("/");
        if (cli.GetAsType<bool>("sens_save"))
            overhead_camera->PushFilter(chrono_types::make_shared<ChFilterSave>(file_path));

        sen_vis->SetSensor(overhead_camera);
        vis_manager->AddVisualization(sen_vis);
    }
#endif

    mpi_manager.Barrier();
    mpi_manager.Initialize();

    std::cout << "Rank " << rank << " entering simulation loop." << std::endl;

    int step_number = 0;

    // Simulation Loop
    while (mpi_manager.IsOk()) {
        mpi_manager.Advance(heartbeat * step_number++);
        mpi_manager.Synchronize();
        mpi_manager.Update();
    }

    std::cout << "Rank " << rank << " completed successfully." << std::endl;

    return 0;
}

ChContactMethod ContactMethodFromString(std::string str) {
    if (str == "SMC")
        return ChContactMethod::SMC;
    if (str == "NSC")
        return ChContactMethod::NSC;
    throw ChException(str + " is not a valid ChContactMethod (SMC or NSC)");
}

std::string StringFromContactMethod(ChContactMethod contact) {
    switch (contact) {
        case ChContactMethod::NSC:
            return "NSC";
        case ChContactMethod::SMC:
            return "SMC";
        default:
            throw ChException("ChContactMethod improperly enumerated in StringFromContactMethod");
    }
}

void AddCommandLineOptions(ChCLI& cli) {
    // Standard demo options
    cli.AddOption<double>("Simulation", "step_size", "Step size", std::to_string(step_size));
    cli.AddOption<double>("Simulation", "end_time", "End time", std::to_string(end_time));
    cli.AddOption<double>("Simulation", "heartbeat", "Heartbeat", std::to_string(heartbeat));
    cli.AddOption<std::string>("Simulation", "contact_method", "Contact Method",
                               StringFromContactMethod(contact_method), "NSC/SMC");

    // SCM specific options
    cli.AddOption<double>("Demo", "d,dpu", "Divisions per unit", "20");
    cli.AddOption<std::string>("Demo", "t,terrain_type", "Terrain Type", "Rigid", "Rigid,SCM");

    // Visualization is the only reason you should be shy about terrain size. The implementation can easily handle a
    // practically infinite terrain (provided you don't need to visualize it)
    cli.AddOption<double>("Demo", "x,sizeX", "Size in the X", "100");
    cli.AddOption<double>("Demo", "y,sizeY", "Size in the Y", "50");

    // Irrlicht options
    cli.AddOption<std::vector<int>>("Irrlicht", "irr", "Ranks for irrlicht usage", "-1");

    // Sensor options
    cli.AddOption<std::vector<int>>("Sensor", "sens", "Ranks for sensor usage", "-1");
    cli.AddOption<bool>("Sensor", "sens_save", "Toggle sensor saving ON", "false");
    cli.AddOption<bool>("Sensor", "sens_vis", "Toggle sensor visualization ON", "false");

    cli.AddOption<std::vector<int>>("Demo", "r,res", "Camera resolution", "1280,720", "width,height");
    cli.AddOption<std::vector<double>>("Demo", "c_pos", "Camera Position", "-15,-25", "X,Y");
}