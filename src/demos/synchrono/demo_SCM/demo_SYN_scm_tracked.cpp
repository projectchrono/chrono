#include "chrono_synchrono/communication/mpi/SynMPIManager.h"
#include "chrono_synchrono/cli/SynCLI.h"
#include "chrono_synchrono/utils/SynDataLoader.h"

#include "chrono_synchrono/visualization/SynVisualizationManager.h"

#include "chrono_synchrono/agent/SynTrackedVehicleAgent.h"

#include "chrono_synchrono/terrain/SynSCMTerrain.h"
#include "chrono_synchrono/terrain/SynRigidTerrain.h"

#ifdef CHRONO_IRRLICHT
#include "chrono_synchrono/visualization/SynIrrVehicleVisualization.h"
#endif

#ifdef CHRONO_SENSOR
#include "chrono_synchrono/visualization/SynSensorVisualization.h"
#endif

#include "chrono_vehicle/driver/ChPathFollowerDriver.h"

#include <chrono>

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::synchrono;

int main(int argc, char* argv[]) {
    SynMPIConfig config = MPI_CONFIG_DEFAULT;
    config.memory_mode = SynMPIMemoryMode::DYNAMIC_RESERVE;

    // Initialize the MPIManager
    SynMPIManager mpi_manager(argc, argv, config);
    int rank = mpi_manager.GetRank();
    int num_ranks = mpi_manager.GetNumRanks();

    // CLI tools for default synchrono demos and some custom options
    SynCLI cli(argv[0]);
    cli.AddDefaultDemoOptions();

    cli.AddOption<std::vector<int>>("Demo", "r,res", "Camera resolution", "1280,720", "width,height");
    cli.AddOption<std::vector<double>>("Demo", "c_pos", "Camera Position", "-15,-25", "X,Y");
    cli.AddOption<std::string>("Demo", "t,terrain_type", "Terrain Type", "Rigid", "Rigid,SCM");
    cli.AddOption<double>("Demo", "d,dpu", "Divisions per unit", "20");
    cli.AddOption<double>("Demo", "x,sizeX", "Size in the X", "100");
    cli.AddOption<double>("Demo", "y,sizeY", "Size in the Y", "50");

    if (!cli.Parse(argc, argv, rank == 0))
        mpi_manager.Exit();

    // Set values
    const double size_x = cli.GetAsType<double>("sizeX");
    const double size_y = cli.GetAsType<double>("sizeY");
    const double cam_x = cli.GetAsType<std::vector<double>>("c_pos")[0];
    const double cam_y = cli.GetAsType<std::vector<double>>("c_pos")[1];
    const int CAM_RES_WIDTH = cli.GetAsType<std::vector<int>>("res")[0];
    const int CAM_RES_HEIGHT = cli.GetAsType<std::vector<int>>("res")[1];
    const double dpu = cli.GetAsType<double>("dpu");
    const std::string terrain_type = cli.GetAsType<std::string>("terrain_type");

    // --------------------
    // Agent Initialization
    // --------------------
    auto agent = chrono_types::make_shared<SynTrackedVehicleAgent>(rank);
    mpi_manager.AddAgent(agent, rank);

    // Use up more of the mesh by not placing vehicles in the middle
    ChVector<> offset(-15 / 2 + 5, 0, 0);

    ChVector<> init_loc;
    ChQuaternion<> init_rot;
    bool is_even = rank % 2 == 0;
    if (is_even) {
        // Start even vehicles in a row on the south side, driving north
        init_loc = offset + ChVector<>(0, 2.0 * (rank - 1), 0.75);
        init_rot = Q_from_AngZ(0);
    } else {
        // Start odd vehicles staggered going up the west edge, driving east
        init_loc = offset + ChVector<>(2.0 * (rank - 1), -7.5 - 2.0 * (rank - 1), 0.75);
        init_rot = Q_from_AngZ(CH_C_PI / 2);
    }
    ChVector<> opp_loc = is_even ? ChVector<>(100, 0, 0) : ChVector<>(0, 100, 0);
    std::vector<ChVector<>> curve_pts = {init_loc, init_loc + opp_loc};

    // -------
    // Vehicle
    // -------
    std::string vehicle_filename = GetSynDataFile("vehicle/M113.json");
    auto vehicle = chrono_types::make_shared<SynTrackedVehicle>(vehicle_filename, CONTACT_METHOD);
    vehicle->Initialize(ChCoordsys<>(init_loc, init_rot));
    agent->SetVehicle(vehicle);

    // -------
    // Terrain
    // -------
    if (terrain_type.compare("SCM") == 0) {
        auto scm = chrono_types::make_shared<SCMDeformableTerrain>(agent->GetSystem());

        scm->Initialize(size_x, size_y, 1. / dpu);

        // Configure the SCM terrain
        bool enable_bulldozing = true;
        if (enable_bulldozing) {
            scm->EnableBulldozing(enable_bulldozing);
            scm->SetBulldozingParameters(
                55,   // angle of friction for erosion of displaced material at the border of the rut
                1,    // displaced material vs downward pressed material.
                5,    // number of erosion refinements per timestep
                10);  // number of concentric vertex selections subject to erosion
        }

        scm->SetPlotType(SCMDeformableTerrain::PLOT_SINKAGE, 0, 0.1);
        scm->GetMesh()->SetWireframe(true);

        auto terrain = chrono_types::make_shared<SynSCMTerrain>(scm, agent->GetSystem());
        agent->SetTerrain(terrain);

        SCMParameters params;
        params.InitializeParametersAsSoft();
        terrain->SetSoilParametersFromStruct(&params);

        scm->AddMovingPatch(vehicle->GetVehicle().GetChassisBody(), ChVector<>(0, 0, 0), ChVector<>(7.5, 7.5, 1));

#ifdef CHRONO_SENSOR
        // Add texture for the terrain
        auto vis_mat = chrono_types::make_shared<ChVisualMaterial>();
        vis_mat->SetSpecularColor({.1f, .1f, .1f});
        vis_mat->SetKdTexture(GetChronoDataFile("sensor/textures/grass_texture.jpg"));
        scm->GetMesh()->material_list.push_back(vis_mat);
#endif
    } else if (terrain_type.compare("Rigid") == 0) {
        std::string terrain_filename = GetSynDataFile("terrain/RigidPlane.json");
        auto terrain = chrono_types::make_shared<SynRigidTerrain>(agent->GetSystem(), terrain_filename);
        agent->SetTerrain(terrain);
    } else {
        std::string out = "\"" + terrain_type + "\" is not a viable terrain type. Must be \"SCM\" or \"Rigid\".";
        std::cout << out << std::endl;
        mpi_manager.Exit();
    }

    auto path = chrono_types::make_shared<ChBezierCurve>(curve_pts);
    auto driver = chrono_types::make_shared<ChPathFollowerDriver>(vehicle->GetVehicle(), path, "Box path", 10);

    driver->GetSpeedController().SetGains(0.4, 0, 0);
    driver->GetSteeringController().SetGains(0.4, 0, 0);
    driver->GetSteeringController().SetLookAheadDistance(5);

    auto brain = chrono_types::make_shared<SynVehicleBrain>(rank, driver, agent->GetChVehicle());
    agent->SetBrain(brain);

    auto vis_manager = chrono_types::make_shared<SynVisualizationManager>();
    agent->AttachVisualizationManager(vis_manager);
#ifdef CHRONO_IRRLICHT
    if (cli.HasValueInVector<int>("irr", rank)) {
        auto irr_vis = chrono_types::make_shared<SynIrrVehicleVisualization>(driver, STEP_SIZE);
        irr_vis->InitializeAsDefaultTrackedChaseCamera(agent->GetTrackedVehicle(), 8);
        vis_manager->AddVisualization(irr_vis);
    }
#endif  // IRRLICHT

#ifdef CHRONO_SENSOR
    if (cli.HasValueInVector<int>("sens", rank)) {
        std::string path = std::string("SENSOR_OUTPUT/SCM_TRACKED/M113_") + std::to_string(rank) + std::string("/");

        std::shared_ptr<SynSensorVisualization> sen_vis = chrono_types::make_shared<SynSensorVisualization>();
        sen_vis->InitializeDefaultSensorManager(agent->GetSystem());
        auto cam = chrono_types::make_shared<ChCameraSensor>(
            agent->GetChVehicle().GetChassisBody(),  // body camera is attached to
            30,                                      // update rate in Hz
            chrono::ChFrame<double>({-15, 0, 4}, Q_from_AngAxis(CH_C_PI / 20, {0, 1, 0})),  // offset pose
            1280,                                                                           // image width
            720,                                                                            // image height
            CH_C_PI / 3);                                                                   // horizontal field of view
        cam->SetLag(1 / 30.);
        sen_vis->SetSensor(cam);
        if (cli.GetAsType<bool>("sens_save"))
            sen_vis->AddFilterSave(path);
        if (cli.GetAsType<bool>("sens_vis"))
            sen_vis->AddFilterVisualize();
        vis_manager->AddVisualization(sen_vis);
    }
    // if (cli.HasValueInVector<int>("sens", rank)) {
    //     auto sen_vis = chrono_types::make_shared<SynSensorVisualization>();
    //     sen_vis->InitializeDefaultSensorManager(agent->GetSystem());

    //     auto origin = chrono_types::make_shared<ChBody>();
    //     origin->SetBodyFixed(true);
    //     agent->GetSystem()->AddBody(origin);

    //     ChVector<> camera_loc(cam_x, cam_y, 65);

    //     // Rotations to get a nice angle
    //     ChQuaternion<> rotation = QUNIT;
    //     // ChQuaternion<> qA = Q_from_AngAxis(35 * CH_C_DEG_TO_RAD, VECT_Y);
    //     // ChQuaternion<> qB = Q_from_AngAxis(135 * CH_C_DEG_TO_RAD, VECT_Z);
    //     ChQuaternion<> qA = Q_from_AngAxis(90 * CH_C_DEG_TO_RAD, VECT_Y);
    //     ChQuaternion<> qB = Q_from_AngAxis(180 * CH_C_DEG_TO_RAD, VECT_Z);
    //     rotation = rotation >> qA >> qB;

    //     auto intersection_camera = chrono_types::make_shared<chrono::sensor::ChCameraSensor>(
    //         origin,                                         // body camera is attached to
    //         30,                                             // update rate in Hz
    //         chrono::ChFrame<double>(camera_loc, rotation),  // offset pose
    //         CAM_RES_WIDTH,                                  // image width
    //         CAM_RES_HEIGHT,                                 // image height
    //         CH_C_PI / 3);

    //     intersection_camera->SetName("Intersection Cam");
    //     intersection_camera->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());
    //     if (cli.GetAsType<bool>("sens_vis")) {
    //         intersection_camera->PushFilter(
    //             chrono_types::make_shared<ChFilterVisualize>(CAM_RES_WIDTH, CAM_RES_HEIGHT));
    //     }

    //     std::string file_path = std::string("SENSOR_OUTPUT/Sedan") + std::to_string(rank) + std::string("/");
    //     if (cli.GetAsType<bool>("sens_save")) {
    //         intersection_camera->PushFilter(chrono_types::make_shared<ChFilterSave>(file_path));
    //     }

    //     sen_vis->SetSensor(intersection_camera);
    //     vis_manager->AddVisualization(sen_vis);
    // }
#endif

    mpi_manager.Barrier();
    mpi_manager.Initialize();
    mpi_manager.Barrier();

    // Simulation Loop
    while (mpi_manager.IsOk()) {
        mpi_manager.Advance();
        mpi_manager.Synchronize();
        mpi_manager.Update();
    }

    std::cout << "Rank " << rank << " completed successfully" << std::endl;

    return 0;
}
