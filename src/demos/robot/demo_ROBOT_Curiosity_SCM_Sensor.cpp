// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2021 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Jason Zhou
// =============================================================================
//
// Demo to show Curiosity Rover operated on SCM Terrain crossing obstacles
// A radar and a lidar sensors are used in this demo
//
// =============================================================================

#include "chrono_models/robot/curiosity/Curiosity.h"

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChInertiaUtils.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"

#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/assets/ChVisualShapeBox.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/SCMTerrain.h"

#include "chrono_sensor/sensors/ChLidarSensor.h"
#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterPCfromDepth.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/filters/ChFilterVisualizePointCloud.h"
#include "chrono_sensor/filters/ChFilterLidarReduce.h"
#include "chrono_sensor/filters/ChFilterLidarNoise.h"
#include "chrono_sensor/filters/ChFilterSavePtCloud.h"
#include "chrono_sensor/sensors/Sensor.h"
#include "chrono_sensor/filters/ChFilterRadarVisualizeCluster.h"
#include "chrono_sensor/filters/ChFilterRadarXYZReturn.h"
#include "chrono_sensor/filters/ChFilterRadarXYZVisualize.h"

#include "chrono_thirdparty/filesystem/path.h"

#include <chrono>

#ifdef CHRONO_IRRLICHT
    #include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
using namespace chrono::irrlicht;
#endif
#ifdef CHRONO_VSG
    #include "chrono_vsg/ChVisualSystemVSG.h"
using namespace chrono::vsg3d;
#endif

using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::geometry;
using namespace chrono::curiosity;
using namespace chrono::sensor;

using namespace irr;

bool output = false;
const std::string out_dir = GetChronoOutputPath() + "SCM_DEF_SOIL";

// SCM grid spacing
double mesh_resolution = 0.01;

// Enable/disable bulldozing effects
bool enable_bulldozing = false;

// Enable/disable moving patch feature
bool enable_moving_patch = true;

// If true, use provided callback to change soil properties based on location
bool var_params = true;

// Specify rover chassis type
// The options are Scarecrow and FullRover
CuriosityChassisType chassis_type = CuriosityChassisType::FullRover;

// Specify rover wheel type
// The options are RealWheel, SimpleWheel, and CylWheel
CuriosityWheelType wheel_type = CuriosityWheelType::RealWheel;

// Simulation time step
double time_step = 5e-4;

// Run-time visualization system (IRRLICHT or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// -----------------------------------------------------------------------------

// Custom callback for setting location-dependent soil properties.
class MySoilParams : public vehicle::SCMTerrain::SoilParametersCallback {
  public:
    virtual void Set(const ChVector<>& loc,
                     double& Bekker_Kphi,
                     double& Bekker_Kc,
                     double& Bekker_n,
                     double& Mohr_cohesion,
                     double& Mohr_friction,
                     double& Janosi_shear,
                     double& elastic_K,
                     double& damping_R) override {
        Bekker_Kphi = 0.82e6;
        Bekker_Kc = 0.14e4;
        Bekker_n = 1.0;
        Mohr_cohesion = 0.017e4;
        Mohr_friction = 35.0;
        Janosi_shear = 1.78e-2;
        elastic_K = 2e8;
        damping_R = 3e4;
    }
};

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Global parameter for moving patch size:
    double wheel_range = 0.5;

    // Create a Chrono physical system and associated collision system
    ChSystemSMC sys;
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // Initialize output
    if (output) {
        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            std::cout << "Error creating directory " << out_dir << std::endl;
            return 1;
        }
    }
    utils::CSV_writer csv(" ");

    // Curiosity rover initial position and orientation
    ChVector<double> body_pos(-5, -0.2, 0);
    ChQuaternion<> body_rot = Q_from_AngX(-CH_C_PI / 2);

    // Create a Curiosity rover
    Curiosity rover(&sys, chassis_type, wheel_type);

    // Create a CuriosityDriver to command the rover
    auto driver = chrono_types::make_shared<CuriositySpeedDriver>(1.0, CH_C_PI);
    rover.SetDriver(driver);
    rover.Initialize(ChFrame<>(body_pos, body_rot));

    std::shared_ptr<ChBodyAuxRef> rock_1;
    std::shared_ptr<ChBodyAuxRef> rock_2;
    std::shared_ptr<ChBodyAuxRef> rock_3;
    std::shared_ptr<ChBodyAuxRef> rock_4;
    std::shared_ptr<ChBodyAuxRef> rock_5;
    std::shared_ptr<ChBodyAuxRef> rock_6;

    // create default SMC materials for the obstacles
    std::shared_ptr<ChMaterialSurface> rockSufaceMaterial = ChMaterialSurface::DefaultMaterial(sys.GetContactMethod());

    for (int i = 0; i < 2; i++) {
        // Create a rock
        std::string rock1_obj_path = GetChronoDataFile("robot/curiosity/rocks/rock1.obj");
        double scale_ratio = 0.8;
        auto rock_1_mmesh = ChTriangleMeshConnected::CreateFromWavefrontFile(rock1_obj_path, false, true);
        rock_1_mmesh->Transform(ChVector<>(0, 0, 0), ChMatrix33<>(scale_ratio));  // scale to a different size
        rock_1_mmesh->RepairDuplicateVertexes(1e-9);                              // if meshes are not watertight

        // compute mass inertia from mesh
        double mmass;
        ChVector<> mcog;
        ChMatrix33<> minertia;
        double mdensity = 8000;  // paramsH->bodyDensity;
        rock_1_mmesh->ComputeMassProperties(true, mmass, mcog, minertia);
        ChMatrix33<> principal_inertia_rot;
        ChVector<> principal_I;
        ChInertiaUtils::PrincipalInertia(minertia, principal_I, principal_inertia_rot);

        // set the abs orientation, position and velocity
        auto rock1_Body = chrono_types::make_shared<ChBodyAuxRef>();
        ChQuaternion<> rock1_rot = ChQuaternion<>(1, 0, 0, 0);
        ChVector<> rock1_pos;
        if (i == 0) {
            rock1_pos = ChVector<>(-2.5, -0.3, -1.0);
        } else {
            rock1_pos = ChVector<>(-2.5, -0.3, 1.0);
        }

        rock1_Body->SetFrame_COG_to_REF(ChFrame<>(mcog, principal_inertia_rot));

        rock1_Body->SetMass(mmass * mdensity);  // mmass * mdensity
        rock1_Body->SetInertiaXX(mdensity * principal_I);

        rock1_Body->SetFrame_REF_to_abs(ChFrame<>(ChVector<>(rock1_pos), ChQuaternion<>(rock1_rot)));
        sys.Add(rock1_Body);

        rock1_Body->SetBodyFixed(false);

        auto rock1_ct_shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(rockSufaceMaterial, rock_1_mmesh, false,
                                                                                false, 0.005);
        rock1_Body->AddCollisionShape(rock1_ct_shape);
        rock1_Body->SetCollide(true);

        auto rock1_mesh = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        rock1_mesh->SetMesh(rock_1_mmesh);
        rock1_mesh->SetBackfaceCull(true);
        rock1_Body->AddVisualShape(rock1_mesh);

        if (i == 0) {
            rock_1 = rock1_Body;
        } else {
            rock_2 = rock1_Body;
        }
    }

    for (int i = 0; i < 2; i++) {
        // Create a rock
        std::string rock2_obj_path = GetChronoDataFile("robot/curiosity/rocks/rock1.obj");
        double scale_ratio = 0.45;
        auto rock_2_mmesh = ChTriangleMeshConnected::CreateFromWavefrontFile(rock2_obj_path, false, true);
        rock_2_mmesh->Transform(ChVector<>(0, 0, 0), ChMatrix33<>(scale_ratio));  // scale to a different size
        rock_2_mmesh->RepairDuplicateVertexes(1e-9);                              // if meshes are not watertight

        // compute mass inertia from mesh
        double mmass;
        ChVector<> mcog;
        ChMatrix33<> minertia;
        double mdensity = 8000;  // paramsH->bodyDensity;
        rock_2_mmesh->ComputeMassProperties(true, mmass, mcog, minertia);
        ChMatrix33<> principal_inertia_rot;
        ChVector<> principal_I;
        ChInertiaUtils::PrincipalInertia(minertia, principal_I, principal_inertia_rot);

        // set the abs orientation, position and velocity
        auto rock2_Body = chrono_types::make_shared<ChBodyAuxRef>();
        ChQuaternion<> rock2_rot = ChQuaternion<>(1, 0, 0, 0);
        ChVector<> rock2_pos;
        if (i == 0) {
            rock2_pos = ChVector<>(-1.0, -0.3, -1.0);
        } else {
            rock2_pos = ChVector<>(-1.0, -0.3, 1.0);
        }

        rock2_Body->SetFrame_COG_to_REF(ChFrame<>(mcog, principal_inertia_rot));

        rock2_Body->SetMass(mmass * mdensity);  // mmass * mdensity
        rock2_Body->SetInertiaXX(mdensity * principal_I);

        rock2_Body->SetFrame_REF_to_abs(ChFrame<>(ChVector<>(rock2_pos), ChQuaternion<>(rock2_rot)));
        sys.Add(rock2_Body);

        rock2_Body->SetBodyFixed(false);

        auto rock2_ct_shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(rockSufaceMaterial, rock_2_mmesh,
                                                                                      false, false, 0.005);
        rock2_Body->AddCollisionShape(rock2_ct_shape);
        rock2_Body->SetCollide(true);

        auto rock2_mesh = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        rock2_mesh->SetMesh(rock_2_mmesh);
        rock2_mesh->SetBackfaceCull(true);
        rock2_Body->AddVisualShape(rock2_mesh);

        if (i == 0) {
            rock_3 = rock2_Body;
        } else {
            rock_4 = rock2_Body;
        }
    }

    for (int i = 0; i < 2; i++) {
        // Create a rock
        std::string rock3_obj_path = GetChronoDataFile("robot/curiosity/rocks/rock3.obj");
        double scale_ratio = 0.45;
        auto rock_3_mmesh = ChTriangleMeshConnected::CreateFromWavefrontFile(rock3_obj_path, false, true);
        rock_3_mmesh->Transform(ChVector<>(0, 0, 0), ChMatrix33<>(scale_ratio));  // scale to a different size
        rock_3_mmesh->RepairDuplicateVertexes(1e-9);                              // if meshes are not watertight

        // compute mass inertia from mesh
        double mmass;
        ChVector<> mcog;
        ChMatrix33<> minertia;
        double mdensity = 8000;  // paramsH->bodyDensity;
        rock_3_mmesh->ComputeMassProperties(true, mmass, mcog, minertia);
        ChMatrix33<> principal_inertia_rot;
        ChVector<> principal_I;
        ChInertiaUtils::PrincipalInertia(minertia, principal_I, principal_inertia_rot);

        // set the abs orientation, position and velocity
        auto rock3_Body = chrono_types::make_shared<ChBodyAuxRef>();
        ChQuaternion<> rock3_rot = ChQuaternion<>(1, 0, 0, 0);
        ChVector<> rock3_pos;
        if (i == 0) {
            rock3_pos = ChVector<>(0.5, -0.3, -1.0);
        } else {
            rock3_pos = ChVector<>(0.5, -0.3, 1.0);
        }

        rock3_Body->SetFrame_COG_to_REF(ChFrame<>(mcog, principal_inertia_rot));

        rock3_Body->SetMass(mmass * mdensity);  // mmass * mdensity
        rock3_Body->SetInertiaXX(mdensity * principal_I);

        rock3_Body->SetFrame_REF_to_abs(ChFrame<>(ChVector<>(rock3_pos), ChQuaternion<>(rock3_rot)));
        sys.Add(rock3_Body);

        rock3_Body->SetBodyFixed(false);

        auto rock3_ct_shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(rockSufaceMaterial, rock_3_mmesh,
                                                                                      false, false, 0.005);
        rock3_Body->AddCollisionShape(rock3_ct_shape);
        rock3_Body->SetCollide(true);

        auto rock3_mesh = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        rock3_mesh->SetMesh(rock_3_mmesh);
        rock3_mesh->SetBackfaceCull(true);
        rock3_Body->AddVisualShape(rock3_mesh);

        if (i == 0) {
            rock_5 = rock3_Body;
        } else {
            rock_6 = rock3_Body;
        }
    }

    //
    // THE DEFORMABLE TERRAIN
    //

    // Create the 'deformable terrain' object
    vehicle::SCMTerrain terrain(&sys);

    // Displace/rotate the terrain reference plane.
    // Note that SCMTerrain uses a default ISO reference frame (Z up). Since the mechanism is modeled here in
    // a Y-up global frame, we rotate the terrain plane by -90 degrees about the X axis.
    // Note: Irrlicht uses a Y-up frame
    terrain.SetPlane(ChCoordsys<>(ChVector<>(0, -0.5, 0), Q_from_AngX(-CH_C_PI_2)));

    // Use a regular grid:
    double length = 14;
    double width = 4;
    terrain.Initialize(length, width, mesh_resolution);

    // Set the soil terramechanical parameters
    if (var_params) {
        // Here we use the soil callback defined at the beginning of the code
        auto my_params = chrono_types::make_shared<MySoilParams>();
        terrain.RegisterSoilParametersCallback(my_params);
    } else {
        // If var_params is set to be false, these parameters will be used
        terrain.SetSoilParameters(0.2e6,  // Bekker Kphi
                                  0,      // Bekker Kc
                                  1.1,    // Bekker n exponent
                                  0,      // Mohr cohesive limit (Pa)
                                  30,     // Mohr friction limit (degrees)
                                  0.01,   // Janosi shear coefficient (m)
                                  4e7,    // Elastic stiffness (Pa/m), before plastic yield, must be > Kphi
                                  3e4     // Damping (Pa s/m), proportional to negative vertical speed (optional)
        );
    }

    // Set up bulldozing factors
    if (enable_bulldozing) {
        terrain.EnableBulldozing(true);  // inflate soil at the border of the rut
        terrain.SetBulldozingParameters(
            55,  // angle of friction for erosion of displaced material at the border of the rut
            1,   // displaced material vs downward pressed material.
            5,   // number of erosion refinements per timestep
            6);  // number of concentric vertex selections subject to erosion
    }

    // We need to add a moving patch under every wheel
    // Or we can define a large moving patch at the pos of the rover body
    if (enable_moving_patch) {
        // add moving patch for the SCM terrain
        // the bodies were retrieved from the rover instance
        terrain.AddMovingPatch(rover.GetWheel(CuriosityWheelID::C_LF)->GetBody(), ChVector<>(0, 0, 0),
                               ChVector<>(0.5, 2 * wheel_range, 2 * wheel_range));
        terrain.AddMovingPatch(rover.GetWheel(CuriosityWheelID::C_RF)->GetBody(), ChVector<>(0, 0, 0),
                               ChVector<>(0.5, 2 * wheel_range, 2 * wheel_range));
        terrain.AddMovingPatch(rover.GetWheel(CuriosityWheelID::C_LM)->GetBody(), ChVector<>(0, 0, 0),
                               ChVector<>(0.5, 2 * wheel_range, 2 * wheel_range));
        terrain.AddMovingPatch(rover.GetWheel(CuriosityWheelID::C_RM)->GetBody(), ChVector<>(0, 0, 0),
                               ChVector<>(0.5, 2 * wheel_range, 2 * wheel_range));
        terrain.AddMovingPatch(rover.GetWheel(CuriosityWheelID::C_LB)->GetBody(), ChVector<>(0, 0, 0),
                               ChVector<>(0.5, 2 * wheel_range, 2 * wheel_range));
        terrain.AddMovingPatch(rover.GetWheel(CuriosityWheelID::C_RB)->GetBody(), ChVector<>(0, 0, 0),
                               ChVector<>(0.5, 2 * wheel_range, 2 * wheel_range));

        // add moving patch for all obstacles
        terrain.AddMovingPatch(rock_1, ChVector<>(0, 0, 0), ChVector<>(0.5, 2 * wheel_range, 2 * wheel_range));
        terrain.AddMovingPatch(rock_2, ChVector<>(0, 0, 0), ChVector<>(0.5, 2 * wheel_range, 2 * wheel_range));
        terrain.AddMovingPatch(rock_3, ChVector<>(0, 0, 0), ChVector<>(0.5, 2 * wheel_range, 2 * wheel_range));
        terrain.AddMovingPatch(rock_4, ChVector<>(0, 0, 0), ChVector<>(0.5, 2 * wheel_range, 2 * wheel_range));
        terrain.AddMovingPatch(rock_5, ChVector<>(0, 0, 0), ChVector<>(0.5, 2 * wheel_range, 2 * wheel_range));
        terrain.AddMovingPatch(rock_6, ChVector<>(0, 0, 0), ChVector<>(0.5, 2 * wheel_range, 2 * wheel_range));
    }

    // Set some visualization parameters: either with a texture, or with falsecolor plot, etc.
    terrain.SetPlotType(vehicle::SCMTerrain::PLOT_PRESSURE, 0, 20000);

    terrain.GetMesh()->SetWireframe(true);

    // Create the run-time visualization interface
#ifndef CHRONO_IRRLICHT
    if (vis_type == ChVisualSystem::Type::IRRLICHT)
        vis_type = ChVisualSystem::Type::VSG;
#endif
#ifndef CHRONO_VSG
    if (vis_type == ChVisualSystem::Type::VSG)
        vis_type = ChVisualSystem::Type::IRRLICHT;
#endif
    std::shared_ptr<ChVisualSystem> vis;
    switch (vis_type) {
        case ChVisualSystem::Type::IRRLICHT: {
#ifdef CHRONO_IRRLICHT
            auto vis_irr = chrono_types::make_shared<ChVisualSystemIrrlicht>();
            vis_irr->AttachSystem(&sys);
            vis_irr->SetCameraVertical(CameraVerticalDir::Y);
            vis_irr->SetWindowSize(800, 600);
            vis_irr->SetWindowTitle("Curiosity Obstacle Crossing on SCM");
            vis_irr->Initialize();
            vis_irr->AddLogo();
            vis_irr->AddSkyBox();
            vis_irr->AddCamera(ChVector<>(-1.0, 1.0, 3.0), ChVector<>(-5.0, 0.0, 0.0));
            vis_irr->AddTypicalLights();
            vis_irr->AddLightWithShadow(ChVector<>(-5.0, 8.0, -0.5), ChVector<>(-1, 0, 0), 100, 1, 35, 85, 512,
                                        ChColor(0.8f, 0.8f, 0.8f));
            vis_irr->EnableShadows();

            vis = vis_irr;
#endif
            break;
        }
        default:
        case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
            auto vis_vsg = chrono_types::make_shared<ChVisualSystemVSG>();
            vis_vsg->AttachSystem(&sys);
            vis_vsg->SetCameraVertical(CameraVerticalDir::Y);
            vis_vsg->SetWindowSize(800, 600);
            vis_vsg->SetWindowTitle("Curiosity Obstacle Crossing on SCM");
            vis_vsg->AddCamera(ChVector<>(-1.0, 1.0, 3.0), ChVector<>(-5.0, 0.0, 0.0));
            vis_vsg->AddGuiColorbar("Pressure yield [kPa]", 0.0, 20.0);
            vis_vsg->Initialize();

            vis = vis_vsg;
#endif
            break;
        }
    }

    //
    // SENSOR SIMULATION
    //

    // Create a sensor manager
    auto manager = chrono_types::make_shared<ChSensorManager>(&sys);
    manager->scene->AddPointLight({100, 100, 100}, {0.4f, 0.4f, 0.4f}, 500);
    manager->SetVerbose(false);

    // Create a lidar and add it to the sensor manager
    auto offset_pose = chrono::ChFrame<double>({0, 0, 0.16}, Q_from_AngAxis(0, {0, 0, 1}));

    auto lidar = chrono_types::make_shared<ChLidarSensor>(rover.GetChassis()->GetBody(),  // body lidar is attached to
                                                          25,                             // scanning rate in Hz
                                                          offset_pose,                    // offset pose
                                                          480,                   // number of horizontal samples
                                                          300,                   // number of vertical channels
                                                          (float)(2 * CH_C_PI),  // horizontal field of view
                                                          (float)CH_C_PI / 12, (float)-CH_C_PI / 6,
                                                          120.0f  // vertical field of view
    );
    lidar->SetName("Lidar Sensor 1");
    lidar->SetLag(0.f);
    lidar->SetCollectionWindow(0.04f);

    // Create a filter graph for post-processing the data from the lidar
    // Provides the host access to the Depth,Intensity data
    lidar->PushFilter(chrono_types::make_shared<ChFilterDIAccess>());

    // Renders the raw lidar data
    // lidar->PushFilter(chrono_types::make_shared<ChFilterVisualize>(800, 600, "Raw Lidar Depth Data"));

    // Convert Depth,Intensity data to XYZI point
    // cloud data
    lidar->PushFilter(chrono_types::make_shared<ChFilterPCfromDepth>());
    // Render the point cloud
    lidar->PushFilter(
        chrono_types::make_shared<ChFilterVisualizePointCloud>(960, 600, 0.2, "Bottom Lidar Point Cloud"));
    // Access the lidar data as an XYZI buffer
    lidar->PushFilter(chrono_types::make_shared<ChFilterXYZIAccess>());
    lidar->PushFilter(chrono_types::make_shared<ChFilterLidarNoiseXYZI>(0.01f, 0.001f, 0.001f, 0.01f));

    // Radar Offset
    auto offset_pose_1 = chrono::ChFrame<double>({-0.5, 0, 0.4}, Q_from_AngAxis(0, {0, 0, 1}));

    // Create a radar and attach to rover chassis
    auto radar = chrono_types::make_shared<ChRadarSensor>(rover.GetChassis()->GetBody(), 25, offset_pose_1, 300, 200,
                                                          (float)(CH_C_PI / 1.5), float(CH_C_PI / 5), 100.f);
    radar->SetName("Radar Sensor");
    radar->SetLag(0.f);
    radar->SetCollectionWindow(0.04f);

    radar->PushFilter(chrono_types::make_shared<ChFilterRadarXYZReturn>("Front Radar"));
    radar->PushFilter(chrono_types::make_shared<ChFilterRadarXYZVisualize>(1920, 200, 0.2, "Front Radar"));

    // add sensor to the manager
    manager->AddSensor(lidar);
    manager->AddSensor(radar);

    // Simulation loop
    while (vis->Run()) {
#if defined(CHRONO_IRRLICHT) || defined(CHRONO_VSG)
        vis->BeginScene();
        vis->Render();
        ////tools::drawColorbar(vis.get(), 0, 20000, "Pressure yield [Pa]", 1600);
        vis->EndScene();
#endif

        if (output) {
            // write drive torques of all six wheels into file
            csv << sys.GetChTime() << rover.GetWheelTracTorque(CuriosityWheelID::C_LF)
                << rover.GetWheelTracTorque(CuriosityWheelID::C_RF) << rover.GetWheelTracTorque(CuriosityWheelID::C_LM)
                << rover.GetWheelTracTorque(CuriosityWheelID::C_RM) << rover.GetWheelTracTorque(CuriosityWheelID::C_LB)
                << rover.GetWheelTracTorque(CuriosityWheelID::C_RB) << std::endl;
        }
        rover.Update();

        // update sensor manager
        manager->Update();

        sys.DoStepDynamics(time_step);

        ////std::cout << "--------- " << sys.GetChTime() << std::endl;
        ////terrain.PrintStepStatistics(std::cout);
    }

    if (output) {
        // write output data into file
        csv.write_to_file(out_dir + "/output.dat");
    }

    return 0;
}
