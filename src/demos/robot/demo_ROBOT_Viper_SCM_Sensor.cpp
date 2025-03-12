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
// Demo to show Viper Rover operated on SCM Terrain with obstacles
// A radar and a lidar sensors are used in this demo
//
// =============================================================================

#include "chrono_models/robot/viper/Viper.h"

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/physics/ChInertiaUtils.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/SCMTerrain.h"

#include "chrono_sensor/sensors/ChLidarSensor.h"
#include "chrono_sensor/sensors/ChCameraSensor.h"
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

#include "chrono/assets/ChVisualSystem.h"
#ifdef CHRONO_IRRLICHT
    #include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
using namespace chrono::irrlicht;
#endif
#ifdef CHRONO_VSG
    #include "chrono_vsg/ChVisualSystemVSG.h"
using namespace chrono::vsg3d;
#endif

using namespace chrono;
using namespace chrono::viper;
using namespace chrono::sensor;

bool output = false;
const std::string out_dir = GetChronoOutputPath() + "SCM_DEF_SOIL";

// SCM grid spacing
double mesh_resolution = 0.02;

// Enable/disable bulldozing effects
bool enable_bulldozing = false;

// Enable/disable moving patch feature
bool enable_moving_patch = true;

// If true, use provided callback to change soil properties based on location
bool var_params = true;

// Define Viper rover wheel type
ViperWheelType wheel_type = ViperWheelType::RealWheel;

// Run-time visualization system (IRRLICHT or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// Custom callback for setting location-dependent soil properties.
// Note that the location is given in the SCM reference frame.
class MySoilParams : public vehicle::SCMTerrain::SoilParametersCallback {
  public:
    virtual void Set(const ChVector3d& loc,
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

// Use custom material for the Viper Wheel
bool use_custom_mat = false;

// Return customized wheel material parameters
std::shared_ptr<ChContactMaterial> CustomWheelMaterial(ChContactMethod contact_method) {
    float mu = 0.4f;   // coefficient of friction
    float cr = 0.1f;   // coefficient of restitution
    float Y = 2e7f;    // Young's modulus
    float nu = 0.3f;   // Poisson ratio
    float kn = 2e5f;   // normal stiffness
    float gn = 40.0f;  // normal viscous damping
    float kt = 2e5f;   // tangential stiffness
    float gt = 20.0f;  // tangential viscous damping

    switch (contact_method) {
        case ChContactMethod::NSC: {
            auto matNSC = chrono_types::make_shared<ChContactMaterialNSC>();
            matNSC->SetFriction(mu);
            matNSC->SetRestitution(cr);
            return matNSC;
        }
        case ChContactMethod::SMC: {
            auto matSMC = chrono_types::make_shared<ChContactMaterialSMC>();
            matSMC->SetFriction(mu);
            matSMC->SetRestitution(cr);
            matSMC->SetYoungModulus(Y);
            matSMC->SetPoissonRatio(nu);
            matSMC->SetKn(kn);
            matSMC->SetGn(gn);
            matSMC->SetKt(kt);
            matSMC->SetGt(gt);
            return matSMC;
        }
        default:
            return std::shared_ptr<ChContactMaterial>();
    }
}

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Global parameter for moving patch size:
    double wheel_range = 0.5;
    ////double body_range = 1.2;

    // Create a Chrono physical system and associated collision system
    ChSystemNSC sys;
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));

    // Initialize output
    if (output) {
        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            std::cout << "Error creating directory " << out_dir << std::endl;
            return 1;
        }
    }
    utils::ChWriterCSV csv(" ");

    // Create the rover
    auto driver = chrono_types::make_shared<ViperDCMotorControl>();

    Viper viper(&sys, wheel_type);

    viper.SetDriver(driver);
    if (use_custom_mat)
        viper.SetWheelContactMaterial(CustomWheelMaterial(ChContactMethod::NSC));

    viper.Initialize(ChFrame<>(ChVector3d(-5, 0, -0.2), QUNIT));

    // Get wheels and bodies to set up SCM patches
    auto Wheel_1 = viper.GetWheel(ViperWheelID::V_LF)->GetBody();
    auto Wheel_2 = viper.GetWheel(ViperWheelID::V_RF)->GetBody();
    auto Wheel_3 = viper.GetWheel(ViperWheelID::V_LB)->GetBody();
    auto Wheel_4 = viper.GetWheel(ViperWheelID::V_RB)->GetBody();
    auto Body_1 = viper.GetChassis()->GetBody();

    // Obstacles
    std::vector<std::shared_ptr<ChBodyAuxRef>> rocks;
    std::shared_ptr<ChContactMaterial> rockSufaceMaterial = ChContactMaterial::DefaultMaterial(sys.GetContactMethod());

    // rock material
    auto rock_vis_mat = chrono_types::make_shared<ChVisualMaterial>();
    rock_vis_mat->SetAmbientColor({1, 1, 1});  // 0.65f,0.65f,0.65f
    rock_vis_mat->SetDiffuseColor({1, 1, 1});
    rock_vis_mat->SetSpecularColor({1, 1, 1});
    rock_vis_mat->SetUseSpecularWorkflow(true);
    rock_vis_mat->SetRoughness(1.0f);
    rock_vis_mat->SetUseHapke(true);
    rock_vis_mat->SetHapkeParameters(0.32357f, 0.23955f, 0.30452f, 1.80238f, 0.07145f, 0.3f,
                                     float(23.4 * CH_DEG_TO_RAD));

    // Add pre-defined 20 rocks
    for (int i = 0; i < 20; i++) {
        // create a rock
        std::string rock_obj_path;
        if (i % 3 == 0) {
            rock_obj_path = GetChronoDataFile("robot/curiosity/rocks/rock1.obj");
        } else if (i % 3 == 1) {
            rock_obj_path = GetChronoDataFile("robot/curiosity/rocks/rock2.obj");
        } else {
            rock_obj_path = GetChronoDataFile("robot/curiosity/rocks/rock3.obj");
        }

        double scale_ratio = 0.15;
        auto rock_mmesh = ChTriangleMeshConnected::CreateFromWavefrontFile(rock_obj_path, false, true);
        rock_mmesh->Transform(ChVector3d(0, 0, 0), ChMatrix33<>(scale_ratio));  // scale to a different size
        rock_mmesh->RepairDuplicateVertexes(1e-9);                              // if meshes are not watertight

        // compute mass inertia from mesh
        double mmass;
        ChVector3d mcog;
        ChMatrix33<> minertia;
        double mdensity = 8000;  // paramsH->bodyDensity;
        rock_mmesh->ComputeMassProperties(true, mmass, mcog, minertia);
        ChMatrix33<> principal_inertia_rot;
        ChVector3d principal_I;
        ChInertiaUtils::PrincipalInertia(minertia, principal_I, principal_inertia_rot);

        // set the abs orientation, position and velocity
        auto rock_Body = chrono_types::make_shared<ChBodyAuxRef>();
        ChQuaternion<> rock_rot = QuatFromAngleX(CH_PI / 2);
        ChVector3d rock_pos;

        // predefined customized location
        if (i == 0)
            rock_pos = ChVector3d(1.0, -0.5, 0.0);
        else if (i == 1)
            rock_pos = ChVector3d(-0.5, -0.5, 0.0);
        else if (i == 2)
            rock_pos = ChVector3d(2.4, 0.4, 0.0);
        else if (i == 3)
            rock_pos = ChVector3d(0.6, 1.0, 0.0);
        else if (i == 4)
            rock_pos = ChVector3d(5.5, 1.2, 0.0);
        else if (i == 5)
            rock_pos = ChVector3d(1.2, 2.1, 0.0);
        else if (i == 6)
            rock_pos = ChVector3d(-0.3, -2.1, 0.0);
        else if (i == 7)
            rock_pos = ChVector3d(0.4, 2.5, 0.0);
        else if (i == 8)
            rock_pos = ChVector3d(4.2, 1.4, 0.0);
        else if (i == 9)
            rock_pos = ChVector3d(5.0, 2.4, 0.0);
        else if (i == 10)
            rock_pos = ChVector3d(0.6, -1.2, 0.0);
        else if (i == 11)
            rock_pos = ChVector3d(4.8, -1.2, 0.0);
        else if (i == 12)
            rock_pos = ChVector3d(2.5, 2.2, 0.0);
        else if (i == 13)
            rock_pos = ChVector3d(4.7, -2.2, 0.0);
        else if (i == 14)
            rock_pos = ChVector3d(-1.7, 1.5, 0.0);
        else if (i == 15)
            rock_pos = ChVector3d(-2.0, -1.1, 0.0);
        else if (i == 16)
            rock_pos = ChVector3d(-5.0, -2.1, 0.0);
        else if (i == 17)
            rock_pos = ChVector3d(1.5, -0.8, 0.0);
        else if (i == 18)
            rock_pos = ChVector3d(-2.6, 1.6, 0.0);
        else if (i == 19)
            rock_pos = ChVector3d(-2.0, 1.8, 0.0);

        rock_Body->SetFrameCOMToRef(ChFrame<>(mcog, principal_inertia_rot));

        rock_Body->SetMass(mmass * mdensity);  // mmass * mdensity
        rock_Body->SetInertiaXX(mdensity * principal_I);

        rock_Body->SetFrameRefToAbs(ChFrame<>(ChVector3d(rock_pos), ChQuaternion<>(rock_rot)));
        sys.Add(rock_Body);

        rock_Body->SetFixed(false);

        auto rock_ct_shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(rockSufaceMaterial, rock_mmesh,
                                                                                     false, false, 0.005);
        rock_Body->AddCollisionShape(rock_ct_shape);
        rock_Body->EnableCollision(true);

        auto rock_mesh = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        rock_mesh->SetMesh(rock_mmesh);
        rock_mesh->SetBackfaceCull(true);

        if (rock_mesh->GetNumMaterials() == 0) {
            rock_mesh->AddMaterial(rock_vis_mat);
        } else {
            rock_mesh->GetMaterials()[0] = rock_vis_mat;
        }

        rock_Body->AddVisualShape(rock_mesh);

        rocks.push_back(rock_Body);
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
    terrain.SetPlane(ChCoordsys<>(ChVector3d(0, 0, -0.5)));

    // Use a regular grid:
    double length = 15;
    double width = 15;
    terrain.Initialize(length, width, mesh_resolution);

    // Add hapke material to the terrain
    auto lunar_material = chrono_types::make_shared<ChVisualMaterial>();
    lunar_material->SetAmbientColor({0.0, 0.0, 0.0});  // 0.65f,0.65f,0.65f
    lunar_material->SetDiffuseColor({0.7f, 0.7f, 0.7f});
    lunar_material->SetSpecularColor({1.0f, 1.0f, 1.0f});
    lunar_material->SetUseSpecularWorkflow(true);
    lunar_material->SetRoughness(0.8f);
    lunar_material->SetAnisotropy(1.f);
    lunar_material->SetUseHapke(true);
    lunar_material->SetHapkeParameters(0.32357f, 0.23955f, 0.30452f, 1.80238f, 0.07145f, 0.3f,
                                       float(23.4 * CH_DEG_TO_RAD));
    lunar_material->SetClassID(30000);
    lunar_material->SetInstanceID(20000);
    auto mesh = terrain.GetMesh();

    {
        if (mesh->GetNumMaterials() == 0) {
            mesh->AddMaterial(lunar_material);
        } else {
            mesh->GetMaterials()[0] = lunar_material;
        }
    }
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
        terrain.AddMovingPatch(Wheel_1, ChVector3d(0, 0, 0), ChVector3d(0.5, 2 * wheel_range, 2 * wheel_range));
        terrain.AddMovingPatch(Wheel_2, ChVector3d(0, 0, 0), ChVector3d(0.5, 2 * wheel_range, 2 * wheel_range));
        terrain.AddMovingPatch(Wheel_3, ChVector3d(0, 0, 0), ChVector3d(0.5, 2 * wheel_range, 2 * wheel_range));
        terrain.AddMovingPatch(Wheel_4, ChVector3d(0, 0, 0), ChVector3d(0.5, 2 * wheel_range, 2 * wheel_range));

        for (int i = 0; i < 20; i++) {
            terrain.AddMovingPatch(rocks[i], ChVector3d(0, 0, 0), ChVector3d(0.5, 0.5, 0.5));
        }
    }

    // Set some visualization parameters: either with a texture, or with falsecolor plot, etc.
    terrain.SetPlotType(vehicle::SCMTerrain::PLOT_PRESSURE, 0, 20000);

    terrain.SetMeshWireframe(true);

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
            vis_irr->SetCameraVertical(CameraVerticalDir::Z);
            vis_irr->SetWindowSize(800, 600);
            vis_irr->SetWindowTitle("Viper Rover on SCM");
            vis_irr->Initialize();
            vis_irr->AddLogo();
            vis_irr->AddSkyBox();
            vis_irr->AddCamera(ChVector3d(1.0, 2.0, 1.4), ChVector3d(0, 0, wheel_range));
            vis_irr->AddTypicalLights();
            vis_irr->AddLightWithShadow(ChVector3d(-5.0, -0.5, 8.0), ChVector3d(-1, 0, 0), 100, 1, 35, 85, 512,
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
            vis_vsg->SetWindowSize(800, 600);
            vis_vsg->SetWindowTitle("Viper Rover on SCM");
            vis_vsg->AddCamera(ChVector3d(1.0, 2.0, 1.4), ChVector3d(0, 0, wheel_range));
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
    manager->scene->AddPointLight({-10, 0, 50}, {1.f, 1.f, 1.f}, 1000);
    manager->SetVerbose(false);
    Background b;
    // b.mode = BackgroundMode::SOLID_COLOR;
    // b.color_horizon = ChVector3f(0.f, 0.f, 0.f);
    // b.color_zenith = ChVector3f(0.f, 0.f, 0.f);
    b.mode = BackgroundMode::ENVIRONMENT_MAP;
    b.env_tex = GetChronoDataFile("sensor/textures/starmap_2020_4k.hdr");
    manager->scene->SetBackground(b);

    // Create a lidar and add it to the sensor manager
    auto offset_pose = chrono::ChFrame<double>({1.5, 0, 0.4}, QuatFromAngleZ(0));

    auto lidar = chrono_types::make_shared<ChLidarSensor>(viper.GetChassis()->GetBody(),  // body lidar is attached to
                                                          50,                             // scanning rate in Hz
                                                          offset_pose,                    // offset pose
                                                          480,                 // number of horizontal samples
                                                          300,                 // number of vertical channels
                                                          (float)(2 * CH_PI),  // horizontal field of view
                                                          (float)CH_PI / 12, (float)-CH_PI_3,
                                                          140.0f  // vertical field of view
    );
    lidar->SetName("Lidar Sensor 1");
    lidar->SetLag(0.f);
    lidar->SetCollectionWindow(0.02f);

    // Create a filter graph for post-processing the data from the lidar
    // Provides the host access to the Depth,Intensity data
    lidar->PushFilter(chrono_types::make_shared<ChFilterDIAccess>());

    // Renders the raw lidar data
    lidar->PushFilter(chrono_types::make_shared<ChFilterVisualize>(960, 480, "Raw Lidar Depth Data"));

    // Convert Depth,Intensity data to XYZI point
    // cloud data
    lidar->PushFilter(chrono_types::make_shared<ChFilterPCfromDepth>());
    // Render the point cloud
    lidar->PushFilter(chrono_types::make_shared<ChFilterVisualizePointCloud>(960, 600, 0.25, "Lidar Point Cloud"));
    // Access the lidar data as an XYZI buffer
    lidar->PushFilter(chrono_types::make_shared<ChFilterXYZIAccess>());

    // Create a radar and attach to rover chassis
    auto radar = chrono_types::make_shared<ChRadarSensor>(viper.GetChassis()->GetBody(), 50, offset_pose, 240, 120,
                                                          (float)(CH_PI / 1.5), float(CH_PI / 5), 100.f);
    radar->SetName("Radar Sensor");
    radar->SetLag(0.f);
    radar->SetCollectionWindow(0.02f);

    radar->PushFilter(chrono_types::make_shared<ChFilterRadarXYZReturn>("Front Radar"));
    radar->PushFilter(chrono_types::make_shared<ChFilterRadarXYZVisualize>(960, 480, 0.2, "Front Radar"));

    // Add camera
    auto cam = chrono_types::make_shared<ChCameraSensor>(viper.GetChassis()->GetBody(),  // body lidar is attached to
                                                         50,                             // scanning rate in Hz
                                                         offset_pose,                    // offset pose
                                                         960,                            // image width
                                                         480,                            // image height
                                                         CH_PI_3                       // FOV
    );
    cam->SetName("Camera Sensor");
    cam->SetLag(0.f);
    cam->SetCollectionWindow(0.02f);
    cam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(960, 480, "Camera Image"));

    // add lidar, radar and camera to the sensor manager
    manager->AddSensor(lidar);
    manager->AddSensor(radar);
    manager->AddSensor(cam);

    while (vis->Run()) {
#if defined(CHRONO_IRRLICHT) || defined(CHRONO_VSG)
        vis->BeginScene();
        vis->SetCameraTarget(Body_1->GetPos());
        vis->Render();
        ////tools::drawColorbar(vis.get(), 0, 20000, "Pressure yield [Pa]", 1180);
        vis->EndScene();
#endif

        // update sensor manager
        manager->Update();

        if (output) {
            // write drive torques of all four wheels into file
            csv << sys.GetChTime() << viper.GetWheelTracTorque(ViperWheelID::V_LF)
                << viper.GetWheelTracTorque(ViperWheelID::V_RF) << viper.GetWheelTracTorque(ViperWheelID::V_LB)
                << viper.GetWheelTracTorque(ViperWheelID::V_RB) << std::endl;
        }

        sys.DoStepDynamics(5e-4);
        viper.Update();
        ////terrain.PrintStepStatistics(std::cout);
    }

    if (output) {
        csv.WriteToFile(out_dir + "/output.dat");
    }

    return 0;
}
