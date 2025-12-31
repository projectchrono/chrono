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
#include "chrono/input_output/ChWriterCSV.h"
#include "chrono/physics/ChMassProperties.h"

#include "chrono_vehicle/ChVehicleDataPath.h"
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
    #include "chrono_vehicle/visualization/ChScmVisualizationVSG.h"
    #include "chrono_vsg/ChVisualSystemVSG.h"
using namespace chrono::vsg3d;
#endif

using namespace chrono;
using namespace chrono::viper;
using namespace chrono::sensor;

// -----------------------------------------------------------------------------

bool output = false;

// SCM grid spacing
double mesh_resolution = 0.02;

// Enable/disable bulldozing effects
bool enable_bulldozing = true;

// Enable/disable active domains feature
bool enable_active_domains = true;

// If true, use provided callback to change soil properties based on location
bool var_params = true;

// Define Viper rover wheel type
ViperWheelType wheel_type = ViperWheelType::RealWheel;

// Use custom material for the Viper wheel
bool use_custom_mat = false;

// Run-time visualization system (IRRLICHT or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// -----------------------------------------------------------------------------

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

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Reference heights
    double terrain_height = -0.5;
    double rover_height = -0.2;
    double rock_height = -0.3;

    // Wheel dimensions (for SCM active domains)
    double wheel_diameter = 0.6;
    double wheel_width = 0.4;
    ChVector3d wheel_size(wheel_diameter, wheel_width, wheel_diameter);

    // Rock dimensions (for SCM active domains)
    ChVector3d rock_size(0.25, 0.25, 0.25);

    // Create a Chrono physical system and associated collision system
    ChSystemNSC sys;
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));

    // Initialize output
    std::string out_dir = GetChronoOutputPath() + "ROBOT_Viper_SCM_SENSOR";
    if (output) {
        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            std::cout << "Error creating directory " << out_dir << std::endl;
            return 1;
        }
    }
    ChWriterCSV csv(" ");

    // VIPER ROVER

    // Create the rover
    auto driver = chrono_types::make_shared<ViperDCMotorControl>();

    Viper viper(&sys, wheel_type);

    viper.SetDriver(driver);
    if (use_custom_mat)
        viper.SetWheelContactMaterial(CustomWheelMaterial(ChContactMethod::NSC));

    viper.Initialize(ChFrame<>(ChVector3d(-5, 0, rover_height), QUNIT));

    // Get wheels and bodies to set up SCM patches
    auto Wheel_1 = viper.GetWheel(ViperWheelID::V_LF)->GetBody();
    auto Wheel_2 = viper.GetWheel(ViperWheelID::V_RF)->GetBody();
    auto Wheel_3 = viper.GetWheel(ViperWheelID::V_LB)->GetBody();
    auto Wheel_4 = viper.GetWheel(ViperWheelID::V_RB)->GetBody();
    auto Body_1 = viper.GetChassis()->GetBody();

    // OBSTACLES

    std::shared_ptr<ChContactMaterial> rock_ct_mat = ChContactMaterial::DefaultMaterial(sys.GetContactMethod());

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

    // Rock parameters
    std::vector<ChVector3d> rock_pos = {
        ChVector3d(+1.0, -0.5, rock_height),  //
        ChVector3d(-0.5, -0.5, rock_height),  //
        ChVector3d(+2.4, +0.4, rock_height),  //
        ChVector3d(+0.6, +1.0, rock_height),  //
        ChVector3d(+5.5, +1.2, rock_height),  //
        ChVector3d(+1.2, +2.1, rock_height),  //
        ChVector3d(-0.3, -2.1, rock_height),  //
        ChVector3d(+0.4, +2.5, rock_height),  //
        ChVector3d(+4.2, +1.4, rock_height),  //
        ChVector3d(+5.0, +2.4, rock_height),  //
        ChVector3d(+0.6, -1.2, rock_height),  //
        ChVector3d(+4.8, -1.2, rock_height),  //
        ChVector3d(+2.5, +2.2, rock_height),  //
        ChVector3d(+4.7, -2.2, rock_height),  //
        ChVector3d(-1.7, +1.5, rock_height),  //
        ChVector3d(-2.0, -1.1, rock_height),  //
        ChVector3d(-5.0, -2.1, rock_height),  //
        ChVector3d(+1.5, -0.8, rock_height),  //
        ChVector3d(-2.6, +1.6, rock_height),  //
        ChVector3d(-2.0, +1.8, rock_height)   //
    };
    ChQuaternion<> rock_rot = QuatFromAngleX(CH_PI / 2);

    double rock_density = 8000;

    std::vector<std::string> rock_obj_file = {
        GetChronoDataFile("robot/curiosity/rocks/rock1.obj"),  //
        GetChronoDataFile("robot/curiosity/rocks/rock2.obj"),  //
        GetChronoDataFile("robot/curiosity/rocks/rock3.obj"),  //
    };
    std::vector<std::shared_ptr<ChTriangleMeshConnected>> rock_mesh;
    std::vector<std::shared_ptr<ChCollisionShapeTriangleMesh>> rock_ct_shape;
    std::vector<std::shared_ptr<ChVisualShapeTriangleMesh>> rock_vis_shape;
    double scale_ratio = 0.15;
    for (auto obj : rock_obj_file) {
        // triangular mesh
        auto mesh = ChTriangleMeshConnected::CreateFromWavefrontFile(obj, false, true);
        mesh->Transform(ChVector3d(0, 0, 0), ChMatrix33<>(scale_ratio));  // scale to a different size
        mesh->RepairDuplicateVertexes(1e-9);                              // if meshes are not watertight
        rock_mesh.push_back(mesh);

        // contact shape
        auto ct_shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(rock_ct_mat, mesh, false, false, 0.005);
        rock_ct_shape.push_back(ct_shape);

        // visualization shape
        auto vis_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        vis_shape->SetMesh(mesh);
        vis_shape->SetBackfaceCull(true);
        if (vis_shape->GetNumMaterials() == 0)
            vis_shape->AddMaterial(rock_vis_mat);
        else
            vis_shape->GetMaterials()[0] = rock_vis_mat;
        rock_vis_shape.push_back(vis_shape);
    }

    // Add pre-defined 20 rocks
    std::vector<std::shared_ptr<ChBodyAuxRef>> rocks;

    for (int i = 0; i < 20; i++) {
        // rock shape index
        int j = i % 3;

        // compute mass and inertia from mesh (for density = 1)
        double mass;
        ChVector3d cog;
        ChMatrix33<> inertia;
        rock_mesh[j]->ComputeMassProperties(true, mass, cog, inertia);
        ChMatrix33<> principal_inertia_rot;
        ChVector3d principal_inertia;
        ChInertiaUtils::PrincipalInertia(inertia, principal_inertia, principal_inertia_rot);

        // set the abs orientation, position and velocity
        auto rock_body = chrono_types::make_shared<ChBodyAuxRef>();
        rock_body->SetFixed(false);
        rock_body->SetMass(rock_density * mass);
        rock_body->SetInertiaXX(rock_density * principal_inertia);
        rock_body->SetFrameCOMToRef(ChFrame<>(cog, principal_inertia_rot));
        rock_body->SetFrameRefToAbs(ChFrame<>(rock_pos[i], rock_rot));
        sys.Add(rock_body);

        rock_body->AddCollisionShape(rock_ct_shape[j]);
        rock_body->EnableCollision(true);

        rock_body->AddVisualShape(rock_vis_shape[j]);

        rocks.push_back(rock_body);
    }

    // DEFORMABLE TERRAIN

    // Create the SCM terrain object
    vehicle::SCMTerrain terrain(&sys);

    // Displace/rotate the terrain reference frame.
    // Note that SCMTerrain uses a default ISO reference frame (Z up). Since the mechanism is modeled here in
    // a Y-up global frame, we rotate the terrain frame by -90 degrees about the X axis.
    terrain.SetReferenceFrame(ChCoordsys<>(ChVector3d(0, 0, terrain_height)));

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

    if (mesh->GetNumMaterials() == 0) {
        mesh->AddMaterial(lunar_material);
    } else {
        mesh->GetMaterials()[0] = lunar_material;
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

    // Add an active domains for each wheel and for each obstacle
    if (enable_active_domains) {
        terrain.AddActiveDomain(Wheel_1, ChVector3d(0, 0, 0), wheel_size);
        terrain.AddActiveDomain(Wheel_2, ChVector3d(0, 0, 0), wheel_size);
        terrain.AddActiveDomain(Wheel_3, ChVector3d(0, 0, 0), wheel_size);
        terrain.AddActiveDomain(Wheel_4, ChVector3d(0, 0, 0), wheel_size);

        for (int i = 0; i < 20; i++) {
            terrain.AddActiveDomain(rocks[i], ChVector3d(0, 0, 0), rock_size);
        }
    }

    // Set some visualization parameters: either with a texture, or with falsecolor plot, etc.
    terrain.SetColormap(ChColormap::Type::BROWN);
    terrain.SetPlotType(vehicle::SCMTerrain::PLOT_SINKAGE, 0, 0.08);
    ////terrain.SetPlotType(vehicle::SCMTerrain::PLOT_PRESSURE, 0, 20000);

    terrain.SetMeshWireframe(false);

    // SENSOR SIMULATION

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
                                                          480,                // number of horizontal samples
                                                          300,                // number of vertical channels
                                                          (float)(CH_2PI),    // horizontal field of view
                                                          (float)CH_PI / 12,  // max vertical angle
                                                          (float)-CH_PI_3,    // min verticval angle
                                                          140.0f              // max distance
    );
    lidar->SetName("Lidar Sensor 1");
    lidar->SetLag(0.f);
    lidar->SetCollectionWindow(0.02f);

    // Create a filter graph for post-processing the data from the lidar
    // Provides the host access to the (depth,intensity) data
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
    auto cam = chrono_types::make_shared<ChCameraSensor>(viper.GetChassis()->GetBody(),  // body camera is attached to
                                                         50,                             // scanning rate in Hz
                                                         offset_pose,                    // offset pose
                                                         960,                            // image width
                                                         480,                            // image height
                                                         CH_PI_3                         // FOV
    );
    cam->SetName("Camera Sensor");
    cam->SetLag(0.f);
    cam->SetCollectionWindow(0.02f);
    cam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(960, 480, "Camera Image"));

    // add lidar, radar and camera to the sensor manager
    manager->AddSensor(lidar);
    manager->AddSensor(radar);
    manager->AddSensor(cam);

    // RUN_TIME VISUALIZATION

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
            vis_irr->AddCamera(ChVector3d(1.0, 2.0, 1.4), ChVector3d(0, 0, wheel_diameter));
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
            // SCM plugin
            auto visSCM = chrono_types::make_shared<vehicle::ChScmVisualizationVSG>(&terrain);

            auto vis_vsg = chrono_types::make_shared<ChVisualSystemVSG>();
            vis_vsg->AttachSystem(&sys);
            vis_vsg->AttachPlugin(visSCM);
            vis_vsg->SetWindowSize(1280, 800);
            vis_vsg->SetWindowTitle("Viper Rover on SCM");
            vis_vsg->SetBackgroundColor(ChColor(0, 0, 0));
            vis_vsg->AddCamera(ChVector3d(1.0, 2.0, 1.4), ChVector3d(0, 0, wheel_diameter));
            vis_vsg->SetLightIntensity(1.0f);
            vis_vsg->SetLightDirection(CH_PI, 1.37);
            vis_vsg->EnableShadows();
            vis_vsg->Initialize();

            vis = vis_vsg;
#endif
            break;
        }
    }

    // SIMULATION LOOP

    while (vis->Run()) {
#if defined(CHRONO_IRRLICHT) || defined(CHRONO_VSG)
        vis->BeginScene();
        vis->SetCameraTarget(Body_1->GetPos());
        vis->Render();
        vis->EndScene();
#endif

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
