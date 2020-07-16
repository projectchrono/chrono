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
// Author: Radu Serban
// =============================================================================
//
// Chrono demonstration of a HMMWV on SCMDeformableTerrain
// Sensors are used to visualize the scene
//
// =============================================================================

#include <cstdio>
#include <cmath>
#include <vector>

#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/ChDriver.h"
#include "chrono_vehicle/terrain/SCMDeformableTerrain.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h"

#include "chrono_models/vehicle/hmmwv/HMMWV.h"

#include "chrono_thirdparty/filesystem/path.h"

#include "chrono_sensor/ChCameraSensor.h"
#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/filters/ChFilterSave.h"
#include "chrono_sensor/utils/ChVisualMaterialUtils.h"
#include "chrono_sensor/filters/ChFilterAccess.h"

using namespace chrono;
using namespace chrono::collision;
using namespace chrono::irrlicht;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;
using namespace chrono::sensor;

using std::cout;
using std::endl;

// -----------------------------------------------------------------------------
// Terrain parameters
// -----------------------------------------------------------------------------

// Dimensions
double terrainHeight = 0;
double terrainLength = 20.0;  // size in X direction
double terrainWidth = 20.0;   // size in Y direction

// Divisions (X and Y)
int divLength = 20 * 20;  // 1024;
int divWidth = 20 * 20;   // 512;

// -----------------------------------------------------------------------------
// Vehicle parameters
// -----------------------------------------------------------------------------

// Type of wheel/tire (controls both contact and visualization)
enum WheelType { CYLINDRICAL, LUGGED };
WheelType wheel_type = LUGGED;

// Type of terrain
enum TerrainType { DEFORMABLE_SOIL, RIGID_SOIL };
TerrainType terrain_type = DEFORMABLE_SOIL;

// Type of powertrain model (SHAFTS, SIMPLE)
PowertrainModelType powertrain_model = PowertrainModelType::SHAFTS;

// Drive type (FWD, RWD, or AWD)
DrivelineType drive_type = DrivelineType::AWD;

// Chassis visualization (MESH, PRIMITIVES, NONE)
VisualizationType chassis_vis = VisualizationType::PRIMITIVES;

// Initial vehicle position and orientation
ChVector<> initLoc(-5, -2, 0.6);
ChQuaternion<> initRot(1, 0, 0, 0);

// Contact material properties
float Y_t = 1.0e6f;
float cr_t = 0.1f;
float mu_t = 0.8f;

// -----------------------------------------------------------------------------
// Camera parameters
// -----------------------------------------------------------------------------

// Update rate in Hz
int update_rate = 30;

// Image width and height
unsigned int image_width = 1920;
unsigned int image_height = 1080;

// Camera's horizontal field of view
float fov = CH_C_PI / 3.;

// Lag (in seconds) between sensing and when data becomes accessible
float lag = 0;

// Exposure (in seconds) of each image
float exposure_time = 0;

// -----------------------------------------------------------------------------
// Simulation parameters
// -----------------------------------------------------------------------------

// Simulation step size
double step_size = 3e-3;

// Time interval between two render frames (1/FPS)
double render_step_size = 1.0 / 100;

// Point on chassis tracked by the camera
ChVector<> trackPoint(0.0, 0.0, 1.75);

// Output directories
const std::string out_dir = GetChronoOutputPath() + "HMMWV_DEF_SOIL";
const std::string img_dir = out_dir + "/IMG";
const std::string sens_dir = out_dir + "/SENSOR_OUTPUT";

// Visualization output
bool img_output = false;

// Simulation end time
float end_time = 20.0f;

// Save camera images
bool save = false;

// Render camera images
bool vis = true;

class MyDriver : public ChDriver {
  public:
    MyDriver(ChVehicle& vehicle, double delay) : ChDriver(vehicle), m_delay(delay) {}
    ~MyDriver() {}

    virtual void Synchronize(double time) override {
        m_throttle = 0;
        m_steering = 0;
        m_braking = 0;

        double eff_time = time - m_delay;

        // Do not generate any driver inputs for a duration equal to m_delay.
        if (eff_time < 0)
            return;

        if (eff_time > 0.2)
            m_throttle = 0.7;
        else
            m_throttle = 3.5 * eff_time;

        if (eff_time < 2)
            m_steering = 0;
        else
            m_steering = 0.6 * std::sin(CH_C_2PI * (eff_time - 2) / 6);
    }

  private:
    double m_delay;
};

// =============================================================================

void CreateLuggedGeometry(std::shared_ptr<ChBody> wheel_body, std::shared_ptr<ChMaterialSurfaceSMC> wheel_material) {
    std::string lugged_file("hmmwv/lugged_wheel_section.obj");
    geometry::ChTriangleMeshConnected lugged_mesh;
    ChConvexDecompositionHACDv2 lugged_convex;
    utils::LoadConvexMesh(vehicle::GetDataFile(lugged_file), lugged_mesh, lugged_convex);
    int num_hulls = lugged_convex.GetHullCount();

    auto coll_model = wheel_body->GetCollisionModel();
    coll_model->ClearModel();

    // Assemble the tire contact from 15 segments, properly offset.
    // Each segment is further decomposed in convex hulls.
    for (int iseg = 0; iseg < 15; iseg++) {
        ChQuaternion<> rot = Q_from_AngAxis(iseg * 24 * CH_C_DEG_TO_RAD, VECT_Y);
        for (int ihull = 0; ihull < num_hulls; ihull++) {
            std::vector<ChVector<> > convexhull;
            lugged_convex.GetConvexHullResult(ihull, convexhull);
            coll_model->AddConvexHull(wheel_material, convexhull, VNULL, rot);
        }
    }

    // Add a cylinder to represent the wheel hub.
    coll_model->AddCylinder(wheel_material, 0.223, 0.223, 0.126);
    coll_model->BuildModel();

    // Visualization
    auto trimesh = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
    trimesh->LoadWavefrontMesh(vehicle::GetDataFile("hmmwv/lugged_wheel.obj"), false, false);

    auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
    trimesh_shape->SetMesh(trimesh);
    trimesh_shape->SetName("lugged_wheel");
    trimesh_shape->SetStatic(true);
    wheel_body->AddAsset(trimesh_shape);

    auto mcolor = chrono_types::make_shared<ChColorAsset>(0.3f, 0.3f, 0.3f);
    wheel_body->AddAsset(mcolor);

    auto vis_mat = chrono_types::make_shared<ChVisualMaterial>();
    vis_mat->SetDiffuseColor({.2f, .2f, .2f});
    vis_mat->SetSpecularColor({.1f, .1f, .1f});
    vis_mat->SetRoughness(.5);
    vis_mat->SetFresnelMax(.1);
    trimesh_shape->material_list.push_back(vis_mat);
}

// =============================================================================

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // --------------------
    // Create HMMWV vehicle
    // --------------------
    HMMWV_Full my_hmmwv;
    my_hmmwv.SetContactMethod(ChContactMethod::SMC);
    my_hmmwv.SetChassisFixed(false);
    my_hmmwv.SetInitPosition(ChCoordsys<>(initLoc, initRot));
    my_hmmwv.SetPowertrainType(powertrain_model);
    my_hmmwv.SetDriveType(drive_type);
    my_hmmwv.SetTireType(TireModelType::RIGID);
    my_hmmwv.Initialize();

    VisualizationType wheel_vis = (wheel_type == CYLINDRICAL) ? VisualizationType::MESH : VisualizationType::NONE;
    my_hmmwv.SetChassisVisualizationType(chassis_vis);
    my_hmmwv.SetWheelVisualizationType(wheel_vis);

    ChSystem* system = my_hmmwv.GetSystem();

    // --------------------------------------------------------
    // Set wheel contact material.
    // If needed, modify wheel contact and visualization models
    // --------------------------------------------------------
    auto wheel_material = chrono_types::make_shared<ChMaterialSurfaceSMC>();
    wheel_material->SetFriction(mu_t);
    wheel_material->SetYoungModulus(Y_t);
    wheel_material->SetRestitution(cr_t);

    for (auto& axle : my_hmmwv.GetVehicle().GetAxles()) {
        auto wheelBodyL = axle->m_wheels[0]->GetSpindle();
        CreateLuggedGeometry(wheelBodyL, wheel_material);
        auto wheelBodyR = axle->m_wheels[1]->GetSpindle();
        CreateLuggedGeometry(wheelBodyR, wheel_material);
    }

    // --------------------
    // Create driver system
    // --------------------
    MyDriver driver(my_hmmwv.GetVehicle(), 0.5);
    driver.Initialize();

    // ------------------
    // Create the terrain
    // ------------------
    SCMDeformableTerrain terrain(system);
    terrain.SetSoilParameters(2e6,   // Bekker Kphi
                              0,     // Bekker Kc
                              1.1,   // Bekker n exponent
                              0,     // Mohr cohesive limit (Pa)
                              30,    // Mohr friction limit (degrees)
                              0.01,  // Janosi shear coefficient (m)
                              2e8,   // Elastic stiffness (Pa/m), before plastic yield
                              3e4    // Damping (Pa s/m), proportional to negative vertical speed (optional)
    );
    terrain.SetAutomaticRefinement(false);
    terrain.SetAutomaticRefinementResolution(0.04);
    terrain.AddMovingPatch(my_hmmwv.GetChassisBody(), ChVector<>(0, 0, 0), 5, 3);
    terrain.SetPlotType(vehicle::SCMDeformableTerrain::PLOT_SINKAGE, 0, 0.1);

    terrain.Initialize(terrainHeight, terrainLength, terrainWidth, divLength, divWidth);

    auto vis_mat = chrono_types::make_shared<ChVisualMaterial>();
    vis_mat->SetSpecularColor({.1f, .1f, .1f});
    vis_mat->SetKdTexture(GetChronoDataFile("sensor/textures/grass_texture.jpg"));
    terrain.GetMesh()->material_list.push_back(vis_mat);

    // ---------------------------------------
    // Create the vehicle Irrlicht application
    // ---------------------------------------
    ChWheeledVehicleIrrApp app(&my_hmmwv.GetVehicle(), L"HMMWV Deformable Soil Demo");
    app.SetSkyBox();
    app.AddTypicalLights(irr::core::vector3df(30.f, -30.f, 100.f), irr::core::vector3df(30.f, 50.f, 100.f), 250, 130);
    app.SetChaseCamera(trackPoint, 6.0, 0.5);
    app.SetTimestep(step_size);
    app.AssetBindAll();
    app.AssetUpdateAll();

    // -----------------
    // Initialize output
    // -----------------
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    if (img_output) {
        if (!filesystem::create_directory(filesystem::path(img_dir))) {
            std::cout << "Error creating directory " << img_dir << std::endl;
            return 1;
        }
    }

    // ---------------
    // Simulation loop
    // ---------------

    std::cout << "Total vehicle mass: " << my_hmmwv.GetTotalMass() << std::endl;

    // Solver settings.
    system->SetSolverMaxIterations(50);

    // Number of simulation steps between two 3D view render frames
    int render_steps = (int)std::ceil(render_step_size / step_size);

    // Initialize simulation frame counter
    int step_number = 0;
    int render_frame = 0;

    // Create the sensor manager and a camera
    auto manager = chrono_types::make_shared<ChSensorManager>(my_hmmwv.GetSystem());

    // set lights
    manager->scene->AddPointLight({100, 100, 100}, {1, 1, 1}, 2000);
    manager->scene->AddPointLight({-100, 100, 100}, {1, 1, 1}, 2000);

    // set environment map
    // manager->scene->GetBackground().has_texture = true;
    // manager->scene->GetBackground().env_tex = "sensor/textures/cloud_layers_8k.hdr";
    // manager->scene->GetBackground().has_changed = true;

    // set up camera
    chrono::ChFrame<double> offset_pose1({-8, 0, 3}, Q_from_AngAxis(.2, {0, 1, 0}));
    auto cam = chrono_types::make_shared<ChCameraSensor>(my_hmmwv.GetChassisBody(),  // body camera is attached to
                                                         update_rate,                // update rate in Hz
                                                         offset_pose1,               // offset pose
                                                         image_width,                // image width
                                                         image_height,               // image height
                                                         fov  // camera's horizontal field of view
    );
    cam->SetName("Camera Sensor");
    cam->SetLag(lag);
    cam->SetCollectionWindow(exposure_time);

    // Renders the image at current point in the filter graph
    if (vis)
        cam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(int(image_width * 3 / 4),
                                                                     int(image_height * 3 / 4), "SCM Camera"));

    // Provides the host access to this RGBA8 buffer
    cam->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());

    if (save)
        // Save the current image to a png file at the specified path
        cam->PushFilter(chrono_types::make_shared<ChFilterSave>(sens_dir + "/cam/"));

    // Add sensor to the manager
    manager->AddSensor(cam);

    auto cam2 = chrono_types::make_shared<ChCameraSensor>(my_hmmwv.GetChassisBody(),  // body camera is attached to
                                                          update_rate,                // update rate in Hz
                                                          offset_pose1,               // offset pose
                                                          image_width,                // image width
                                                          image_height,               // image height
                                                          fov  // camera's horizontal field of view
    );
    cam2->SetName("Camera Sensor");
    cam2->SetLag(lag);
    cam2->SetCollectionWindow(exposure_time);
    if (vis)
        cam2->PushFilter(chrono_types::make_shared<ChFilterVisualize>(int(image_width * 3 / 4),
                                                                      int(image_height * 3 / 4), "SCM Camera"));
    manager->AddSensor(cam2);

    while (app.GetDevice()->run()) {
        double time = system->GetChTime();

        // End simulation if end time exceeded
        if (time > end_time)
            break;

        // Render scene

        if (img_output && step_number % render_steps == 0) {
            char filename[100];
            sprintf(filename, "%s/img_%03d.jpg", img_dir.c_str(), render_frame + 1);
            app.WriteImageToFile(filename);
            render_frame++;
        }

        if (step_number % render_steps == 0) {
            app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
            app.DrawAll();
            ChIrrTools::drawColorbar(0, 0.1, "Sinkage", app.GetDevice(), 30);
            app.EndScene();
        }

        // Driver inputs
        ChDriver::Inputs driver_inputs = driver.GetInputs();

        // Update modules
        driver.Synchronize(time);
        terrain.Synchronize(time);
        my_hmmwv.Synchronize(time, driver_inputs, terrain);
        app.Synchronize("", driver_inputs);

        manager->Update();

        // Advance dynamics
        system->DoStepDynamics(step_size);
        app.Advance(step_size);

        // Increment frame number
        step_number++;
    }

    return 0;
}
