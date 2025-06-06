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
// Chrono::Vehicle demo program for simulating a HMMWV vehicle on SCM deformable
// terrain.
//
// The global reference frame has Z up.
// All units SI.
// =============================================================================

#include <cstdio>
#include <cmath>
#include <vector>

#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/ChDriver.h"
#include "chrono_vehicle/terrain/SCMTerrain.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"

#include "chrono_models/vehicle/hmmwv/HMMWV.h"

#include "chrono_thirdparty/filesystem/path.h"

#include "chrono_sensor/sensors/ChCameraSensor.h"
#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"

using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;
using namespace chrono::sensor;

using std::cout;
using std::endl;

// =============================================================================
// USER SETTINGS
// =============================================================================

// -----------------------------------------------------------------------------
// Terrain parameters
// -----------------------------------------------------------------------------

double terrainLength = 16.0;  // size in X direction
double terrainWidth = 8.0;    // size in Y direction
double delta = 0.05;          // SCM grid spacing

// -----------------------------------------------------------------------------
// Vehicle parameters
// -----------------------------------------------------------------------------

// Type of tire (controls both contact and visualization)
enum class TireType { CYLINDRICAL, LUGGED };
TireType tire_type = TireType::LUGGED;

// Tire contact material properties
float Y_t = 1.0e6f;
float cr_t = 0.1f;
float mu_t = 0.8f;

// Initial vehicle position and orientation
ChVector3d initLoc(-5, -2, 0.6);
ChQuaternion<> initRot(1, 0, 0, 0);

// -----------------------------------------------------------------------------
// Camera parameters
// -----------------------------------------------------------------------------

// Update Rate in Hz
float update_rate = 60.0f;

// Image width and height
unsigned int image_width = 1920;
unsigned int image_height = 1080;

// Camera's horizontal field of view
float fov = (float)(CH_PI_3);

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
ChVector3d trackPoint(0.0, 0.0, 1.75);

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
bool visualize = true;

// =============================================================================

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
            m_steering = 0.6 * std::sin(CH_2PI * (eff_time - 2) / 6);
    }

  private:
    double m_delay;
};

// =============================================================================

void CreateLuggedGeometry(std::shared_ptr<ChBody> wheel_body, std::shared_ptr<ChContactMaterialSMC> wheel_material) {
    std::string lugged_file("hmmwv/lugged_wheel_section.obj");
    ChTriangleMeshConnected lugged_mesh;
    ChConvexDecompositionHACDv2 lugged_convex;
    utils::LoadConvexMesh(vehicle::GetDataFile(lugged_file), lugged_mesh, lugged_convex);
    int num_hulls = lugged_convex.GetHullCount();

    // Assemble the tire contact from 15 segments, properly offset.
    // Each segment is further decomposed in convex hulls.
    for (int iseg = 0; iseg < 15; iseg++) {
        ChQuaternion<> rot = QuatFromAngleAxis(iseg * 24 * CH_DEG_TO_RAD, VECT_Y);
        for (int ihull = 0; ihull < num_hulls; ihull++) {
            std::vector<ChVector3d> convexhull;
            lugged_convex.GetConvexHullResult(ihull, convexhull);
            auto ct_shape = chrono_types::make_shared<ChCollisionShapeConvexHull>(wheel_material, convexhull);
            wheel_body->AddCollisionShape(ct_shape, ChFrame<>(VNULL, rot));
        }
    }

    // Add a cylinder to represent the wheel hub.
    auto cyl_shape = chrono_types::make_shared<ChCollisionShapeCylinder>(wheel_material, 0.223, 0.252);
    wheel_body->AddCollisionShape(cyl_shape, ChFrame<>(VNULL, QuatFromAngleX(CH_PI_2)));

    // Visualization
    auto trimesh =
        ChTriangleMeshConnected::CreateFromWavefrontFile(vehicle::GetDataFile("hmmwv/lugged_wheel.obj"), false, false);

    auto trimesh_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    trimesh_shape->SetMesh(trimesh);
    trimesh_shape->SetName("lugged_wheel");
    trimesh_shape->SetMutable(false);
    wheel_body->AddVisualShape(trimesh_shape, ChFrame<>());

    auto vis_mat = chrono_types::make_shared<ChVisualMaterial>();
    vis_mat->SetDiffuseColor({0.3f, 0.3f, 0.3f});
    vis_mat->SetSpecularColor({0.1f, 0.1f, 0.1f});
    trimesh_shape->AddMaterial(vis_mat);
}

// =============================================================================

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // --------------------
    // Create HMMWV vehicle
    // --------------------
    HMMWV_Full my_hmmwv;
    my_hmmwv.SetContactMethod(ChContactMethod::SMC);
    my_hmmwv.SetChassisFixed(false);
    my_hmmwv.SetInitPosition(ChCoordsys<>(initLoc, initRot));
    my_hmmwv.SetEngineType(EngineModelType::SHAFTS);
    my_hmmwv.SetTransmissionType(TransmissionModelType::AUTOMATIC_SHAFTS);
    my_hmmwv.SetDriveType(DrivelineTypeWV::AWD);
    switch (tire_type) {
        case TireType::CYLINDRICAL:
            my_hmmwv.SetTireType(TireModelType::RIGID_MESH);
            break;
        case TireType::LUGGED:
            my_hmmwv.SetTireType(TireModelType::RIGID);
            break;
    }

    my_hmmwv.Initialize();

    my_hmmwv.SetChassisVisualizationType(VisualizationType::NONE);

    // -----------------------------------------------------------
    // Set tire contact material, contact model, and visualization
    // -----------------------------------------------------------
    auto wheel_material = chrono_types::make_shared<ChContactMaterialSMC>();
    wheel_material->SetFriction(mu_t);
    wheel_material->SetYoungModulus(Y_t);
    wheel_material->SetRestitution(cr_t);

    switch (tire_type) {
        case TireType::CYLINDRICAL:
            my_hmmwv.SetTireVisualizationType(VisualizationType::MESH);
            break;
        case TireType::LUGGED:
            my_hmmwv.SetTireVisualizationType(VisualizationType::NONE);
            for (auto& axle : my_hmmwv.GetVehicle().GetAxles()) {
                CreateLuggedGeometry(axle->m_wheels[0]->GetSpindle(), wheel_material);
                CreateLuggedGeometry(axle->m_wheels[1]->GetSpindle(), wheel_material);
            }
    }

    // --------------------
    // Create driver system
    // --------------------
    MyDriver driver(my_hmmwv.GetVehicle(), 0.5);
    driver.Initialize();

    // ------------------
    // Create the terrain
    // ------------------
    ChSystem* system = my_hmmwv.GetSystem();

    SCMTerrain terrain(system);
    terrain.SetSoilParameters(2e6,   // Bekker Kphi
                              0,     // Bekker Kc
                              1.1,   // Bekker n exponent
                              0,     // Mohr cohesive limit (Pa)
                              30,    // Mohr friction limit (degrees)
                              0.01,  // Janosi shear coefficient (m)
                              2e8,   // Elastic stiffness (Pa/m), before plastic yield
                              3e4    // Damping (Pa s/m), proportional to negative vertical speed (optional)
    );
    terrain.GetMesh()->SetWireframe(false);

    ////terrain.SetBulldozingFlow(true);      // inflate soil at the border of the rut
    ////terrain.SetBulldozingParameters(55,   // angle of friction for erosion of displaced material at rut border
    ////                                0.8,  // displaced material vs downward pressed material.
    ////                                5,    // number of erosion refinements per timestep
    ////                                10);  // number of concentric vertex selections subject to erosion

    // Optionally, enable active domains feature (single domain around vehicle chassis)
    terrain.AddActiveDomain(my_hmmwv.GetChassisBody(), ChVector3d(0, 0, 0), ChVector3d(5, 3, 1));

    // Optionally, enable active domains feature (multiple domains around each wheel)
    ////for (auto& axle : my_hmmwv.GetVehicle().GetAxles()) {
    ////    terrain.AddActiveDomain(axle->m_wheels[0]->GetSpindle(), ChVector3d(0, 0, 0), ChVector3d(1, 0.5, 1));
    ////    terrain.AddActiveDomain(axle->m_wheels[1]->GetSpindle(), ChVector3d(0, 0, 0), ChVector3d(1, 0.5, 1));
    ////}

    ////terrain.SetTexture(vehicle::GetDataFile("terrain/textures/grass.jpg"), 80, 16);
    ////terrain.SetPlotType(vehicle::SCMTerrain::PLOT_PRESSURE_YELD, 0, 30000.2);
    terrain.SetPlotType(vehicle::SCMTerrain::PLOT_SINKAGE, 0, 0.1);

    terrain.Initialize(terrainLength, terrainWidth, delta);

    auto vis_mat = chrono_types::make_shared<ChVisualMaterial>();
    vis_mat->SetSpecularColor({.1f, .1f, .1f});
    vis_mat->SetRoughness(1);
    vis_mat->SetKdTexture(GetChronoDataFile("sensor/textures/grass_texture.jpg"));
    terrain.GetMesh()->AddMaterial(vis_mat);

    // ---------------------------------------
    // Create the vehicle Irrlicht application
    // ---------------------------------------

    auto vis = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
    vis->SetWindowTitle("HMMWV Deformable Soil Demo");
    vis->SetChaseCamera(trackPoint, 6.0, 0.5);
    vis->Initialize();
    vis->AddTypicalLights();
    vis->AddSkyBox();
    vis->AddLogo();
    vis->AttachVehicle(&my_hmmwv.GetVehicle());
    vis->AddGuiColorbar("Sinkage", {0, 0.1}, ChColormap::Type::JET, false);

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

    // Create the sensor manager
    auto manager = chrono_types::make_shared<ChSensorManager>(my_hmmwv.GetSystem());

    // Set lights
    float intensity = 1.0;
    manager->scene->AddPointLight({20, 20, 30}, {intensity, intensity, intensity}, 5000);

    // Set up Camera
    chrono::ChFrame<double> offset_pose1({-8, 0, 3}, QuatFromAngleAxis(.2, {0, 1, 0}));
    auto cam =
        chrono_types::make_shared<ChCameraSensor>(my_hmmwv.GetChassisBody(), update_rate, offset_pose1, image_width,
                                                  image_height, fov, 1, CameraLensModelType::PINHOLE, false);
    cam->SetName("Camera Sensor");
    cam->SetLag(lag);
    cam->SetCollectionWindow(exposure_time);

    // Renders the image at current point in the filter graph
    if (visualize)
        cam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(int(image_width * 3 / 4),
                                                                     int(image_height * 3 / 4), "SCM Camera"));

    // Provides the host access to this RGBA8_buffer
    // cam->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());

    // if (save)
    //     cam->PushFilter(chrono_types::make_shared<ChFilterSave>(sens_dir + "/cam/"));

    // Add sensor to the manager
    manager->AddSensor(cam);

    // ---------------
    // Simulation loop
    // ---------------
    std::cout << "Vehicle mass: " << my_hmmwv.GetVehicle().GetMass() << std::endl;

    // Solver settings
    system->SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
    system->GetSolver()->AsIterative()->SetMaxIterations(50);

    // Number of simulation steps between two 3D view render frames
    int render_steps = (int)std::ceil(render_step_size / step_size);

    // Initialize simulation frame counter
    int step_number = 0;
    int render_frame = 0;

    ChTimer timer;

    while (vis->Run()) {
        double time = system->GetChTime();

        if (step_number == 800) {
            std::cout << "\nstart timer at t = " << time << std::endl;
            timer.start();
        }
        if (step_number == 1400) {
            timer.stop();
            std::cout << "stop timer at t = " << time << std::endl;
            std::cout << "elapsed: " << timer() << std::endl;
            std::cout << "\nSCM stats for last step:" << std::endl;
            terrain.PrintStepStatistics(std::cout);
        }

        // Render scene
        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        if (img_output && step_number % render_steps == 0) {
            char filename[100];
            sprintf(filename, "%s/img_%03d.jpg", img_dir.c_str(), render_frame + 1);
            vis->WriteImageToFile(filename);
            render_frame++;
        }

        // Driver inputs
        DriverInputs driver_inputs = driver.GetInputs();

        // Update modules
        driver.Synchronize(time);
        terrain.Synchronize(time);
        my_hmmwv.Synchronize(time, driver_inputs, terrain);
        vis->Synchronize(time, driver_inputs);

        manager->Update();

        // Advance dynamics
        system->DoStepDynamics(step_size);
        vis->Advance(step_size);

        // Increment frame number
        step_number++;
    }

    return 0;
}
