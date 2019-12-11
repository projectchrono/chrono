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
// Chrono::Vehicle + ChronoParallel demo program for simulating a HMMWV vehicle
// over rigid or granular material.
//
// Contact uses the SMC (penalty) formulation.
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
#include "chrono_vehicle/terrain/SCMDeformableTerrain.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h"

#include "chrono_models/vehicle/hmmwv/HMMWV.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::collision;
using namespace chrono::irrlicht;
using namespace chrono::vehicle;
using namespace chrono::vehicle::hmmwv;

using std::cout;
using std::endl;

// =============================================================================
// USER SETTINGS
// =============================================================================

// -----------------------------------------------------------------------------
// Terrain parameters
// -----------------------------------------------------------------------------

// Dimensions
double terrainHeight = 0;
double terrainLength = 16.0;  // size in X direction
double terrainWidth = 8.0;    // size in Y direction

// Divisions (X and Y)
int divLength = 128;//1024;
int divWidth = 64;//512;

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
VisualizationType chassis_vis = VisualizationType::NONE;

// Initial vehicle position and orientation
ChVector<> initLoc(-5, -2, 0.6);
ChQuaternion<> initRot(1, 0, 0, 0);

// Contact material properties
float Y_t = 1.0e6f;
float cr_t = 0.1f;
float mu_t = 0.8f;

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

// Visualization output
bool img_output = false;

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
            m_steering = 0.6 * std::sin(CH_C_2PI * (eff_time - 2) / 6);
    }

  private:
    double m_delay;
};

// =============================================================================

void CreateLuggedGeometry(std::shared_ptr<ChBody> wheelBody) {
    std::string lugged_file("hmmwv/lugged_wheel_section.obj");
    geometry::ChTriangleMeshConnected lugged_mesh;
    ChConvexDecompositionHACDv2 lugged_convex;
    utils::LoadConvexMesh(vehicle::GetDataFile(lugged_file), lugged_mesh, lugged_convex);
    int num_hulls = lugged_convex.GetHullCount();

    auto coll_model = wheelBody->GetCollisionModel();
    coll_model->ClearModel();

    // Assemble the tire contact from 15 segments, properly offset.
    // Each segment is further decomposed in convex hulls.
    for (int iseg = 0; iseg < 15; iseg++) {
        ChQuaternion<> rot = Q_from_AngAxis(iseg * 24 * CH_C_DEG_TO_RAD, VECT_Y);
        for (int ihull = 0; ihull < num_hulls; ihull++) {
            std::vector<ChVector<> > convexhull;
            lugged_convex.GetConvexHullResult(ihull, convexhull);
            coll_model->AddConvexHull(convexhull, VNULL, rot);
        }
    }

    // Add a cylinder to represent the wheel hub.
    coll_model->AddCylinder(0.223, 0.223, 0.126);
    coll_model->BuildModel();

    // Visualization
    auto trimesh = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
    trimesh->LoadWavefrontMesh(vehicle::GetDataFile("hmmwv/lugged_wheel.obj"), false, false);

    auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
    trimesh_shape->SetMesh(trimesh);
    trimesh_shape->SetName("lugged_wheel");
    wheelBody->AddAsset(trimesh_shape);

    auto mcolor = chrono_types::make_shared<ChColorAsset>(0.3f, 0.3f, 0.3f);
    wheelBody->AddAsset(mcolor);
}

// =============================================================================

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // --------------------
    // Create HMMWV vehicle
    // --------------------
    HMMWV_Full my_hmmwv;
    my_hmmwv.SetContactMethod(ChMaterialSurface::SMC);
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
    for (auto& axle : my_hmmwv.GetVehicle().GetAxles()) {
        auto wheelBodyL = axle->m_wheels[0]->GetSpindle();
        wheelBodyL->GetMaterialSurfaceSMC()->SetFriction(mu_t);
        wheelBodyL->GetMaterialSurfaceSMC()->SetYoungModulus(Y_t);
        wheelBodyL->GetMaterialSurfaceSMC()->SetRestitution(cr_t);
        CreateLuggedGeometry(wheelBodyL);
        auto wheelBodyR = axle->m_wheels[1]->GetSpindle();
        wheelBodyR->GetMaterialSurfaceSMC()->SetFriction(mu_t);
        wheelBodyR->GetMaterialSurfaceSMC()->SetYoungModulus(Y_t);
        wheelBodyR->GetMaterialSurfaceSMC()->SetRestitution(cr_t);
        CreateLuggedGeometry(wheelBodyR);
    }

    // --------------------
    // Create driver system
    // --------------------
    MyDriver driver(my_hmmwv.GetVehicle(), 0.5);
    driver.Initialize();

    // ------------------
    // Create the terrain
    // ------------------
    ChTerrain* terrain;

    switch (terrain_type) {
        case DEFORMABLE_SOIL: {
            SCMDeformableTerrain* terrainD = new SCMDeformableTerrain(system);
            terrainD->SetPlane(ChCoordsys<>(VNULL, Q_from_AngX(CH_C_PI_2)));
            terrainD->SetSoilParametersSCM(2e6,   // Bekker Kphi
                                           0,     // Bekker Kc
                                           1.1,   // Bekker n exponent
                                           0,     // Mohr cohesive limit (Pa)
                                           30,    // Mohr friction limit (degrees)
                                           0.01,  // Janosi shear coefficient (m)
                                           2e8,   // Elastic stiffness (Pa/m), before plastic yield
                                           3e4    // Damping (Pa s/m), proportional to negative vertical speed (optional)
                                           );
            /*
            terrainD->SetBulldozingFlow(true);    // inflate soil at the border of the rut
            terrainD->SetBulldozingParameters(55, // angle of friction for erosion of displaced material at the border of the rut
                                            0.8, // displaced material vs downward pressed material.
                                            5,   // number of erosion refinements per timestep
                                            10); // number of concentric vertex selections subject to erosion
            */
            // Turn on the automatic level of detail refinement, so a coarse terrain mesh
            // is automatically improved by adding more points under the wheel contact patch:
            terrainD->SetAutomaticRefinement(true);
            terrainD->SetAutomaticRefinementResolution(0.04);

            ////terrainD->SetTexture(vehicle::GetDataFile("terrain/textures/grass.jpg"), 80, 16);
            ////terrainD->SetPlotType(vehicle::SCMDeformableTerrain::PLOT_PRESSURE_YELD, 0, 30000.2);
            terrainD->SetPlotType(vehicle::SCMDeformableTerrain::PLOT_SINKAGE, 0, 0.1);

            terrainD->Initialize(terrainHeight, terrainLength, terrainWidth, divLength, divWidth);

            terrain = terrainD;

            break;
        }

        case RIGID_SOIL: {
            RigidTerrain* terrainR = new RigidTerrain(system);
            auto patch = terrainR->AddPatch(ChCoordsys<>(ChVector<>(0, 0, terrainHeight - 5), QUNIT),
                                            ChVector<>(terrainLength, terrainWidth, 10));
            patch->SetContactFrictionCoefficient(0.9f);
            patch->SetContactRestitutionCoefficient(0.01f);
            patch->SetContactMaterialProperties(2e7f, 0.3f);
            patch->SetColor(ChColor(0.8f, 0.8f, 0.5f));
            patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 200);
            terrainR->Initialize();

            terrain = terrainR;

            break;
        }
    }

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

    while (app.GetDevice()->run()) {
        double time = system->GetChTime();

        // Render scene
        app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
        app.DrawAll();
        ChIrrTools::drawColorbar(0, 0.1, "Sinkage", app.GetDevice(), 30);

        if (img_output && step_number % render_steps == 0) {
            char filename[100];
            sprintf(filename, "%s/img_%03d.jpg", img_dir.c_str(), render_frame + 1);
            app.WriteImageToFile(filename);
            render_frame++;
        }

        // Driver inputs
        ChDriver::Inputs driver_inputs = driver.GetInputs();

        // Update modules
        driver.Synchronize(time);
        terrain->Synchronize(time);
        my_hmmwv.Synchronize(time, driver_inputs, *terrain);
        app.Synchronize("", driver_inputs);

        // Advance dynamics
        system->DoStepDynamics(step_size);
        app.Advance(step_size);

        // Increment frame number
        step_number++;

        app.EndScene();
    }

    // Cleanup
    delete terrain;

    return 0;
}
