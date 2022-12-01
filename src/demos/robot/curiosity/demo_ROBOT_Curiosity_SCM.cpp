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
// Authors: Jason Zhou, Radu Serban
// =============================================================================
//
// Demo to show Curiosity Rovering across obstacles with symmetric arrangement
// on SCM terrain
//
// =============================================================================

#include "chrono_models/robot/curiosity/Curiosity.h"

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChInertiaUtils.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/utils/ChUtilsCreators.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/SCMDeformableTerrain.h"

#include "chrono_thirdparty/filesystem/path.h"

#include <chrono>

using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::geometry;
using namespace chrono::curiosity;

// SCM grid spacing
double mesh_resolution = 0.02;

// Enable/disable bulldozing effects
bool enable_bulldozing = true;

// Enable/disable moving patch feature
bool enable_moving_patch = true;

// Specify rover chassis type (Scarecrow or FullRover)
CuriosityChassisType chassis_type = CuriosityChassisType::FullRover;

// Specify rover wheel type (RealWheel, SimpleWheel, or CylWheel)
CuriosityWheelType wheel_type = CuriosityWheelType::RealWheel;

// Simulation time step
double time_step = 5e-4;

// Enable/disable output
bool output = false;
const std::string out_dir = GetChronoOutputPath() + "CURIOSITY_SCM";

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Global parameter for moving patch size:
    double wheel_range = 0.5;

    // Create the Chrono system and Curiosity rover
    ChSystemSMC sys;
    Curiosity rover(&sys, chassis_type, wheel_type);
    rover.SetDriver(chrono_types::make_shared<CuriositySpeedDriver>(1.0, CH_C_PI));
    rover.Initialize(ChFrame<>(ChVector<>(-5, -0.2, 0), Q_from_AngX(-CH_C_PI / 2)));

    // Create obstacles
    std::vector<std::shared_ptr<ChBodyAuxRef>> rock;
    std::vector<std::string> rock_meshfile = {
        "robot/curiosity/rocks/rock1.obj", "robot/curiosity/rocks/rock1.obj",  //
        "robot/curiosity/rocks/rock1.obj", "robot/curiosity/rocks/rock1.obj",  //
        "robot/curiosity/rocks/rock3.obj", "robot/curiosity/rocks/rock3.obj"   //
    };
    std::vector<ChVector<>> rock_pos = {
        ChVector<>(-2.5, -0.3, -1.0), ChVector<>(-2.5, -0.3, +1.0), //
        ChVector<>(-1.0, -0.3, -1.0), ChVector<>(-1.0, -0.3, +1.0), //
        ChVector<>(+0.5, -0.3, -1.0), ChVector<>(+0.5, -0.3, +1.0) //
    };
    std::vector<double> rock_scale = {
        0.8,  0.8,   //
        0.45, 0.45,  //
        0.45, 0.45   //
    };
    double rock_density = 8000;
    std::shared_ptr<ChMaterialSurface> rock_mat = ChMaterialSurface::DefaultMaterial(sys.GetContactMethod());

    for (int i = 0; i < 6; i++) {
        auto mesh = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile(rock_meshfile[i]), false, true);
        mesh->Transform(ChVector<>(0, 0, 0), ChMatrix33<>(rock_scale[i]));

        double mass;
        ChVector<> cog;
        ChMatrix33<> inertia;
        mesh->ComputeMassProperties(true, mass, cog, inertia);
        ChMatrix33<> principal_inertia_rot;
        ChVector<> principal_I;
        ChInertiaUtils::PrincipalInertia(inertia, principal_I, principal_inertia_rot);

        auto body = chrono_types::make_shared<ChBodyAuxRef>();
        sys.Add(body);
        body->SetBodyFixed(false);
        body->SetFrame_REF_to_abs(ChFrame<>(ChVector<>(rock_pos[i]), QUNIT));
        body->SetFrame_COG_to_REF(ChFrame<>(cog, principal_inertia_rot));
        body->SetMass(mass * rock_density);
        body->SetInertiaXX(rock_density * principal_I);

        body->GetCollisionModel()->ClearModel();
        body->GetCollisionModel()->AddTriangleMesh(rock_mat, mesh, false, false, VNULL, ChMatrix33<>(1),
                                                   0.005);
        body->GetCollisionModel()->BuildModel();
        body->SetCollide(true);

        auto mesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
        mesh_shape->SetMesh(mesh);
        mesh_shape->SetBackfaceCull(true);
        body->AddVisualShape(mesh_shape);

        rock.push_back(body);
    }

    // Create the SCM deformable terrain
    vehicle::SCMDeformableTerrain terrain(&sys);

    // Displace/rotate the terrain reference plane.
    // Note that SCMDeformableTerrain uses a default ISO reference frame (Z up). Since the mechanism is modeled here in
    // a Y-up global frame, we rotate the terrain plane by -90 degrees about the X axis.
    terrain.SetPlane(ChCoordsys<>(ChVector<>(0, -0.5, 0), Q_from_AngX(-CH_C_PI_2)));

    // Use a regular grid
    double length = 14;
    double width = 4;
    terrain.Initialize(length, width, mesh_resolution);

    // Set the soil terramechanical parameters
    terrain.SetSoilParameters(0.82e6,   // Bekker Kphi
                              0.14e4,   // Bekker Kc
                              1.0,      // Bekker n exponent
                              0.017e4,  // Mohr cohesive limit (Pa)
                              35,       // Mohr friction limit (degrees)
                              1.78e-2,  // Janosi shear coefficient (m)
                              2e8,      // Elastic stiffness (Pa/m), before plastic yield, must be > Kphi
                              3e4       // Damping (Pa s/m), proportional to negative vertical speed (optional)
    );

    // Set up bulldozing factors
    terrain.EnableBulldozing(enable_bulldozing);
    terrain.SetBulldozingParameters(55,  // angle of friction for erosion of displaced material at the border of the rut
                                    1,   // displaced material vs downward pressed material.
                                    5,   // number of erosion refinements per timestep
                                    6);  // number of concentric vertex selections subject to erosion

    // Enable moving patches (for SCM efficiency)
    if (enable_moving_patch) {
        // add moving patch for each rover wheel
        for (const auto& wheel : rover.GetWheels())
            terrain.AddMovingPatch(wheel->GetBody(), VNULL, ChVector<>(0.5, 2 * wheel_range, 2 * wheel_range));

        // add moving patch for each obstacles
        for (int i = 0; i < 6; i++)
            terrain.AddMovingPatch(rock[i], VNULL, ChVector<>(2.0, 2.0, 2.0));
    }

    // Set some visualization parameters
    terrain.SetPlotType(vehicle::SCMDeformableTerrain::PLOT_PRESSURE, 0, 20000);
    terrain.GetMesh()->SetWireframe(true);

    // Create the Irrlicht visualization sys
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->AttachSystem(&sys);
    vis->SetCameraVertical(CameraVerticalDir::Y);
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("Curiosity Obstacle Crossing on SCM");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector<>(-1.0, 1.0, 3.0), ChVector<>(-5.0, 0.0, 0.0));
    vis->AddTypicalLights();
    vis->AddLightWithShadow(ChVector<>(-5.0, 8.0, -0.5), ChVector<>(-1, 0, 0), 100, 1, 35, 85, 512,
                            ChColor(0.8f, 0.8f, 0.8f));
    vis->EnableShadows();

    // Initialize output
    if (output) {
        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            std::cout << "Error creating directory " << out_dir << std::endl;
            return 1;
        }
    }
    utils::CSV_writer csv(" ");

    // Simulation loop
    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        tools::drawColorbar(vis.get(), 0, 20000, "Pressure yield [Pa]", 1600);
        vis->EndScene();

        if (output) {
            // write drive torques of all six wheels into file
            csv << sys.GetChTime() << rover.GetWheelTracTorque(CuriosityWheelID::C_LF)
                << rover.GetWheelTracTorque(CuriosityWheelID::C_RF) << rover.GetWheelTracTorque(CuriosityWheelID::C_LM)
                << rover.GetWheelTracTorque(CuriosityWheelID::C_RM) << rover.GetWheelTracTorque(CuriosityWheelID::C_LB)
                << rover.GetWheelTracTorque(CuriosityWheelID::C_RB) << std::endl;
        }
        rover.Update();

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
