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

#include <chrono>

#include "chrono_models/robot/curiosity/Curiosity.h"

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChMassProperties.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/input_output/ChWriterCSV.h"
#include "chrono/utils/ChUtilsCreators.h"

#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/terrain/SCMTerrain.h"

#include "chrono_thirdparty/filesystem/path.h"

#ifdef CHRONO_POSTPROCESS
    #include "chrono_postprocess/ChGnuPlot.h"
#endif

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
using namespace chrono::curiosity;

// -----------------------------------------------------------------------------

// Run-time visualization system (IRRLICHT or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// SCM grid spacing
double mesh_resolution = 0.02;

// Enable/disable bulldozing effects
bool enable_bulldozing = true;

// Enable/disable active domainfeature
bool enable_active_domains = true;

// Specify rover chassis type (Scarecrow or FullRover)
CuriosityChassisType chassis_type = CuriosityChassisType::FullRover;

// Specify rover wheel type (RealWheel, SimpleWheel, or CylWheel)
CuriosityWheelType wheel_type = CuriosityWheelType::RealWheel;

// Simulation time step
double time_step = 5e-4;

// Rendering frequency
double render_fps = 120;

// Enable/disable output
bool output = false;

// Enable/disable run-time visualization snapshots
bool snapshots = false;

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Global parameter for moving patch size:
    double wheel_range = 0.5;

    // Create the Chrono system and the associated collision system
    ChSystemSMC sys;
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // Create the Curiosity rover
    Curiosity rover(&sys, chassis_type, wheel_type);
    rover.SetDriver(chrono_types::make_shared<CuriositySpeedDriver>(1.0, CH_PI));
    rover.Initialize(ChFrame<>(ChVector3d(-5, -0.2, 0), QuatFromAngleX(-CH_PI / 2)));

    // Create obstacles
    std::vector<std::shared_ptr<ChBodyAuxRef>> rock;
    std::vector<std::string> rock_meshfile = {
        "robot/curiosity/rocks/rock1.obj", "robot/curiosity/rocks/rock1.obj",  //
        "robot/curiosity/rocks/rock1.obj", "robot/curiosity/rocks/rock1.obj",  //
        "robot/curiosity/rocks/rock3.obj", "robot/curiosity/rocks/rock3.obj"   //
    };
    std::vector<ChVector3d> rock_pos = {
        ChVector3d(-2.5, -0.3, -1.0), ChVector3d(-2.5, -0.3, +1.0),  //
        ChVector3d(-1.0, -0.3, -1.0), ChVector3d(-1.0, -0.3, +1.0),  //
        ChVector3d(+0.5, -0.3, -1.0), ChVector3d(+0.5, -0.3, +1.0)   //
    };
    std::vector<double> rock_scale = {
        0.8,  0.8,   //
        0.45, 0.45,  //
        0.45, 0.45   //
    };
    double rock_density = 8000;
    int num_rocks = 6;
    std::shared_ptr<ChContactMaterial> rock_mat = ChContactMaterial::DefaultMaterial(sys.GetContactMethod());

    for (int i = 0; i < num_rocks; i++) {
        auto mesh = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile(rock_meshfile[i]), false, true);
        mesh->Transform(ChVector3d(0, 0, 0), ChMatrix33<>(rock_scale[i]));

        double mass;
        ChVector3d cog;
        ChMatrix33<> inertia;
        mesh->ComputeMassProperties(true, mass, cog, inertia);
        ChMatrix33<> principal_inertia_rot;
        ChVector3d principal_I;
        ChInertiaUtils::PrincipalInertia(inertia, principal_I, principal_inertia_rot);

        auto body = chrono_types::make_shared<ChBodyAuxRef>();
        sys.Add(body);
        body->SetFixed(false);
        body->SetFrameRefToAbs(ChFrame<>(ChVector3d(rock_pos[i]), QUNIT));
        body->SetFrameCOMToRef(ChFrame<>(cog, principal_inertia_rot));
        body->SetMass(mass * rock_density);
        body->SetInertiaXX(rock_density * principal_I);

        auto ct_shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(rock_mat, mesh, false, false, 0.005);
        body->AddCollisionShape(ct_shape);
        body->EnableCollision(true);

        auto vis_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        vis_shape->SetMesh(mesh);
        vis_shape->SetBackfaceCull(true);
        body->AddVisualShape(vis_shape);

        rock.push_back(body);
    }

    // Create the SCM deformable terrain
    vehicle::SCMTerrain terrain(&sys);

    // Displace/rotate the terrain reference frame.
    // Note that SCMTerrain uses a default ISO reference frame (Z up). Since the mechanism is modeled here in
    // a Y-up global frame, we rotate the terrain frame by -90 degrees about the X axis.
    terrain.SetReferenceFrame(ChCoordsys<>(ChVector3d(0, -0.5, 0), QuatFromAngleX(-CH_PI_2)));

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

    // Enable active domains (for SCM efficiency)
    if (enable_active_domains) {
        // add active domain for each rover wheel
        for (const auto& wheel : rover.GetWheels())
            terrain.AddActiveDomain(wheel->GetBody(), VNULL, ChVector3d(0.5, 2 * wheel_range, 2 * wheel_range));

        // add active domain for each obstacles
        for (int i = 0; i < num_rocks; i++)
            terrain.AddActiveDomain(rock[i], VNULL, ChVector3d(2.0, 2.0, 2.0));
    }

    // Set some visualization parameters
    terrain.SetColormap(ChColormap::Type::COPPER);
    ////terrain.SetPlotType(vehicle::SCMTerrain::PLOT_PRESSURE, 0, 20000);
    terrain.SetPlotType(vehicle::SCMTerrain::PLOT_SINKAGE, -0.01, 0.04);
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
            vis_irr->SetWindowSize(1280, 720);
            vis_irr->SetWindowTitle("Curiosity Obstacle Crossing on SCM");
            vis_irr->Initialize();
            vis_irr->AddLogo();
            vis_irr->AddSkyBox();
            vis_irr->AddCamera(ChVector3d(-1.0, 1.0, 3.0), ChVector3d(-5.0, 0.0, 0.0));
            vis_irr->AddTypicalLights();
            vis_irr->AddLightWithShadow(ChVector3d(-5.0, 8.0, -0.5), ChVector3d(-1, 0, 0), 100, 1, 35, 85, 512,
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
            vis_vsg->SetWindowSize(1152, 648);
            vis_vsg->SetWindowTitle("Curiosity Obstacle Crossing on SCM");
            vis_vsg->SetBackgroundColor(ChColor(0.2f, 0.2f, 0.2f));
            vis_vsg->SetLightIntensity(1.0f);
            vis_vsg->SetLightDirection(-1.5 * CH_PI_2, CH_PI_4);
            vis_vsg->EnableShadows();
            vis_vsg->SetCameraVertical(CameraVerticalDir::Y);
            vis_vsg->AddCamera(ChVector3d(0.0, 1.5, -4.0), ChVector3d(-5.0, 0.0, 0.0));
            vis_vsg->Initialize();

            vis = vis_vsg;
#endif
            break;
        }
    }

    // Initialize output
    const std::string out_dir = GetChronoOutputPath() + "CURIOSITY_SCM";
    if (output || snapshots) {
        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            std::cout << "Error creating directory " << out_dir << std::endl;
            return 1;
        }
        if (snapshots) {
            if (!filesystem::create_directory(filesystem::path(out_dir + "/snapshots"))) {
                std::cerr << "Error creating directory " << out_dir + "/snapshots" << std::endl;
                return 1;
            }
        }
    }

    ChWriterCSV csv(" ");

    // Simulation loop
    double time = 0;
    double time_end = 14;
    int render_frame = 0;
    
    while (vis->Run()) {
        time = sys.GetChTime();
        if (time >= time_end)
            break;

        if (time >= render_frame / render_fps) {
            vis->BeginScene();
            vis->SetCameraTarget(rover.GetChassisPos());
            vis->Render();
            vis->EndScene();

            if (snapshots) {
                std::ostringstream filename;
                filename << out_dir << "/snapshots/" << std::setw(5) << std::setfill('0') << render_frame << ".jpg";
                vis->WriteImageToFile(filename.str());
            }

            render_frame++;
        }

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
        csv.WriteToFile(out_dir + "/output.dat");
    }

    return 0;
}
