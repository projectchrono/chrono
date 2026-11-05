// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Chrono::DEM demo of the force network drawn by the Chrono::VSG plugin for DEM
// systems.
//
// Spheres fall into a box and are compressed by a heavy lid, which can move only
// vertically. The box walls are the walls of the DEM domain. The lid is a plane
// boundary condition whose vertical motion is integrated here, from the reaction
// force of the spheres on it. The plugin draws the contact force network (contacts
// with the walls and the lid included) and the spheres, and shows a panel with
// particle and contact counts. The spheres are hidden at start, since the network
// runs inside them; the panel has checkboxes for both.
//
// The material-based contact model records the forces of contacts with boundary
// conditions. With the MULTI_STEP friction model, the network can also be colored
// by the stored tangential displacement.
//
// The global reference frame has Z up.
// =============================================================================

#include <cmath>
#include <iomanip>
#include <random>
#include <sstream>

#include "chrono/assets/ChVisualShapeBox.h"
#include "chrono/core/ChDataPath.h"
#include "chrono/physics/ChBody.h"

#include "chrono_dem/physics/ChSystemDem.h"
#include "chrono_dem/visualization/ChDemVisualizationVSG.h"

#include "chrono_vsg/ChVisualSystemVSG.h"

using namespace chrono;
using namespace chrono::dem;
using namespace chrono::vsg3d;

// -----------------------------------------------------------------------------

// Quantity used to color the force network
auto coloring = ChDemVisualizationVSG::NetworkColoring::NORMAL_FORCE;

// Draw only a slab through the middle of the box (section cut), for both the network and the spheres
bool section_cut = false;

// Save images of the rendered frames
bool save_images = false;

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2026 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Problem parameters
    float radius = 0.005f;       // sphere radius
    float density = 2500;        // sphere density
    float width = 0.2f;          // inner width of the box (in x and y)
    float height = 0.2f;         // inner height of the box
    float thickness = 0.01f;     // thickness of the box walls and of the lid (visualization only)
    int num_layers = 10;         // number of layers of spheres
    double lid_pressure = 2000;  // pressure applied by the lid
    double gravity = 9.81;       // gravitational acceleration
    float time_step = 1e-4f;     // integration step size
    double time_end = 1;         // simulation length (the lid settles within this time)
    double render_fps = 50;      // rendering frequency (in simulation time)

    // DEM system, with the walls of the domain as the walls of the box (the domain is centered above the origin)
    ChSystemDem dem_sys(radius, density, ChVector3f(width, width, height), ChVector3f(0, 0, height / 2));
    dem_sys.SetGravitationalAcceleration(ChVector3f(0, 0, (float)-gravity));
    dem_sys.SetBDFixed(true);

    // Material-based contact model, with MULTI_STEP friction
    dem_sys.UseMaterialBasedModel(true);
    dem_sys.SetYoungModulus_SPH(1e7);
    dem_sys.SetYoungModulus_WALL(1e7);
    dem_sys.SetPoissonRatio_SPH(0.3);
    dem_sys.SetPoissonRatio_WALL(0.3);
    dem_sys.SetRestitution_SPH(0.3);
    dem_sys.SetRestitution_WALL(0.3);
    dem_sys.SetFrictionMode(CHDEM_FRICTION_MODE::MULTI_STEP);
    dem_sys.SetStaticFrictionCoeff_SPH2SPH(0.5f);
    dem_sys.SetStaticFrictionCoeff_SPH2WALL(0.5f);

    // The force network requires the recording of contact forces
    dem_sys.SetRecordingContactInfo(true);

    dem_sys.SetTimeIntegrator(CHDEM_TIME_INTEGRATOR::EXTENDED_TAYLOR);
    dem_sys.SetFixedStepSize(time_step);
    dem_sys.SetVerbosity(CHDEM_VERBOSITY::QUIET);

    // Spheres, on a grid with a random horizontal offset
    std::mt19937 generator(1);
    std::uniform_real_distribution<float> unif(-1, 1);
    float pitch = 2.5f * radius;
    int n = static_cast<int>(std::floor(width / pitch));
    std::vector<ChVector3f> positions;
    for (int k = 0; k < num_layers; k++) {
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                positions.push_back(ChVector3f(pitch * (i - 0.5f * (n - 1)) + 0.1f * radius * unif(generator),  //
                                               pitch * (j - 0.5f * (n - 1)) + 0.1f * radius * unif(generator),  //
                                               pitch * (k + 0.5f)));
            }
        }
    }
    dem_sys.SetParticles(positions);
    std::cout << "Number of spheres: " << positions.size() << std::endl;

    // Lid: a plane boundary condition above the spheres, facing down, with its reaction force tracked
    double lid_mass = lid_pressure * width * width / gravity;
    double lid_z0 = pitch * (num_layers + 1);
    size_t lid_id = dem_sys.CreateBCPlane(ChVector3f(0, 0, (float)lid_z0), ChVector3f(0, 0, -1), true);

    dem_sys.Initialize();

    // The plane follows the lid height through its offset function
    double lid_z = lid_z0;
    double lid_vel = 0;
    dem_sys.SetBCOffsetFunction(lid_id, [&lid_z, lid_z0](float t) {
        double3 offset = {0, 0, lid_z - lid_z0};
        return offset;
    });

    // DEM plugin for the VSG visual system
    auto dem_vis = chrono_types::make_shared<ChDemVisualizationVSG>(&dem_sys);
    dem_vis->SetNetworkVisibility(true);
    // The segments of a contact run inside the spheres, so the spheres are hidden (the plugin panel has a checkbox to
    // show them)
    dem_vis->EnableParticles(false);
    dem_vis->SetNetworkMode(ChForceNetworkVSG::Mode::CYLINDERS);
    switch (coloring) {
        case ChDemVisualizationVSG::NetworkColoring::NORMAL_FORCE: {
            // Range from the force of the lid on one sphere of the top layer
            double force = lid_mass * gravity / (n * n);
            dem_vis->SetNetworkColoring(coloring, 0, 3 * force);
            break;
        }
        case ChDemVisualizationVSG::NetworkColoring::COULOMB_RATIO:
            dem_vis->SetNetworkColoring(coloring, 0, 1);
            break;
        case ChDemVisualizationVSG::NetworkColoring::TANGENTIAL_DISPLACEMENT:
            // Range from 0 to the largest stored displacement at each frame
            dem_vis->SetNetworkColoring(coloring);
            break;
    }
    dem_vis->SetNetworkColormap(ChColormap::Type::JET);
    dem_vis->SetNetworkRadius(0.15 * radius);
    if (section_cut) {
        dem_vis->SetRegionSlab(ChVector3d(0, 0, 0), ChVector3d(0, 1, 0), 2 * radius);
        dem_vis->SetRegionParticleFiltering(true);
    }

    // Proxy bodies for the box walls (translucent, open at the top) and the lid, whose bottom face is the plane
    auto box_material = chrono_types::make_shared<ChVisualMaterial>();
    box_material->SetDiffuseColor(ChColor(0.5f, 0.6f, 0.7f));
    box_material->SetOpacity(0.25f);

    auto box = chrono_types::make_shared<ChBody>();
    double hw = width / 2 + thickness / 2;
    auto floor_shape = chrono_types::make_shared<ChVisualShapeBox>(width + 2 * thickness, width + 2 * thickness, thickness);
    auto wall_x_shape = chrono_types::make_shared<ChVisualShapeBox>(thickness, width, height);
    auto wall_y_shape = chrono_types::make_shared<ChVisualShapeBox>(width + 2 * thickness, thickness, height);
    floor_shape->AddMaterial(box_material);
    wall_x_shape->AddMaterial(box_material);
    wall_y_shape->AddMaterial(box_material);
    box->AddVisualShape(floor_shape, ChFramed(ChVector3d(0, 0, -thickness / 2)));
    box->AddVisualShape(wall_x_shape, ChFramed(ChVector3d(-hw, 0, height / 2)));
    box->AddVisualShape(wall_x_shape, ChFramed(ChVector3d(+hw, 0, height / 2)));
    box->AddVisualShape(wall_y_shape, ChFramed(ChVector3d(0, -hw, height / 2)));
    box->AddVisualShape(wall_y_shape, ChFramed(ChVector3d(0, +hw, height / 2)));
    dem_vis->AddProxyBody(box);

    auto lid = chrono_types::make_shared<ChBody>();
    auto lid_shape = chrono_types::make_shared<ChVisualShapeBox>(width, width, thickness);
    lid_shape->SetColor(ChColor(0.6f, 0.3f, 0.2f));
    lid->AddVisualShape(lid_shape, ChFramed(ChVector3d(0, 0, thickness / 2)));
    lid->SetPos(ChVector3d(0, 0, lid_z0));
    dem_vis->AddProxyBody(lid);

    // VSG visual system
    auto vis = chrono_types::make_shared<ChVisualSystemVSG>();
    vis->AttachPlugin(dem_vis);
    vis->SetWindowTitle("DEM force network");
    vis->SetWindowSize(1280, 800);
    vis->SetCameraVertical(CameraVerticalDir::Z);
    vis->AddCamera(ChVector3d(0.12, -0.42, 0.14), ChVector3d(0, 0, 0.05));
    vis->SetLightIntensity(1.0f);
    vis->SetLightDirection(5 * CH_PI / 3, CH_PI_4);  // from the camera side
    vis->Initialize();

    const std::string out_dir = GetChronoOutputPath() + "DEM_FORCE_NETWORK";
    if (save_images && !CreateOutputDirectory(std::filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    // Simulation loop (rendering continues after the end of the simulation, until the window is closed)
    double time = 0;
    int render_steps = static_cast<int>(std::ceil(1 / (render_fps * time_step)));
    int step = 0;
    int image = 0;
    while (vis->Run()) {
        if (time >= time_end) {
            vis->Render();
            continue;
        }

        if (step % render_steps == 0) {
            lid->SetPos(ChVector3d(0, 0, lid_z));
            vis->Render();
            if (save_images) {
                // The image of this frame is written during the next call to Render()
                std::ostringstream filename;
                filename << out_dir << "/img_" << std::setw(5) << std::setfill('0') << image++ << ".bmp";
                vis->WriteImageToFile(filename.str());
            }
        }

        dem_sys.AdvanceSimulation(time_step);
        time += time_step;
        step++;

        // Vertical motion of the lid (semi-implicit Euler), under gravity and the reaction force of the spheres
        ChVector3f force;
        dem_sys.GetBCReactionForces(lid_id, force);
        lid_vel += time_step * (force.z() / lid_mass - gravity);
        lid_z += time_step * lid_vel;
    }

    return 0;
}
