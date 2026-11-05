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
// Chrono::Multicore demo of the Chrono::VSG plugin for Multicore systems.
//
// Spheres fall into a box and are compressed by a heavy lid, which can move only
// vertically. The plugin draws the contact force network (contacts with the walls
// and the lid included) and the spheres, as GPU instances, and shows a panel with
// timers, contact counts, and solver statistics. The spheres are hidden at start,
// since the network runs inside them; the panel has checkboxes for both.
//
// The SMC contact model uses the MultiStep tangential displacement model, so the
// network can also be colored by the stored tangential displacement.
//
// The global reference frame has Z up.
// =============================================================================

#include <random>

#include "chrono/ChConfig.h"
#include "chrono/physics/ChLinkLock.h"
#include "chrono/utils/ChUtilsCreators.h"

#include "chrono_multicore/physics/ChSystemMulticore.h"
#include "chrono_multicore/visualization/ChMulticoreVisualizationVSG.h"

#include "chrono_vsg/ChVisualSystemVSG.h"

using namespace chrono;
using namespace chrono::vsg3d;

// -----------------------------------------------------------------------------

// Quantity used to color the force network
auto coloring = ChMulticoreVisualizationVSG::NetworkColoring::NORMAL_FORCE;

// Draw only a slab through the middle of the box (section cut), for both the network and the spheres
bool section_cut = false;

// Save images of the rendered frames
bool save_images = false;

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2026 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Problem parameters
    double radius = 0.005;       // mean sphere radius (radii uniform in [0.8, 1.2] times the mean)
    double density = 2500;       // sphere density
    double width = 0.2;          // inner width of the box (in x and y)
    double height = 0.2;         // inner height of the box
    double thickness = 0.01;     // wall thickness
    int num_layers = 10;         // number of layers of spheres
    double lid_pressure = 2000;  // pressure applied by the lid
    double gravity = 9.81;       // gravitational acceleration
    double time_step = 1e-4;     // integration step size
    double time_end = 2;         // simulation length
    double render_fps = 50;      // rendering frequency (in simulation time)

    // Create the system
    ChSystemMulticoreSMC sys;
    sys.SetCollisionSystemType(ChCollisionSystem::Type::MULTICORE);
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -gravity));
    sys.SetNumThreads(8);

    sys.GetSettings()->solver.contact_force_model = ChSystemSMC::ContactForceModel::Hertz;
    sys.GetSettings()->solver.tangential_displ_mode = ChSystemSMC::TangentialDisplacementModel::MultiStep;
    sys.GetSettings()->collision.narrowphase_algorithm = ChNarrowphase::Algorithm::HYBRID;
    sys.GetSettings()->collision.bins_per_axis = vec3(20, 20, 20);

    auto material = chrono_types::make_shared<ChContactMaterialSMC>();
    material->SetYoungModulus(1e7f);
    material->SetPoissonRatio(0.3f);
    material->SetFriction(0.5f);
    material->SetRestitution(0.3f);

    // Box (fixed, open at the top, with translucent walls), in collision family 1
    auto box_material = chrono_types::make_shared<ChVisualMaterial>();
    box_material->SetDiffuseColor(ChColor(0.5f, 0.6f, 0.7f));
    box_material->SetOpacity(0.25f);

    auto box = chrono_types::make_shared<ChBody>();
    box->SetFixed(true);
    box->EnableCollision(true);
    utils::AddBoxContainer(box, material, ChFrame<>(ChVector3d(0, 0, height / 2), QUNIT), ChVector3d(width, width, height), thickness, ChVector3i(2, 2, -1), true, box_material);
    box->GetCollisionModel()->SetFamily(1);
    sys.AddBody(box);

    // Grid of initial sphere positions
    double pitch = 2.5 * radius;
    int n = static_cast<int>(std::floor(width / pitch));

    // Lid, above the spheres, sliding along a vertical prismatic joint; it does not collide with the box.
    // The lid is added before the spheres: the MultiStep model stores the contact history on the body with the
    // larger index, which then has to be the sphere (a body has room for the histories of 20 contacts).
    double lid_mass = lid_pressure * width * width / gravity;
    ChVector3d lid_size(width - 2 * radius, width - 2 * radius, thickness);
    ChVector3d lid_pos(0, 0, pitch * (num_layers + 1) + thickness / 2);

    auto lid = chrono_types::make_shared<ChBody>();
    lid->SetMass(lid_mass);
    double lx = lid_size.x();
    double ly = lid_size.y();
    double lz = lid_size.z();
    lid->SetInertiaXX(lid_mass / 12 * ChVector3d(ly * ly + lz * lz, lx * lx + lz * lz, lx * lx + ly * ly));
    lid->SetPos(lid_pos);
    lid->EnableCollision(true);
    utils::AddBoxGeometry(lid.get(), material, lid_size);
    lid->GetVisualShape(0)->SetColor(ChColor(0.6f, 0.3f, 0.2f));
    lid->GetCollisionModel()->SetFamily(2);
    lid->GetCollisionModel()->DisallowCollisionsWith(1);
    sys.AddBody(lid);

    auto prismatic = chrono_types::make_shared<ChLinkLockPrismatic>();
    prismatic->Initialize(box, lid, ChFrame<>(lid_pos, QUNIT));
    sys.AddLink(prismatic);

    // Spheres, on the grid with a random horizontal offset
    std::mt19937 generator(1);
    std::uniform_real_distribution<double> unif(-1, 1);
    for (int k = 0; k < num_layers; k++) {
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                double r = radius * (1 + 0.2 * unif(generator));
                double mass = density * CH_4_3 * CH_PI * r * r * r;
                ChVector3d pos(pitch * (i - 0.5 * (n - 1)) + 0.1 * radius * unif(generator),  //
                               pitch * (j - 0.5 * (n - 1)) + 0.1 * radius * unif(generator),  //
                               pitch * (k + 0.5));

                auto sphere = chrono_types::make_shared<ChBody>();
                sphere->SetMass(mass);
                sphere->SetInertiaXX(0.4 * mass * r * r * ChVector3d(1, 1, 1));
                sphere->SetPos(pos);
                sphere->EnableCollision(true);
                utils::AddSphereGeometry(sphere.get(), material, r);
                sphere->GetVisualShape(0)->SetColor(ChColor(0.7f, 0.7f, 0.7f));
                sys.AddBody(sphere);
            }
        }
    }
    int num_spheres = num_layers * n * n;
    std::cout << "Number of spheres: " << num_spheres << std::endl;

    // Multicore plugin for the VSG visual system
    auto mcore_vis = chrono_types::make_shared<ChMulticoreVisualizationVSG>(&sys);
    mcore_vis->SetNetworkVisibility(true);
    // The half-segments of a contact run inside the two spheres, so the spheres are hidden (the plugin panel has a
    // checkbox to show them)
    mcore_vis->SetBodyVisibility(false);
    mcore_vis->SetNetworkMode(ChForceNetworkVSG::Mode::CYLINDERS);
    switch (coloring) {
        case ChMulticoreVisualizationVSG::NetworkColoring::NORMAL_FORCE: {
            // Range from the force of the lid on one sphere of the top layer
            double force = lid_mass * gravity / (n * n);
            mcore_vis->SetNetworkColoring(coloring, 0, 3 * force);
            break;
        }
        case ChMulticoreVisualizationVSG::NetworkColoring::COULOMB_RATIO:
            mcore_vis->SetNetworkColoring(coloring, 0, 1);
            break;
        case ChMulticoreVisualizationVSG::NetworkColoring::TANGENTIAL_DISPLACEMENT:
            // Range from 0 to the largest stored displacement at each frame
            mcore_vis->SetNetworkColoring(coloring);
            break;
    }
    mcore_vis->SetNetworkColormap(ChColormap::Type::JET);
    mcore_vis->SetNetworkRadius(0.15 * radius);
    if (section_cut) {
        mcore_vis->SetRegionSlab(ChVector3d(0, 0, 0), ChVector3d(0, 1, 0), 2 * radius);
        mcore_vis->SetRegionBodyFiltering(true);
    }
    if (save_images) {
        mcore_vis->SetImageOutputDirectory(GetChronoOutputPath() + "MCORE_FORCE_NETWORK");
        mcore_vis->SetImageOutput(true);
    }

    // VSG visual system
    auto vis = chrono_types::make_shared<ChVisualSystemVSG>();
    vis->AttachSystem(&sys);
    vis->AttachPlugin(mcore_vis);
    vis->SetWindowTitle("Multicore force network");
    vis->SetWindowSize(1280, 800);
    vis->SetCameraVertical(CameraVerticalDir::Z);
    vis->AddCamera(ChVector3d(0.12, -0.42, 0.14), ChVector3d(0, 0, 0.05));
    vis->SetLightIntensity(1.0f);
    vis->SetLightDirection(5 * CH_PI / 3, CH_PI_4);  // from the camera side
    vis->Initialize();

    std::cout << "Instanced bodies: " << mcore_vis->GetNumInstancedBodies() << " of " << sys.GetBodies().size() << std::endl;

    // Simulation loop (rendering continues after the end of the simulation, until the window is closed)
    double time = 0;
    int render_steps = static_cast<int>(std::ceil(1 / (render_fps * time_step)));
    int step = 0;
    while (vis->Run()) {
        if (time >= time_end) {
            vis->Render();
            continue;
        }

        if (step % render_steps == 0)
            vis->Render();

        sys.DoStepDynamics(time_step);
        time += time_step;
        step++;
    }

    return 0;
}
