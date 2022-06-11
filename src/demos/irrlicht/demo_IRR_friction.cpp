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
// Authors: Alessandro Tasora
// =============================================================================
//
// Demo code about
// - using rolling friction (not only sliding and static friction, available in
//   all objects by default)
// - optional sharing of some assets (visualization stuff)
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/core/ChRealtimeStep.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

// Use the namespaces of Chrono
using namespace chrono;
using namespace chrono::irrlicht;

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create a physical system
    ChSystemNSC sys;

    // Create all the rigid bodies.
    double mradius = 0.5;

    // Create a shared visualization material.
    auto sph_vis_mat = chrono_types::make_shared<ChVisualMaterial>();
    sph_vis_mat->SetKdTexture(GetChronoDataFile("textures/bluewhite.png"));

    // Create some spheres that roll horizontally, with increasing rolling friction values
    for (int bi = 0; bi < 10; bi++) {
        auto mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
        mat->SetFriction(0.4f);
        mat->SetRollingFriction(((float)bi / 10) * 0.05f);

        auto sphereBody = chrono_types::make_shared<ChBodyEasySphere>(mradius,  // radius size
                                                                      1000,     // density
                                                                      true,     // visualization?
                                                                      true,     // collision?
                                                                      mat);     // contact material

        // Set some properties
        sphereBody->SetPos(ChVector<>(-7, mradius - 0.5, -5 + bi * mradius * 2.5));
        sphereBody->GetVisualShape(0)->SetMaterial(0, sph_vis_mat);

        // Set initial speed: rolling in horizontal direction
        double initial_angspeed = 10;
        double initial_linspeed = initial_angspeed * mradius;
        sphereBody->SetWvel_par(ChVector<>(0, 0, -initial_angspeed));
        sphereBody->SetPos_dt(ChVector<>(initial_linspeed, 0, 0));

        // Add to the system
        sys.Add(sphereBody);
    }

    // Create some spheres that spin on place, for a 'drilling friction' case, with increasing spinning friction values
    for (int bi = 0; bi < 10; bi++) {
        auto mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
        mat->SetFriction(0.4f);
        mat->SetSpinningFriction(((float)bi / 10) * 0.02f);

        auto sphereBody = chrono_types::make_shared<ChBodyEasySphere>(mradius,  // radius size
                                                                      1000,     // density
                                                                      true,     // visualization?
                                                                      true,     // collision?
                                                                      mat);     // contact material
        // Set some properties
        sphereBody->SetPos(ChVector<>(-8, 1 + mradius - 0.5, -5 + bi * mradius * 2.5));
        sphereBody->GetVisualShape(0)->SetMaterial(0, sph_vis_mat);

        // Set initial speed: spinning in vertical direction
        sphereBody->SetWvel_par(ChVector<>(0, 20, 0));

        // Add to the system
        sys.Add(sphereBody);

        // Notes:
        // - setting nonzero spinning friction and/or setting nonzero rolling friction
        //   affects the speed of the solver (each contact eats 2x of CPU time repsect to the
        //   case of simple sliding/staic contact)
        // - avoid using zero spinning friction with nonzero rolling friction.
    }

    // Create a container fixed to ground
    auto bin = chrono_types::make_shared<ChBody>();
    bin->SetPos(ChVector<>(0, -1, 0));
    bin->SetBodyFixed(true);
    bin->SetCollide(true);

    // Set rolling and friction coefficients for the container.
    // By default, the composite material will use the minimum value for an interacting collision pair.
    auto bin_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    bin_mat->SetRollingFriction(1);
    bin_mat->SetSpinningFriction(1);

    // Create a shared visualization material.
    auto bin_vis_mat = chrono_types::make_shared<ChVisualMaterial>();
    bin_vis_mat->SetKdTexture(GetChronoDataFile("textures/blue.png"));

    // Add collision geometry and visualization shapes for the floor and the 4 walls
    bin->GetCollisionModel()->ClearModel();
    utils::AddBoxGeometry(bin.get(), bin_mat, ChVector<>(20, 1, 20) / 2.0, ChVector<>(0, 0, 0), QUNIT, true,
                          bin_vis_mat);
    utils::AddBoxGeometry(bin.get(), bin_mat, ChVector<>(1, 2, 20.99) / 2.0, ChVector<>(-10, 1, 0), QUNIT, true,
                          bin_vis_mat);
    utils::AddBoxGeometry(bin.get(), bin_mat, ChVector<>(1, 2, 20.99) / 2.0, ChVector<>(10, 1, 0), QUNIT, true,
                          bin_vis_mat);
    utils::AddBoxGeometry(bin.get(), bin_mat, ChVector<>(20.99, 2, 1) / 2.0, ChVector<>(0, 1, -10), QUNIT, true,
                          bin_vis_mat);
    utils::AddBoxGeometry(bin.get(), bin_mat, ChVector<>(20.99, 2, 1) / 2.0, ChVector<>(0, 1, 10), QUNIT, true,
                          bin_vis_mat);
    bin->GetCollisionModel()->BuildModel();

    sys.Add(bin);

    // Create the Irrlicht visualization system
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    sys.SetVisualSystem(vis);
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("Rolling and spinning friction");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector<>(0, 14, -20));
    vis->AddLight(ChVector<>(30.f, 100.f, 30.f), 290, ChColor(0.7f, 0.7f, 0.7f));
    vis->AddLight(ChVector<>(-30.f, 100.f, -30.f), 190, ChColor(0.7f, 0.8f, 0.8f));

    // Modify some setting of the physical system for the simulation
    sys.SetSolverType(ChSolver::Type::APGD);
    sys.SetSolverMaxIterations(100);

    // Simulation loop
    double timestep = 0.005;
    ChRealtimeStepTimer realtime_timer;
    while (vis->Run()) {
        vis->BeginScene();
        vis->DrawAll();
        vis->EndScene();
        sys.DoStepDynamics(timestep);
        realtime_timer.Spin(timestep);
    }

    return 0;
}
