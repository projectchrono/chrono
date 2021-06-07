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

#include "chrono_irrlicht/ChIrrApp.h"

// Use the namespaces of Chrono
using namespace chrono;
using namespace chrono::irrlicht;

// Use the main namespaces of Irrlicht
using namespace irr;
using namespace irr::core;
using namespace irr::scene;
using namespace irr::video;
using namespace irr::io;
using namespace irr::gui;

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create a physical system
    ChSystemNSC mphysicalSystem;

    // Create the Irrlicht visualization, attach camera and lights, set sky and logo
    ChIrrApp application(&mphysicalSystem, L"Rolling friction", core::dimension2d<u32>(800, 600));
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights(vector3df(30.f, 100.f, 30.f), vector3df(-30.f, 100.f, -30.f));
    application.AddTypicalCamera(vector3df(0, 14, -20));

    // Create all the rigid bodies.
    double mradius = 0.5;

    // Create a texture asset. It can be shared between bodies.
    auto textureasset = chrono_types::make_shared<ChTexture>(GetChronoDataFile("textures/bluewhite.png"));

    // Create some spheres that roll horizontally, with increasing rolling friction values
    for (int bi = 0; bi < 10; bi++) {
        auto mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
        mat->SetFriction(0.4f);
        mat->SetRollingFriction(((float)bi / 10) * 0.05f);

        auto msphereBody = chrono_types::make_shared<ChBodyEasySphere>(mradius,  // radius size
                                                                       1000,     // density
                                                                       true,     // visualization?
                                                                       true,     // collision?
                                                                       mat);     // contact material

        // Set some properties
        msphereBody->SetPos(ChVector<>(-7, mradius - 0.5, -5 + bi * mradius * 2.5));
        msphereBody->AddAsset(textureasset);  // assets can be shared

        // Set initial speed: rolling in horizontal direction
        double initial_angspeed = 10;
        double initial_linspeed = initial_angspeed * mradius;
        msphereBody->SetWvel_par(ChVector<>(0, 0, -initial_angspeed));
        msphereBody->SetPos_dt(ChVector<>(initial_linspeed, 0, 0));

        // Add to the system
        mphysicalSystem.Add(msphereBody);
    }

    // Create some spheres that spin on place, for a 'drilling friction' case, with increasing spinning friction values
    for (int bi = 0; bi < 10; bi++) {
        auto mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
        mat->SetFriction(0.4f);
        mat->SetSpinningFriction(((float)bi / 10) * 0.02f);

        auto msphereBody = chrono_types::make_shared<ChBodyEasySphere>(mradius,  // radius size
                                                                       1000,     // density
                                                                       true,     // visualization?
                                                                       true,     // collision?
                                                                       mat);     // contact material
        // Set some properties
        msphereBody->SetPos(ChVector<>(-8, 1 + mradius - 0.5, -5 + bi * mradius * 2.5));
        msphereBody->AddAsset(textureasset);  // assets can be shared

        // Set initial speed: spinning in vertical direction
        msphereBody->SetWvel_par(ChVector<>(0, 20, 0));

        // Add to the system
        mphysicalSystem.Add(msphereBody);

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

    // Add collision geometry and visualization shapes for the floor and the 4 walls
    bin->GetCollisionModel()->ClearModel();
    utils::AddBoxGeometry(bin.get(), bin_mat, ChVector<>(20, 1, 20) / 2.0, ChVector<>(0, 0, 0));
    utils::AddBoxGeometry(bin.get(), bin_mat, ChVector<>(1, 2, 20.99) / 2.0, ChVector<>(-10, 1, 0));
    utils::AddBoxGeometry(bin.get(), bin_mat, ChVector<>(1, 2, 20.99) / 2.0, ChVector<>(10, 1, 0));
    utils::AddBoxGeometry(bin.get(), bin_mat, ChVector<>(20.99, 2, 1) / 2.0, ChVector<>(0, 1, -10));
    utils::AddBoxGeometry(bin.get(), bin_mat, ChVector<>(20.99, 2, 1) / 2.0, ChVector<>(0, 1, 10));
    bin->GetCollisionModel()->BuildModel();

    bin->AddAsset(chrono_types::make_shared<ChTexture>(GetChronoDataFile("textures/blue.png")));

    mphysicalSystem.Add(bin);

    // Complete asset construction
    application.AssetBindAll();
    application.AssetUpdateAll();

    // Modify some setting of the physical system for the simulation
    mphysicalSystem.SetSolverType(ChSolver::Type::APGD);
    mphysicalSystem.SetSolverMaxIterations(100);

    // Simulation loop
    application.SetTimestep(0.005);
    application.SetTryRealtime(true);

    while (application.GetDevice()->run()) {
        application.BeginScene(true, true, SColor(255, 140, 161, 192));
        application.DrawAll();
        application.DoStep();
        application.EndScene();
    }

    return 0;
}
