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
// Authors: Simone Benatti
// =============================================================================
//
// Demo code about
//     - creating a pendulum
//     - apply custom forces using accumulators
//     - creating constraints with limits
//     - 3D viewing with the Irrlicht library
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemPBD.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/core/ChTimer.h"
#include "chrono/core/ChRealtimeStep.h"

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

    // Create a ChronoENGINE physical system
    ChSystemPBD my_system;
    //my_system.SetSubsteps(2);
    collision::ChCollisionModel::SetDefaultSuggestedEnvelope(0.0001);
    collision::ChCollisionModel::SetDefaultSuggestedMargin(0.0005);
    double cable_length = 1.5;
    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(&my_system, L"A simple pendulum example", core::dimension2d<u32>(800, 600));
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(.5, -cable_length, 0), core::vector3df(0, -cable_length, 0));

    //
    // Create all the rigid bodies!!!!
    //
    // ceiling
    auto ceiling = chrono_types::make_shared<ChBody>();
    ceiling->SetBodyFixed(true);

    // ..create the five pendulums
    std::vector<ChBodyEasySphere> spheresvec;
    std::vector<ChLinkDistance> linkvec;
    auto sph_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>(); 
    double rad = .01;
    
    for (int k = 0; k < 5; k++) {
        double z_step = (double)k * 2 * (rad + 1E-3);

        // .. the ball:
        auto ball = chrono_types::make_shared<ChBodyEasySphere>(rad,    // radius
                                                                8000,   // density: steel
                                                                true,   // visualization?
                                                                true,  // collision?
                                                                sph_mat);
        spheresvec.push_back(*ball);
        ball->SetPos(ChVector<>(0, -cable_length, z_step));
        ball->SetBodyFixed(false);
        if (k==0)
            ball->SetPos_dt(ChVector<>(0, 0, -.1));
        my_system.Add(ball);

        //
        // The cable, modelled as a distance joint:
        //


        auto link_cable = chrono_types::make_shared<ChLinkDistance>();  // right, front, upper, 1
        link_cable->Initialize(ball, ceiling, false, ChVector<>(0, -cable_length + rad, z_step),
                               ChVector<>(0, 0, z_step));
        my_system.AddLink(link_cable);    
        linkvec.push_back(*link_cable);
    }

    //
    // THE SOFT-REAL-TIME CYCLE
    //

    // Use this function for adding a ChIrrNodeAsset to all items
    // Otherwise use application.AssetBind(myitem); on a per-item basis.
    application.AssetBindAll();

    // Use this function for 'converting' assets into Irrlicht meshes
    application.AssetUpdateAll();

    application.SetTimestep(0.01);
    application.SetTryRealtime(true);

    //application.GetSystem()->SetSolverType(ChSolver::Type::BARZILAIBORWEIN); // if you need a more precise CCP solver..


    while (application.GetDevice()->run()) {
        // Irrlicht must prepare frame to draw
        application.BeginScene(true, true, SColor(255, 140, 161, 192));

        // Irrlicht application draws all 3D objects and all GUI items
        application.DrawAll();

        // Draw also a grid on the horizontal XZ plane
        tools::drawGrid(application.GetVideoDriver(), 2, 2, 20, 20,
                             ChCoordsys<>(ChVector<>(0, -20, 0), Q_from_AngX(CH_C_PI_2)),
                             video::SColor(255, 80, 100, 100), true);

        // HERE CHRONO INTEGRATION IS PERFORMED: THE
        // TIME OF THE SIMULATION ADVANCES FOR A SINGLE
        // STEP:
        application.DoStep();

        application.EndScene();
    }

    return 0;
}
