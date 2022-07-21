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
// A very simple example that can be used as template project for
// a Chrono::Engine simulator with 3D view.
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/core/ChRealtimeStep.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

// Use the namespace of Chrono
using namespace chrono;
using namespace chrono::irrlicht;

int main(int argc, char* argv[]) {
    // Set path to Chrono data directory
    SetChronoDataPath(CHRONO_DATA_DIR);
    
    // Create a Chrono physical system
    ChSystemNSC sys;


    // Pendulum example ------------------------------------

    // 1 - Create a floor that is fixed (that is used also to represent the absolute reference)

    auto floorBody = std::make_shared<ChBodyEasyBox>(10, 2, 10,  // x, y, z dimensions
                                                     3000,       // density
                                                     true,       // create visualization asset
                                                     false       // no collision geometry
                                                     );
    floorBody->SetPos(ChVector<>(0, -2, 0));
    floorBody->SetBodyFixed(true);

    sys.Add(floorBody);

    // 2 - Create a pendulum

    auto pendulumBody = std::make_shared<ChBodyEasyBox>(0.5, 2, 0.5,  // x, y, z dimensions
                                                        3000,         // density
                                                        true,         // create visualization asset
                                                        false         // no collision geometry
                                                        );
    pendulumBody->SetPos(ChVector<>(0, 3, 0));
    pendulumBody->SetPos_dt(ChVector<>(1, 0, 0));

    sys.Add(pendulumBody);

    // 3 - Create a spherical constraint.
    //   Here we'll use a ChLinkMateGeneric, but we could also use ChLinkLockSpherical

    auto sphericalLink =
        std::make_shared<ChLinkMateGeneric>(true, true, true, false, false, false);  // x,y,z,Rx,Ry,Rz constrains
    ChFrame<> link_position_abs(ChVector<>(0, 4, 0));

    sphericalLink->Initialize(pendulumBody,        // the 1st body to connect
                              floorBody,           // the 2nd body to connect
                              false,               // the two following frames are in absolute, not relative, coords.
                              link_position_abs,   // the link reference attached to 1st body
                              link_position_abs);  // the link reference attached to 2nd body

    sys.Add(sphericalLink);

    // Optionally, set color and/or texture for visual assets
    pendulumBody->GetVisualShape(0)->SetColor(ChColor(0.2f, 0.5f, 0.25f));
    floorBody->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/checker1.png"), 2, 2);

    // 4 - Create the Irrlicht visualization system
    ChVisualSystemIrrlicht vis;
    vis.SetWindowSize(800, 600);
    vis.SetWindowTitle("A simple project template");
    vis.Initialize();
    vis.AddLogo();
    vis.AddSkyBox();
    vis.AddTypicalLights();
    vis.AddCamera(ChVector<>(2, 2, -5), ChVector<>(0, 1, 0));
    vis.AttachSystem(&sys);

    // 5 - Simulation loop
    ChRealtimeStepTimer realtime_timer;
    double step_size = 5e-3;

    while (vis.Run()) {
        // Render scene
        vis.BeginScene();
        vis.Render();
        vis.EndScene();

        // Perform the integration stpe
        sys.DoStepDynamics(step_size);

        // Spin in place to maintain soft real-time
        realtime_timer.Spin(step_size);
    }

    return 0;
}
