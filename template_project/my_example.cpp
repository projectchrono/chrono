// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2025 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// A very simple example that can be used as template project for a Chrono
// simulation with 3D run-time visualization.
// =============================================================================

#include <chrono/physics/ChSystemNSC.h>
#include <chrono/physics/ChBodyEasy.h>
#include <chrono/physics/ChLinkMate.h>
#include <chrono/assets/ChTexture.h>
#include <chrono/core/ChRealtimeStep.h>
#include <chrono_irrlicht/ChVisualSystemIrrlicht.h>

// Use the namespaces of Chrono
using namespace chrono;
using namespace chrono::irrlicht;

int main(int argc, char* argv[]) {
    // Set path to Chrono data directory
    SetChronoDataPath(CHRONO_DATA_DIR);

    // 0 - Create a Chrono physical system
    ChSystemNSC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, -9.81, 0));
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // 1 - Create a fixed floor body (also used to represent the absolute reference)
    auto floor_body = std::make_shared<ChBodyEasyBox>(10, 2, 10,  // x, y, z dimensions
                                                      3000,       // density
                                                      true,       // create visualization asset
                                                      false       // no collision geometry
    );
    floor_body->SetFixed(true);
    floor_body->SetPos(ChVector3d(0, -2, 0));
    floor_body->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/checker1.png"), 2,
                                              2);  // optionally set color and/or texture for visual assets
    sys.Add(floor_body);

    // 2 - Create a pendulum body
    auto pendulum_body = std::make_shared<ChBodyEasyBox>(0.5, 2, 0.5,  // x, y, z dimensions
                                                         3000,         // density
                                                         true,         // create visualization asset
                                                         false         // no collision geometry
    );
    pendulum_body->SetPos(ChVector3d(0, 3, 0));
    pendulum_body->SetLinVel(ChVector3d(1, 0, 0));
    pendulum_body->GetVisualShape(0)->SetColor(ChColor(0.2f, 0.5f, 0.25f));
    sys.Add(pendulum_body);

    // 3 - Create a spherical constraint
    //     Here we use a ChLinkMateGeneric, but we could also create a ready-to-use ChLinkMateSpherical
    auto sperical_link =
        std::make_shared<ChLinkMateGeneric>(true, true, true, false, false, false);  // x, y, z, Rx, Ry, Rz constrains
    ChFrame<> link_position_abs(ChVector3d(0, 4, 0));
    sperical_link->Initialize(pendulum_body,      // the 1st body to connect
                              floor_body,         // the 2nd body to connect
                              false,              // the two following frames are in absolute, not relative, coordinates
                              link_position_abs,  // the link reference attached to 1st body
                              link_position_abs);  // the link reference attached to 2nd body
    sys.Add(sperical_link);

    // 4 - Create the Irrlicht visualization system
    ChVisualSystemIrrlicht vis;
    vis.SetWindowSize(800, 600);
    vis.SetWindowTitle("A simple project template");
    vis.Initialize();
    vis.AddLogo();
    vis.AddSkyBox();
    vis.AddTypicalLights();
    vis.AddCamera(ChVector3d(2, 2, -5), ChVector3d(0, 1, 0));
    vis.AttachSystem(&sys);

    // 5 - Simulation loop
    ChRealtimeStepTimer realtime_timer;
    double timestep = 5e-3;

    // Optionally customize solver settings
    sys.SetSolverType(ChSolver::Type::PSOR);
    sys.GetSolver()->AsIterative()->SetMaxIterations(100);
    sys.GetSolver()->AsIterative()->SetTolerance(1e-6);

    while (vis.Run()) {
        // Render scene
        vis.BeginScene();
        vis.Render();
        vis.EndScene();

        // Perform the time integration step
        sys.DoStepDynamics(timestep);

        // Spin in place to maintain soft real-time
        realtime_timer.Spin(timestep);
    }

    return 0;
}