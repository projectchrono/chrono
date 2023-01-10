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
// Demonstration of the function to simulate vibrating feeders without the
// need of actually creating a vibrating object - just the info on the direction is needed.
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChFeeder.h"
#include "chrono/core/ChRealtimeStep.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

// Use the namespaces of Chrono
using namespace chrono;
using namespace chrono::collision;
using namespace chrono::irrlicht;

// Static values valid through the entire program (bad
// programming practice, but enough for quick tests)

double STATIC_flow = 100;
double STATIC_maxparticles = 30;
std::vector<std::shared_ptr<ChBody> > particlelist;

// Function that creates debris that fall on the conveyor belt, to be called at each dt.
// In this example, all debris particles share the same contact material.
void create_debris(ChVisualSystemIrrlicht& vis, ChSystem& sys, double dt, double particles_second) {
    if (particlelist.size() > STATIC_maxparticles)
        return;

    double xnozzlesize = 0.2;
    double ynozzle = 0.0;
    ////double znozzlesize = 0.56;

    ////double box_fraction = 0.7;  // 70% boxes, the rest are cylinders

    ////double sphrad = 0.013;

    double exact_particles_dt = dt * particles_second;
    double particles_dt = floor(exact_particles_dt);
    double remaind = exact_particles_dt - particles_dt;
    if (remaind > ChRandom())
        particles_dt += 1;

    auto item_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    item_mat->SetFriction(0.3f);

    for (int i = 0; i < particles_dt; i++) {
        if (particlelist.size() > STATIC_maxparticles)
            return;

        auto rigidBody = chrono_types::make_shared<ChBodyEasyMesh>(
            GetChronoDataFile("models/feeder_item.obj"),  // file name for OBJ Wavefront mesh
            1000,                                         // density of the body
            true,                                         // automatic evaluation of mass, COG position, inertia tensor
            true,                                         // attach visualization asset
            true,                                         // enable the collision detection
            item_mat,                                     // surface contact material
            0.002  // radius of 'inflating' of mesh (for more robust collision detection)
        );

        // position on staggered array for a non-intersecating cascade of items
        rigidBody->SetPos(
            ChVector<>(((particlelist.size() % 3) / 3.0 - 0.5) * xnozzlesize, ynozzle + particlelist.size() * 0.005,
                       ((((particlelist.size() - particlelist.size() % 3) % 5) / (5.0)) - 0.5) * xnozzlesize));

        rigidBody->GetVisualShape(0)->SetColor(ChColor(0.3f, 0.6f, 0.6f));

        sys.Add(rigidBody);

        vis.BindItem(rigidBody);

        particlelist.push_back(rigidBody);
    }
}

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create a ChronoENGINE physical system
    ChSystemNSC sys;

    // Set small collision envelopes for objects that will be created from now on..
    ChCollisionModel::SetDefaultSuggestedEnvelope(0.002);
    ChCollisionModel::SetDefaultSuggestedMargin(0.002);

    auto bowl_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    bowl_mat->SetFriction(0.2f);

    // Create the feeder bowl. Whatever object with collision can be used.
    // In our case we will use a ChBodyEasyMesh.

    auto feeder_bowl = chrono_types::make_shared<ChBodyEasyMesh>(
        GetChronoDataFile("models/feeder_bowl.obj"),  // file name for OBJ Wavefront mesh
        1000,                                         // density of the body
        true,                                         // automatic evaluation of mass, COG position, inertia tensor
        true,                                         // attach visualization asset
        true,                                         // enable the collision detection
        bowl_mat,                                     // surface contact material
        0.002  // radius of 'inflating' of mesh (for more robust collision detection)
    );
    feeder_bowl->SetBodyFixed(true);
    feeder_bowl->SetFrame_REF_to_abs(ChFrame<>(ChVector<>(0, -0.1, 0)));
    sys.Add(feeder_bowl);

    // Create the vibration-like effect for the bowl.
    // This is achieved with the special class ChFeeder. You do not need to setup a motor and springs for
    // generating the vibration of the feeder bowl (that would require very small time steps if the frequency
    // is realistic), because the bowl can be a static part and the ChFeeder object will modify the contacts
    // with the bowl so that they assume high frequency oscillations in that spot, and create a drifting effect.

    auto feeder = chrono_types::make_shared<ChFeeder>();
    feeder->SetFeederObject(feeder_bowl);  // Vibrating feeder drifting effect only for objects touching this.
    sys.Add(feeder);

    // To this end, you just need to set a x,y,z,Rx,Ry,Rz "virtual" vibration mode for the bowl, respect to some
    // reference. Example. typical vibration bowls for helicoidal feeding are vibrating both torsionally and up-down,
    // like turning a screw left & right, so our example has a mode with both Y vertical displacement and rotation on Y
    // axis:
    feeder->SetFeederVibration(
        ChFrame<>(
            ChVector<>(0, -0.1, 0)),  // this is the coordinate frame respect to whom we assume the virtual vibration
        0, 1, 0,                      // virtual vibration mode: x,y,z components. No need to normalize.
        0, 1, 0                       // virtual vibration mode: Rx,Ry,Rz rotation components. No need to normalize.
    );

    // Create the Irrlicht visualization system
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->AttachSystem(&sys);
    vis->SetWindowSize(1024, 768);
    vis->SetWindowTitle("Feeder bowl");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector<>(0.6, 0.5, -0.7), ChVector<>(0.0, 0.0, 0.0));
    vis->AddLight(ChVector<>(2.5, 1.4, -2.0), 6);

    // Simulation loop
    ChRealtimeStepTimer realtime_timer;
    double timestep = 0.005;
    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        sys.DoStepDynamics(timestep);

        // Continuosly create debris that fall on the conveyor belt
        create_debris(*vis, sys, timestep, STATIC_flow);

        realtime_timer.Spin(timestep);
    }

    return 0;
}
