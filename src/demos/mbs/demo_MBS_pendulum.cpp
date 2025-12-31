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
// Authors: Alessandro Tasora, Radu Serban
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
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/core/ChRealtimeStep.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

using namespace chrono;
using namespace chrono::irrlicht;

// -----------------------------------------------------------------------------

// Apply forces caused by a rotating fan, to all objects in front of it.
void apply_fan_force(ChSystem& system,        // contains all bodies
                     ChCoordsys<>& fan_csys,  // pos and rotation of fan
                     double aradius,          // radius of fan
                     double aspeed,           // speed of fan
                     double adensity)         // density (heuristic)
{
    // Add forces to the body accumulator (only one accumulator defined for each body)
    for (auto& body : system.GetBodies()) {
        // Reset accumulator
        body->EmptyAccumulator(0);

        // Initialize speed of air (steady, if outside fan stream)
        ChVector3d abs_wind(0, 0, 0);

        // Calculate the position of body COG in fan coordinates
        ChVector3d mrelpos = fan_csys.TransformPointParentToLocal(body->GetPos());
        ChVector3d mrelpos_ondisc = mrelpos;
        mrelpos_ondisc.z() = 0;

        if (mrelpos.z() > 0)  // if not behind fan..
            if (mrelpos_ondisc.Length() < aradius) {
                // Inside wind stream cylinder; wind is directed as normal to the fan disc
                abs_wind = fan_csys.TransformPointLocalToParent(ChVector3d(0, 0, 1));
                // Wind inside fan stream is constant speed
                abs_wind *= -aspeed;
            }

        // Simple force proportional to relative speed body-wind and fluid density
        ChVector3d abs_force = (abs_wind - body->GetPosDt()) * adensity;

        // apply this force at the body COG
        body->AccumulateForce(0, abs_force, body->GetPos(), false);
    }
}

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Create a Chrono physical system
    ChSystemNSC sys;

    // Create five pendulum chains
    for (int k = 0; k < 5; k++) {
        double z_step = (double)k * 2.;

        // Create pendulum bodies
        auto body0 = chrono_types::make_shared<ChBodyEasyBox>(5, 1, 0.5,  // x,y,z size
                                                              100,        // density
                                                              true,       // visualization?
                                                              false);     // collision?
        body0->SetPos(ChVector3d(0, 0, z_step));
        body0->SetFixed(true);
        sys.Add(body0);

        auto body1 = chrono_types::make_shared<ChBodyEasyBox>(1, 6, 1,  // x,y,z size
                                                              1,        // density
                                                              true,     // visualization?
                                                              false);   // collision?
        body1->SetPos(ChVector3d(0, -3, z_step));
        sys.Add(body1);

        auto body2 = chrono_types::make_shared<ChBodyEasyBox>(1, 6, 1,  // x,y,z size
                                                              1,        // density
                                                              true,     // visualization?
                                                              false);   // collision?
        body2->SetPos(ChVector3d(0, -9, z_step));
        sys.Add(body2);

        auto body3 = chrono_types::make_shared<ChBodyEasyBox>(6, 1, 1,  // x,y,z size
                                                              1,        // density
                                                              true,     // visualization?
                                                              false);   // collision?
        body3->SetPos(ChVector3d(3, -12, z_step));
        sys.Add(body3);

        // Create a joint of type 'point on a line', with upper and lower limits on the X sliding direction=
        auto point_line_01 = chrono_types::make_shared<ChLinkLockPointLine>();
        point_line_01->Initialize(body1, body0, ChFrame<>(ChVector3d(0, 0, z_step)));

        point_line_01->LimitX().SetActive(true);
        point_line_01->LimitX().SetMax(1.0);
        point_line_01->LimitX().SetMin(-1.0);

        sys.AddLink(point_line_01);

        // Create a spherical joint
        auto spherical_12 = chrono_types::make_shared<ChLinkLockSpherical>();
        spherical_12->Initialize(body2, body1, ChFrame<>(ChVector3d(0, -6, z_step)));
        sys.AddLink(spherical_12);

        // Create a second spherical joint
        auto spherical_23 = chrono_types::make_shared<ChLinkLockSpherical>();
        spherical_23->Initialize(body3, body2, ChFrame<>(ChVector3d(0, -12, z_step)));
        sys.AddLink(spherical_23);
    }

    // Create the Irrlicht visualization system
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->AttachSystem(&sys);
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("A simple pendulum example");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector3d(0, 14, -20));
    vis->AddTypicalLights();

    // Create a 'fan ventilator' object (visualization only)
    double fan_radius = 5.3;
    irr::scene::IAnimatedMesh* fanMesh = vis->GetSceneManager()->getMesh(GetChronoDataFile("models/fan2.obj").c_str());
    irr::scene::IAnimatedMeshSceneNode* fanNode = vis->GetSceneManager()->addAnimatedMeshSceneNode(fanMesh);
    fanNode->setScale(irr::core::vector3df((irr::f32)fan_radius, (irr::f32)fan_radius, (irr::f32)fan_radius));

    // Add one force accumulator to each body in the system
    for (auto& body : sys.GetBodies()) {
        body->AddAccumulator();
    }

    // Simulation loop
    double timestep = 0.01;
    ChRealtimeStepTimer realtime_timer;

    while (vis->Run()) {
        vis->BeginScene();

        vis->Render();
        
        tools::drawGrid(vis.get(), 2, 2, 20, 20, ChCoordsys<>(ChVector3d(0, -20, 0), QuatFromAngleX(CH_PI_2)),
                        ChColor(0.3f, 0.5f, 0.5f), true);

        // Update the position of the spinning fan (for visualization purposes only)
        ChQuaternion<> my_fan_rotation;
        my_fan_rotation.SetFromAngleY(sys.GetChTime() * -0.5);
        ChQuaternion<> my_fan_spin;
        my_fan_spin.SetFromAngleZ(sys.GetChTime() * 4);
        ChCoordsys<> my_fan_coord(ChVector3d(12, -6, 0), my_fan_rotation);
        ChFrame<> my_fan_framerotation(my_fan_coord);
        ChFrame<> my_fan_framespin(ChCoordsys<>(VNULL, my_fan_spin));
        ChCoordsys<> my_fan_coordsys = (my_fan_framespin >> my_fan_framerotation).GetCoordsys();
        tools::alignIrrlichtNode(fanNode, my_fan_coordsys);

        vis->EndScene();

        // Apply forces caused by fan & wind if Chrono rigid bodies are in front of the fan, using a simple function
        apply_fan_force(sys, my_fan_coord, fan_radius, 5.2, 0.5);

        sys.DoStepDynamics(timestep);
        realtime_timer.Spin(timestep);
    }

    return 0;
}
