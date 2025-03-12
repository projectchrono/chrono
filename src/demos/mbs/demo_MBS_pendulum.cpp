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
//     - creating a pendulum
//     - apply custom forces using accumulators
//     - creating constraints with limits
//     - 3D viewing with the Irrlicht library
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/core/ChTimer.h"
#include "chrono/core/ChRealtimeStep.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

// Use the namespaces of Chrono
using namespace chrono;
using namespace chrono::irrlicht;

// This function will be used to apply forces caused by
// a rotating fan, to all objects in front of it (a simple
// example just to demonstrate how to apply custom forces).

void apply_fan_force(ChSystemNSC* msystem,    // contains all bodies
                     ChCoordsys<>& fan_csys,  // pos and rotation of fan
                     double aradius,          // radius of fan
                     double aspeed,           // speed of fan
                     double adensity)         // density (heuristic)
{
    for (auto body : msystem->GetBodies()) {
        // Remember to reset 'user forces accumulators':
        body->EmptyAccumulators();

        // initialize speed of air (steady, if outside fan stream):
        ChVector3d abs_wind(0, 0, 0);

        // calculate the position of body COG in fan coordinates:
        ChVector3d mrelpos = fan_csys.TransformPointParentToLocal(body->GetPos());
        ChVector3d mrelpos_ondisc = mrelpos;
        mrelpos_ondisc.z() = 0;

        if (mrelpos.z() > 0)  // if not behind fan..
            if (mrelpos_ondisc.Length() < aradius) {
                // OK! we are inside wind stream cylinder..
                // wind is directed as normal to the fan disc
                abs_wind = fan_csys.TransformPointLocalToParent(ChVector3d(0, 0, 1));
                // wind inside fan stream is constant speed
                abs_wind *= -aspeed;
            }

        // force proportional to relative speed body-wind
        // and fluid density (NOTE! pretty simplified physics..)
        ChVector3d abs_force = (abs_wind - body->GetPosDt()) * adensity;
        // apply this force at the body COG
        body->AccumulateForce(abs_force, body->GetPos(), false);
    }
}

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Create a Chrono physical system
    ChSystemNSC sys;

    // Create all the rigid bodies

    // ..create the five pendulums

    for (int k = 0; k < 5; k++) {
        double z_step = (double)k * 2.;

        // .. the truss
        auto mrigidBody0 = chrono_types::make_shared<ChBodyEasyBox>(5, 1, 0.5,  // x,y,z size
                                                                    100,        // density
                                                                    true,       // visualization?
                                                                    false);     // collision?
        mrigidBody0->SetPos(ChVector3d(0, 0, z_step));
        mrigidBody0->SetFixed(true);  // the truss does not move!
        sys.Add(mrigidBody0);

        auto mrigidBody1 = chrono_types::make_shared<ChBodyEasyBox>(1, 6, 1,  // x,y,z size
                                                                    1,        // density
                                                                    true,     // visualization?
                                                                    false);   // collision?
        mrigidBody1->SetPos(ChVector3d(0, -3, z_step));
        sys.Add(mrigidBody1);

        auto mrigidBody2 = chrono_types::make_shared<ChBodyEasyBox>(1, 6, 1,  // x,y,z size
                                                                    1,        // density
                                                                    true,     // visualization?
                                                                    false);   // collision?
        mrigidBody2->SetPos(ChVector3d(0, -9, z_step));
        sys.Add(mrigidBody2);

        auto mrigidBody3 = chrono_types::make_shared<ChBodyEasyBox>(6, 1, 1,  // x,y,z size
                                                                    1,        // density
                                                                    true,     // visualization?
                                                                    false);   // collision?
        mrigidBody3->SetPos(ChVector3d(3, -12, z_step));
        sys.Add(mrigidBody3);

        //
        // Create the links between bodies!!!!
        //

        // .. a joint of type 'point on a line', with upper and lower limits on
        //    the X sliding direction, for the pendulum-ground constraint.
        auto my_link_01 = chrono_types::make_shared<ChLinkLockPointLine>();
        my_link_01->Initialize(mrigidBody1, mrigidBody0, ChFrame<>(ChVector3d(0, 0, z_step)));

        my_link_01->LimitX().SetActive(true);
        my_link_01->LimitX().SetMax(1.0);
        my_link_01->LimitX().SetMin(-1.0);

        sys.AddLink(my_link_01);

        // .. a spherical joint
        auto my_link_12 = chrono_types::make_shared<ChLinkLockSpherical>();
        my_link_12->Initialize(mrigidBody2, mrigidBody1, ChFrame<>(ChVector3d(0, -6, z_step)));
        sys.AddLink(my_link_12);

        // .. a spherical joint
        auto my_link_23 = chrono_types::make_shared<ChLinkLockSpherical>();
        my_link_23->Initialize(mrigidBody3, mrigidBody2, ChFrame<>(ChVector3d(0, -12, z_step)));
        sys.AddLink(my_link_23);
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

    // Create a 'fan ventilator' object, using Irrlicht mesh loading and handling
    // (this object is here for aesthetical reasons, it is NOT handled by Chrono)
    double fan_radius = 5.3;
    irr::scene::IAnimatedMesh* fanMesh = vis->GetSceneManager()->getMesh(GetChronoDataFile("models/fan2.obj").c_str());
    irr::scene::IAnimatedMeshSceneNode* fanNode = vis->GetSceneManager()->addAnimatedMeshSceneNode(fanMesh);
    fanNode->setScale(irr::core::vector3df((irr::f32)fan_radius, (irr::f32)fan_radius, (irr::f32)fan_radius));

    ////application.GetSystem()->SetSolverType(ChSolver::Type::BARZILAIBORWEIN);

    // Simulation loop
    double timestep = 0.01;
    ChRealtimeStepTimer realtime_timer;
    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        tools::drawGrid(vis.get(), 2, 2, 20, 20, ChCoordsys<>(ChVector3d(0, -20, 0), QuatFromAngleX(CH_PI_2)),
                        ChColor(0.3f, 0.5f, 0.5f), true);

        // Update the position of the spinning fan (an Irrlicht
        // node, which is here just for aesthetical reasons!)
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

        // Apply forces caused by fan & wind if Chrono rigid bodies are
        // in front of the fan, using a simple tutorial function (see above):
        apply_fan_force(&sys, my_fan_coord, fan_radius, 5.2, 0.5);

        sys.DoStepDynamics(timestep);
        realtime_timer.Spin(timestep);
    }

    return 0;
}
