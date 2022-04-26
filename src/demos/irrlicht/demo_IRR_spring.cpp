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
// Authors: Radu Serban
// =============================================================================
//
// Simple example demonstrating the use of ChLinkTSDA.
//
// Two bodies, connected with identical (but modeled differently) spring-dampers
// are created side by side.
//
// Recall that Irrlicht uses a left-hand frame, so everything is rendered with
// left and right flipped.
//
// =============================================================================

#include <cstdio>

#include "chrono/assets/ChPointPointShape.h"

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBody.h"
#include "chrono/core/ChRealtimeStep.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

using namespace chrono;
using namespace chrono::irrlicht;

using namespace irr;

// =============================================================================

double rest_length = 1.5;
double spring_coef = 50;
double damping_coef = 1;

// =============================================================================

// Functor class implementing the force for a ChLinkTSDA link.
// In this simple demonstration, we just reimplement the default linear spring-damper.
class MySpringForce : public ChLinkTSDA::ForceFunctor {
    virtual double evaluate(double time,            // current time
                            double restlength,      // undeformed length
                            double length,          // current length
                            double vel,             // current velocity (positive when extending)
                            const ChLinkTSDA& link  // associated link
                            ) override {
        double force = -spring_coef * (length - restlength) - damping_coef * vel;
        return force;
    }
};

// =============================================================================

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    ChSystemNSC sys;
    sys.Set_G_acc(ChVector<>(0, 0, 0));

    // Create the ground body with two visualization spheres
    // -----------------------------------------------------

    auto ground = chrono_types::make_shared<ChBody>();
    sys.AddBody(ground);
    ground->SetIdentifier(-1);
    ground->SetBodyFixed(true);
    ground->SetCollide(false);

    {
        auto sph_1 = chrono_types::make_shared<ChSphereShape>();
        sph_1->GetSphereGeometry().rad = 0.1;
        ground->AddVisualShape(sph_1, ChFrame<>(ChVector<>(-1, 0, 0), QUNIT));

        auto sph_2 = chrono_types::make_shared<ChSphereShape>();
        sph_2->GetSphereGeometry().rad = 0.1;
        ground->AddVisualShape(sph_2, ChFrame<>(ChVector<>(+1, 0, 0), QUNIT));
    }

    // Create a body suspended through a ChLinkTSDA (default linear)
    // -------------------------------------------------------------

    auto body_1 = chrono_types::make_shared<ChBody>();
    sys.AddBody(body_1);
    body_1->SetPos(ChVector<>(-1, -3, 0));
    body_1->SetIdentifier(1);
    body_1->SetBodyFixed(false);
    body_1->SetCollide(false);
    body_1->SetMass(1);
    body_1->SetInertiaXX(ChVector<>(1, 1, 1));

    // Attach a visualization asset.
    auto box_1 = chrono_types::make_shared<ChBoxShape>();
    box_1->GetBoxGeometry().SetLengths(ChVector<>(1, 1, 1));
    box_1->SetColor(ChColor(0.6f, 0, 0));
    body_1->AddVisualShape(box_1);

    // Create the spring between body_1 and ground. The spring end points are
    // specified in the body relative frames.
    auto spring_1 = chrono_types::make_shared<ChLinkTSDA>();
    spring_1->Initialize(body_1, ground, true, ChVector<>(0, 0, 0), ChVector<>(-1, 0, 0));
    spring_1->SetRestLength(rest_length);
    spring_1->SetSpringCoefficient(spring_coef);
    spring_1->SetDampingCoefficient(damping_coef);
    sys.AddLink(spring_1);

    // Attach a visualization asset.
    spring_1->AddVisualShape(chrono_types::make_shared<ChSpringShape>(0.05, 80, 15));

    // Create a body suspended through a ChLinkTSDA (custom force functor)
    // -------------------------------------------------------------------

    auto body_2 = chrono_types::make_shared<ChBody>();
    sys.AddBody(body_2);
    body_2->SetPos(ChVector<>(1, -3, 0));
    body_2->SetIdentifier(1);
    body_2->SetBodyFixed(false);
    body_2->SetCollide(false);
    body_2->SetMass(1);
    body_2->SetInertiaXX(ChVector<>(1, 1, 1));

    // Attach a visualization asset.
    auto box_2 = chrono_types::make_shared<ChBoxShape>();
    box_2->GetBoxGeometry().SetLengths(ChVector<>(1, 1, 1));
    box_2->SetColor(ChColor(0, 0, 0.6f));
    body_2->AddVisualShape(box_2);

    // Create the spring between body_2 and ground. The spring end points are
    // specified in the body relative frames.
    auto force = chrono_types::make_shared<MySpringForce>();

    auto spring_2 = chrono_types::make_shared<ChLinkTSDA>();
    spring_2->Initialize(body_2, ground, true, ChVector<>(0, 0, 0), ChVector<>(1, 0, 0));
    spring_2->SetRestLength(rest_length);
    spring_2->RegisterForceFunctor(force);
    sys.AddLink(spring_2);

    // Attach a visualization asset.
    spring_2->AddVisualShape(chrono_types::make_shared<ChSpringShape>(0.05, 80, 15));

    // Create the Irrlicht application
    // -------------------------------

    // Create the Irrlicht visualization sys
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    sys.SetVisualSystem(vis);
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("ChLinkTSDA demo");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector<>(0, 0, 6));
    vis->AddTypicalLights();

    // Simulation loop
    int frame = 0;

    double timestep = 0.001;
    ChRealtimeStepTimer realtime_timer;
    while (vis->Run()) {
        vis->BeginScene();
        vis->DrawAll();
        vis->EndScene();

        if (frame % 50 == 0) {
            GetLog() << sys.GetChTime() << "  " << spring_1->GetLength() << "  " << spring_1->GetVelocity() << "  "
                     << spring_1->GetForce() << "\n";

            GetLog() << "            " << spring_2->GetLength() << "  " << spring_2->GetVelocity() << "  "
                     << spring_2->GetForce() << "\n\n";
        }

        sys.DoStepDynamics(timestep);
        realtime_timer.Spin(timestep);

        frame++;
    }

    return 0;
}
