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

#include "chrono/assets/ChPointPointDrawing.h"

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBody.h"

#include "chrono_irrlicht/ChIrrApp.h"

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
    virtual double operator()(double time,         // current time
                              double rest_length,  // undeformed length
                              double length,       // current length
                              double vel,          // current velocity (positive when extending)
                              ChLinkTSDA* link     // back-pointer to associated link
                              ) override {
        double force = -spring_coef * (length - rest_length) - damping_coef * vel;
        return force;
    }
};

// =============================================================================

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    ChSystemNSC system;
    system.Set_G_acc(ChVector<>(0, 0, 0));

    // Create the ground body with two visualization spheres
    // -----------------------------------------------------

    auto ground = chrono_types::make_shared<ChBody>();
    system.AddBody(ground);
    ground->SetIdentifier(-1);
    ground->SetBodyFixed(true);
    ground->SetCollide(false);

    {
        auto sph_1 = chrono_types::make_shared<ChSphereShape>();
        sph_1->GetSphereGeometry().rad = 0.1;
        sph_1->Pos = ChVector<>(-1, 0, 0);
        ground->AddAsset(sph_1);

        auto sph_2 = chrono_types::make_shared<ChSphereShape>();
        sph_2->GetSphereGeometry().rad = 0.1;
        sph_2->Pos = ChVector<>(1, 0, 0);
        ground->AddAsset(sph_2);
    }

    // Create a body suspended through a ChLinkTSDA (default linear)
    // -------------------------------------------------------------

    auto body_1 = chrono_types::make_shared<ChBody>();
    system.AddBody(body_1);
    body_1->SetPos(ChVector<>(-1, -3, 0));
    body_1->SetIdentifier(1);
    body_1->SetBodyFixed(false);
    body_1->SetCollide(false);
    body_1->SetMass(1);
    body_1->SetInertiaXX(ChVector<>(1, 1, 1));

    // Attach a visualization asset.
    auto box_1 = chrono_types::make_shared<ChBoxShape>();
    box_1->GetBoxGeometry().SetLengths(ChVector<>(1, 1, 1));
    body_1->AddAsset(box_1);
    auto col_1 = chrono_types::make_shared<ChColorAsset>();
    col_1->SetColor(ChColor(0.6f, 0, 0));
    body_1->AddAsset(col_1);

    // Create the spring between body_1 and ground. The spring end points are
    // specified in the body relative frames.
    auto spring_1 = chrono_types::make_shared<ChLinkTSDA>();
    spring_1->Initialize(body_1, ground, true, ChVector<>(0, 0, 0), ChVector<>(-1, 0, 0), false, rest_length);
    spring_1->SetSpringCoefficient(spring_coef);
    spring_1->SetDampingCoefficient(damping_coef);
    system.AddLink(spring_1);

    // Attach a visualization asset.
    spring_1->AddAsset(col_1);
    spring_1->AddAsset(chrono_types::make_shared<ChPointPointSpring>(0.05, 80, 15));

    // Create a body suspended through a ChLinkTSDA (custom force functor)
    // -------------------------------------------------------------------

    auto body_2 = chrono_types::make_shared<ChBody>();
    system.AddBody(body_2);
    body_2->SetPos(ChVector<>(1, -3, 0));
    body_2->SetIdentifier(1);
    body_2->SetBodyFixed(false);
    body_2->SetCollide(false);
    body_2->SetMass(1);
    body_2->SetInertiaXX(ChVector<>(1, 1, 1));

    // Attach a visualization asset.
    auto box_2 = chrono_types::make_shared<ChBoxShape>();
    box_2->GetBoxGeometry().SetLengths(ChVector<>(1, 1, 1));
    body_2->AddAsset(box_1);
    auto col_2 = chrono_types::make_shared<ChColorAsset>();
    col_2->SetColor(ChColor(0, 0, 0.6f));
    body_2->AddAsset(col_2);

    // Create the spring between body_2 and ground. The spring end points are
    // specified in the body relative frames.
    auto force = chrono_types::make_shared<MySpringForce>();

    auto spring_2 = chrono_types::make_shared<ChLinkTSDA>();
    spring_2->Initialize(body_2, ground, true, ChVector<>(0, 0, 0), ChVector<>(1, 0, 0), false, rest_length);
    spring_2->RegisterForceFunctor(force);
    system.AddLink(spring_2);

    // Attach a visualization asset.
    spring_2->AddAsset(col_2);
    spring_2->AddAsset(chrono_types::make_shared<ChPointPointSpring>(0.05, 80, 15));

    // Create the Irrlicht application
    // -------------------------------

    ChIrrApp application(&system, L"ChLinkTSDA demo", core::dimension2d<u32>(800, 600), false, true);
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(0, 0, 6));

    application.AssetBindAll();
    application.AssetUpdateAll();

    // Simulation loop
    int frame = 0;

    application.SetTimestep(0.001);

    GetLog().SetNumFormat("%10.3f");

    while (application.GetDevice()->run()) {
        application.BeginScene();

        application.DrawAll();

        application.DoStep();

        if (frame % 50 == 0) {
            GetLog() << system.GetChTime() << "  " << spring_1->GetLength() << "  " << spring_1->GetVelocity() << "  "
                     << spring_1->GetForce() << "\n";

            GetLog() << "            " << spring_2->GetLength() << "  " << spring_2->GetVelocity() << "  "
                     << spring_2->GetForce() << "\n\n";
        }

        frame++;

        application.EndScene();
    }

    return 0;
}
