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
// Demo code about loading a .chulls file, with xyz points of cluters of convex
// hulls that define a complicate concave shape. The shape is a wheel for
// tractors, with large knobs, that has been decomposed using demo_decomposition
// from .obj shape to a .chull.
//
// =============================================================================

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

#include <irrlicht.h>

// Use the namespaces of Chrono
using namespace chrono;
using namespace chrono::collision;
using namespace chrono::irrlicht;

// Use the main namespaces of Irrlicht
using namespace irr;
using namespace irr::core;
using namespace irr::scene;
using namespace irr::video;
using namespace irr::io;
using namespace irr::gui;

std::shared_ptr<ChBody> create_wheel(ChVector<> mposition, ChSystem& sys) {
    ChCollisionModel::SetDefaultSuggestedEnvelope(0.005);
    ChCollisionModel::SetDefaultSuggestedMargin(0.004);

    // create a basic rigid body, it comes with no visualization or collision shapes
    auto mrigidBody = chrono_types::make_shared<ChBody>();
    sys.Add(mrigidBody);
    mrigidBody->SetMass(50);
    mrigidBody->SetInertiaXX(ChVector<>(10, 10, 10));
    mrigidBody->SetPos(mposition);

    // now attach a visualization shape, as a mesh from disk
    auto tireMesh = chrono_types::make_shared<ChObjShapeFile>();
    tireMesh->SetFilename(GetChronoDataFile("models/tractor_wheel/tractor_wheel.obj").c_str());
    mrigidBody->AddVisualShape(tireMesh);

    // contact material shared by all collision shapes of the wheel
    auto mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    mat->SetFriction(0.5f);

    // now attach collision shape, as a compound of convex hulls (for each thread pair):
    mrigidBody->GetCollisionModel()->ClearModel();
    // describe the (invisible) colliding shape by adding the 'carcass' decomposed shape and the
    // 'knobs'. Since these decompositions are only for 1/15th of the wheel, use for() to pattern them:
    for (double mangle = 0; mangle < 360.; mangle += (360. / 15.)) {
        ChQuaternion<> myrot;
        ChStreamInAsciiFile myknobs(GetChronoDataFile("models/tractor_wheel/tractor_wheel_knobs.chulls").c_str());
        ChStreamInAsciiFile myslice(GetChronoDataFile("models/tractor_wheel/tractor_wheel_slice.chulls").c_str());
        myrot.Q_from_AngAxis(mangle * (CH_C_PI / 180.), VECT_X);
        ChMatrix33<> mm(myrot);
        mrigidBody->GetCollisionModel()->AddConvexHullsFromFile(mat, myknobs, ChVector<>(0, 0, 0), mm);
        mrigidBody->GetCollisionModel()->AddConvexHullsFromFile(mat, myslice, ChVector<>(0, 0, 0), mm);
        // break;
    }
    // complete the description.
    mrigidBody->GetCollisionModel()->BuildModel();
    mrigidBody->SetCollide(true);

    return mrigidBody;
}

void create_some_falling_items(ChSystemNSC& sys) {
    ChCollisionModel::SetDefaultSuggestedEnvelope(0.003);
    ChCollisionModel::SetDefaultSuggestedMargin(0.002);

    ChQuaternion<> rot;
    rot.Q_from_AngAxis(ChRandom() * CH_C_2PI, VECT_Y);

    auto pebble_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    pebble_mat->SetFriction(0.4f);

    double bed_x = 0.6;
    double bed_z = 1;

    int n_pebbles = 30;
    for (int bi = 0; bi < n_pebbles; bi++) {
        double sphrad = 0.02 + 0.02 * ChRandom();
        double sphdens = 1;
        ChQuaternion<> randrot(ChRandom(), ChRandom(), ChRandom(), ChRandom());
        randrot.Normalize();

        auto mrigidBody = chrono_types::make_shared<ChBodyEasySphere>(sphrad, sphdens, true, true, pebble_mat);
        sys.Add(mrigidBody);
        mrigidBody->SetRot(randrot);
        mrigidBody->SetPos(ChVector<>(-0.5 * bed_x + ChRandom() * bed_x, 0.01 + 0.04 * ((double)bi / (double)n_pebbles),
                                      -0.5 * bed_z + ChRandom() * bed_z));
    }

    // Create the a plane using body of 'box' type:
    auto ground_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    ground_mat->SetFriction(0.5f);

    auto mrigidBodyB = chrono_types::make_shared<ChBodyEasyBox>(10, 1, 10, 1000, true, true, ground_mat);
    sys.Add(mrigidBodyB);
    mrigidBodyB->SetBodyFixed(true);
    mrigidBodyB->SetPos(ChVector<>(0, -0.5, 0));
}

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create a ChronoENGINE physical system
    ChSystemNSC sys;

    // Create some debris

    create_some_falling_items(sys);

    // Create the wheel

    std::shared_ptr<ChBody> mwheelBody = create_wheel(ChVector<>(0, 1, 0), sys);

    // Create the Irrlicht visualization sys
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    sys.SetVisualSystem(vis);
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("Convex decomposed wheel");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector<>(3.5, 2.5, -2.4));
    vis->AddTypicalLights();

    // Simulation loop
    double timestep = 0.01;
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
