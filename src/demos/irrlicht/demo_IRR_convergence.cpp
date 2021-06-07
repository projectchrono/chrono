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
//  Demonstration program for solver convergence with high stacks of objects.
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/assets/ChTexture.h"
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

// Static data used for this simple demo

std::vector<std::shared_ptr<ChBody> > mspheres;


void create_items(ChIrrAppInterface& application) {
    // Create some spheres in a vertical stack

    auto material = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    material->SetFriction(0.4f);
    material->SetCompliance(0.001f / 1200);               // as 1/K, in m/N. es: 1mm/1200N
    material->SetComplianceT(material->GetCompliance());  // use tangential compliance as normal compliance
    material->SetDampingF(0.1f);                          // damping factor, 0...1
    material->SetRestitution(0.0f);

    bool do_wall = false;
    bool do_stack = true;
    bool do_oddmass = true;
    bool do_spheres = true;
    bool do_heavyonside = true;

    double dens = 1000;

    if (do_stack) {
        int nbodies = 15;

        double totmass = 0;
        double level = 0;
        double sphrad_base = 0.2;
        double oddfactor = 100;

        for (int bi = 0; bi < nbodies; bi++) {
            double sphrad = sphrad_base;
            if (do_oddmass && bi == (nbodies - 1))
                sphrad = sphrad * pow(oddfactor, 1. / 3.);

            std::shared_ptr<ChBody> mrigidBody;

            if (do_spheres) {
                mrigidBody = chrono_types::make_shared<ChBodyEasySphere>(sphrad,     // radius
                                                                         dens,       // density
                                                                         true,       // visualization?
                                                                         true,       // collision?
                                                                         material);  // contact material
                mrigidBody->SetPos(ChVector<>(0.5, sphrad + level, 0.7));
                mrigidBody->AddAsset(chrono_types::make_shared<ChTexture>(GetChronoDataFile("textures/bluewhite.png")));

                application.GetSystem()->Add(mrigidBody);
            } else {
                mrigidBody = chrono_types::make_shared<ChBodyEasyBox>(sphrad, sphrad, sphrad,  // x,y,z size
                                                                      dens,                    // density
                                                                      true,                    // visualization?
                                                                      true,                    // collision?
                                                                      material);               // contact material
                mrigidBody->SetPos(ChVector<>(0.5, sphrad + level, 0.7));
                mrigidBody->AddAsset(
                    chrono_types::make_shared<ChTexture>(GetChronoDataFile("textures/cubetexture_bluewhite.png")));

                application.GetSystem()->Add(mrigidBody);
            }

            mspheres.push_back(mrigidBody);

            level += sphrad * 2;
            totmass += mrigidBody->GetMass();
        }

        GetLog() << "Expected contact force at bottom F=" << (totmass * application.GetSystem()->Get_G_acc().y()) << "\n";
    }

    if (do_wall) {
        for (int ai = 0; ai < 1; ai++) {                                                         // N. of walls
            for (int bi = 0; bi < 10; bi++) {                                                    // N. of vert. bricks
                for (int ui = 0; ui < 15; ui++) {                                                // N. of hor. bricks
                    auto mrigidWall = chrono_types::make_shared<ChBodyEasyBox>(0.396, 0.2, 0.4,  // size
                                                                               dens,             // density
                                                                               true,             // visualization?
                                                                               true,             // collision?
                                                                               material);        // contact material
                    mrigidWall->SetPos(ChVector<>(-0.8 + ui * 0.4 + 0.2 * (bi % 2), 0.10 + bi * 0.2, -0.5 + ai * 0.6));
                    mrigidWall->AddAsset(
                        chrono_types::make_shared<ChTexture>(GetChronoDataFile("textures/cubetexture_bluewhite.png")));

                    application.GetSystem()->Add(mrigidWall);
                }
            }
        }
    }

    if (do_heavyonside) {
        double sphrad = 0.2;
        double hfactor = 100;

        auto mrigidHeavy = chrono_types::make_shared<ChBodyEasySphere>(sphrad,          // radius
                                                                       dens * hfactor,  // density
                                                                       true,            // visualization?
                                                                       true,            // collision?
                                                                       material);       // contact material
        mrigidHeavy->SetPos(ChVector<>(0.5, sphrad + 0.6, -1));
        mrigidHeavy->AddAsset(chrono_types::make_shared<ChTexture>(GetChronoDataFile("textures/pinkwhite.png")));

        application.GetSystem()->Add(mrigidHeavy);

        GetLog() << "Expected contact deformation at side sphere="
                 << (mrigidHeavy->GetMass() * application.GetSystem()->Get_G_acc().y()) * material->GetCompliance()
                 << "\n";
    }

    // Create the floor using a fixed rigid body of 'box' type:

    auto mrigidFloor = chrono_types::make_shared<ChBodyEasyBox>(50, 4, 50,  // radius
                                                                dens,       // density
                                                                true,       // visualization?
                                                                true,       // collision?
                                                                material);  // contact material
    mrigidFloor->SetPos(ChVector<>(0, -2, 0));
    mrigidFloor->SetBodyFixed(true);
    mrigidFloor->AddAsset(chrono_types::make_shared<ChTexture>(GetChronoDataFile("textures/concrete.jpg")));

    application.GetSystem()->Add(mrigidFloor);
}

// Function that forces all spheres in the 'parent' level to be on the same vertical
// axis, without needing any constraint (for simplifying the solver benchmark).
// Also impose no rotation.

void align_spheres(ChIrrAppInterface& application) {
    for (unsigned int i = 0; i < mspheres.size(); ++i) {
        std::shared_ptr<ChBody> body = mspheres[i];
        ChVector<> mpos = body->GetPos();
        mpos.x() = 0.5;
        mpos.z() = 0.7;
        body->SetPos(mpos);
        body->SetRot(QUNIT);
    }
}

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create a ChronoENGINE physical system
    ChSystemNSC mphysicalSystem;

    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(&mphysicalSystem, L"Critical cases for convergence, and compliance", core::dimension2d<u32>(800, 600));
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(1, 2, 6), core::vector3df(0, 2, 0));

    // Create all the rigid bodies.
    create_items(application);

    // Use this function for adding a ChIrrNodeAsset to all already created items (ex. a floor, a wall, etc.)
    // Otherwise use application.AssetBind(myitem); on a per-item basis.
    application.AssetBindAll();
    application.AssetUpdateAll();

    // Modify some setting of the physical system for the simulation, if you want

    mphysicalSystem.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
    mphysicalSystem.SetSolverMaxIterations(60);

	// When using compliance, exp. for large compliances, the max. penetration recovery speed
	// also affects reaction forces, thus it must be deactivated (or used as a very large value)
    mphysicalSystem.SetMaxPenetrationRecoverySpeed(100000);

    application.SetTimestep(0.005);
    application.SetPaused(true);

    

    //
    // THE SOFT-REAL-TIME CYCLE
    //

    while (application.GetDevice()->run()) {
        application.BeginScene(true, true, SColor(255, 140, 161, 192));

        application.DrawAll();

        align_spheres(application);  // just to simplify test, on y axis only

        application.DoStep();

        application.EndScene();
    }

    return 0;
}
