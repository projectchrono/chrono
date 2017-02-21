// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
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

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/solver/ChSolverMINRES.h"

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

    auto material = std::make_shared<ChMaterialSurface>();
    material->SetFriction(0.4f);
    material->SetCompliance(0.001f / 700);                // as 1/K, in m/N. es: 1mm/700N
    material->SetComplianceT(material->GetCompliance());  // use tangential compliance as normal compliance
    material->SetDampingF(0.2f);                          // damping factor, 0...1
    material->SetRestitution(0.0);

    bool do_wall = false;
    bool do_stack = true;
    bool do_oddmass = true;
    bool do_spheres = true;
    bool do_heavyonside = true;

    double sphrad = 0.2;
    double dens = 1000;
    double sphmass = dens * (4. / 3.) * CH_C_PI * pow(sphrad, 3);
    double sphinertia = (2. / 5.) * sphmass * pow(sphrad, 2);

    if (do_stack) {
        int nbodies = 15;

        double totmass = 0;
        double level = 0;
        double sphrad_base = 0.2;
        double oddfactor = 100;

        for (int bi = 0; bi < nbodies; bi++)  // N. of vert. bricks
        {
            double sphrad = sphrad_base;
            if (do_oddmass && bi == (nbodies - 1))
                sphrad = sphrad * pow(oddfactor, 1. / 3.);
            double dens = 1000;

            std::shared_ptr<ChBody> mrigidBody;

            if (do_spheres) {
                mrigidBody = std::make_shared<ChBodyEasySphere>(sphrad,  // radius
                                                                dens,    // density
                                                                true,    // collide enable?
                                                                true);   // visualization?
                mrigidBody->SetPos(ChVector<>(0.5, sphrad + level, 0.7));
                mrigidBody->AddAsset(std::make_shared<ChTexture>(GetChronoDataFile("bluwhite.png")));

                application.GetSystem()->Add(mrigidBody);
            } else {
                mrigidBody = std::make_shared<ChBodyEasyBox>(sphrad, sphrad, sphrad,  // x,y,z size
                                                             dens,                    // density
                                                             true,                    // collide enable?
                                                             true);                   // visualization?
                mrigidBody->SetPos(ChVector<>(0.5, sphrad + level, 0.7));
                mrigidBody->AddAsset(std::make_shared<ChTexture>(GetChronoDataFile("cubetexture_bluwhite.png")));

                application.GetSystem()->Add(mrigidBody);
            }

            mrigidBody->SetMaterialSurface(material);

            mspheres.push_back(mrigidBody);

            level += sphrad * 2;
            totmass += mrigidBody->GetMass();
        }

        GetLog() << "Expected contact force at bottom F=" << (totmass * application.GetSystem()->Get_G_acc().y()) << "\n";
    }

    if (do_wall)
        for (int ai = 0; ai < 1; ai++)  // N. of walls
        {
            for (int bi = 0; bi < 10; bi++)  // N. of vert. bricks
            {
                for (int ui = 0; ui < 15; ui++)  // N. of hor. bricks
                {
                    auto mrigidWall = std::make_shared<ChBodyEasyBox>(0.396, 0.2, 0.4,  // size
                                                                      dens,             // density
                                                                      true,             // collide enable?
                                                                      true);            // visualization?
                    mrigidWall->SetPos(ChVector<>(-0.8 + ui * 0.4 + 0.2 * (bi % 2), 0.10 + bi * 0.2, -0.5 + ai * 0.6));
                    mrigidWall->AddAsset(std::make_shared<ChTexture>(GetChronoDataFile("cubetexture_bluwhite.png")));
                    mrigidWall->SetMaterialSurface(material);

                    application.GetSystem()->Add(mrigidWall);
                }
            }
        }

    if (do_heavyonside) {
        double sphrad = 0.2;
        double dens = 1000;
        double hfactor = 100;

        auto mrigidHeavy = std::make_shared<ChBodyEasySphere>(sphrad,          // radius
                                                              dens * hfactor,  // density
                                                              true,            // collide enable?
                                                              true);           // visualization?
        mrigidHeavy->SetPos(ChVector<>(0.5, sphrad + 0.1, -1));
        mrigidHeavy->AddAsset(std::make_shared<ChTexture>(GetChronoDataFile("pinkwhite.png")));
        mrigidHeavy->SetMaterialSurface(material);

        application.GetSystem()->Add(mrigidHeavy);

        GetLog() << "Expected contact deformation at side sphere="
                 << (mrigidHeavy->GetMass() * application.GetSystem()->Get_G_acc().y()) * material->GetCompliance() << "\n";
    }

    // Create the floor using a fixed rigid body of 'box' type:

    auto mrigidFloor = std::make_shared<ChBodyEasyBox>(50, 4, 50,  // radius
                                                       dens,       // density
                                                       true,       // collide enable?
                                                       true);      // visualization?
    mrigidFloor->SetPos(ChVector<>(0, -2, 0));
    mrigidFloor->SetBodyFixed(true);
    mrigidFloor->GetMaterialSurface()->SetFriction(0.6f);
    mrigidFloor->AddAsset(std::make_shared<ChTexture>(GetChronoDataFile("concrete.jpg")));

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
    // Create a ChronoENGINE physical system
    ChSystem mphysicalSystem;

    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(&mphysicalSystem, L"Critical cases for convergence, and compliance", core::dimension2d<u32>(800, 600),
                         false, true);

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    ChIrrWizard::add_typical_Logo(application.GetDevice());
    ChIrrWizard::add_typical_Sky(application.GetDevice());
    ChIrrWizard::add_typical_Lights(application.GetDevice());
    ChIrrWizard::add_typical_Camera(application.GetDevice(), core::vector3df(1, 2, 6), core::vector3df(0, 2, 0));

    // Create all the rigid bodies.

    create_items(application);


    // Use this function for adding a ChIrrNodeAsset to all already created items (ex. a floor, a wall, etc.)
    // Otherwise use application.AssetBind(myitem); on a per-item basis.
    application.AssetBindAll();
    application.AssetUpdateAll();

    // Modify some setting of the physical system for the simulation, if you want

    mphysicalSystem.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
    // mphysicalSystem.SetSolverType(ChSolver::Type::SOR);
    mphysicalSystem.SetMaxItersSolverSpeed(60);
    mphysicalSystem.SetMaxItersSolverStab(5);
    mphysicalSystem.SetParallelThreadNumber(1);

    mphysicalSystem.SetMaxPenetrationRecoverySpeed(10);

    application.SetTimestep(0.01);
    application.SetPaused(true);

    

    //
    // THE SOFT-REAL-TIME CYCLE
    //

    while (application.GetDevice()->run()) {
        application.GetVideoDriver()->beginScene(true, true, SColor(255, 140, 161, 192));

        application.DrawAll();

        align_spheres(application);  // just to simplify test, on y axis only

        application.DoStep();

        application.GetVideoDriver()->endScene();
    }

    return 0;
}
