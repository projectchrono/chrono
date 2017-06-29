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

#include "chrono_irrlicht/ChBodySceneNode.h"
#include "chrono_irrlicht/ChBodySceneNodeTools.h"
#include "chrono_irrlicht/ChIrrApp.h"

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

std::shared_ptr<ChBody> create_wheel(ChVector<> mposition, ChIrrAppInterface& mapplication) {

    ChCollisionModel::SetDefaultSuggestedEnvelope(0.005);
    ChCollisionModel::SetDefaultSuggestedMargin(0.004);

    // create a basic rigid body, it comes with no visualization or collision shapes 
    auto mrigidBody = std::make_shared<ChBody>();
    mapplication.GetSystem()->Add(mrigidBody);
    mrigidBody->SetMass(50);
    mrigidBody->SetInertiaXX(ChVector<>(10, 10, 10));
    mrigidBody->SetPos(mposition);
    mrigidBody->GetMaterialSurfaceNSC()->SetFriction(0.5);

    // now attach a visualization shape, as a mesh from disk
    auto tireMesh = std::make_shared<ChObjShapeFile>();
    tireMesh->SetFilename(GetChronoDataFile("tractor_wheel.obj").c_str());
    mrigidBody->AddAsset(tireMesh);

    // now attach collision shape, as a compound of convex hulls (for each thread pair):

    // clear model. The colliding shape description MUST be between  ClearModel() .. BuildModel() pair.
    mrigidBody->GetCollisionModel()->ClearModel();
    // describe the (invisible) colliding shape by adding the 'carcass' decomposed shape and the
    // 'knobs'. Since these decompositions are only for 1/15th of the wheel, use for() to pattern them:
    for (double mangle = 0; mangle < 360.; mangle += (360. / 15.)) {
        ChQuaternion<> myrot;
        ChStreamInAsciiFile myknobs(GetChronoDataFile("tractor_wheel_knobs.chulls").c_str());
        ChStreamInAsciiFile myslice(GetChronoDataFile("tractor_wheel_slice.chulls").c_str());
        myrot.Q_from_AngAxis(mangle * (CH_C_PI / 180.), VECT_X);
        ChMatrix33<> mm(myrot);
        mrigidBody->GetCollisionModel()->AddConvexHullsFromFile(myknobs, ChVector<>(0, 0, 0), mm);
        mrigidBody->GetCollisionModel()->AddConvexHullsFromFile(myslice, ChVector<>(0, 0, 0), mm);
        // break;
    }
    // complete the description.
    mrigidBody->GetCollisionModel()->BuildModel();
    mrigidBody->SetCollide(true);

    return mrigidBody;
}

void create_some_falling_items(ChSystemNSC& mphysicalSystem, ISceneManager* msceneManager, IVideoDriver* driver) {
    // Make some pebbles, just for fun, under the wheel
    video::ITexture* cubeMap = driver->getTexture(GetChronoDataFile("concrete.jpg").c_str());
    video::ITexture* rockMap = driver->getTexture(GetChronoDataFile("rock.jpg").c_str());

    ChCollisionModel::SetDefaultSuggestedEnvelope(0.003);
    ChCollisionModel::SetDefaultSuggestedMargin(0.002);

    ChQuaternion<> rot;
    rot.Q_from_AngAxis(ChRandom() * CH_C_2PI, VECT_Y);

    double bed_x = 0.6;
    double bed_z = 1;

    int n_pebbles = 30;
    for (int bi = 0; bi < n_pebbles; bi++) {
        double sphrad = 0.02 + 0.02 * ChRandom();
        double sphdens = 1;
        double sphmass = (4 / 3) * CH_C_PI * pow(sphrad, 3) * sphdens;
        double sphinertia = pow(sphrad, 2) * sphmass;
        ChQuaternion<> randrot(ChRandom(), ChRandom(), ChRandom(), ChRandom());
        randrot.Normalize();

        auto mrigidBody = std::make_shared<ChBodyEasySphere>(sphrad, sphdens, true, true);
        mphysicalSystem.Add(mrigidBody);
        mrigidBody->SetRot(randrot);
        mrigidBody->SetPos(ChVector<>(-0.5 * bed_x + ChRandom() * bed_x, 
                            0.01 + 0.04 * ((double)bi / (double)n_pebbles),
                              -0.5 * bed_z + ChRandom() * bed_z));
        mrigidBody->GetMaterialSurfaceNSC()->SetFriction(0.4f);
    }

    // Create the a plane using body of 'box' type:
    auto mrigidBodyB = std::make_shared<ChBodyEasyBox>(10, 1, 10, 1000, true, true);
    mphysicalSystem.Add(mrigidBodyB);
    mrigidBodyB->SetBodyFixed(true);
    mrigidBodyB->SetPos(ChVector<>(0, -0.5, 0));
    mrigidBodyB->GetMaterialSurfaceNSC()->SetFriction(0.5);
    auto mcolor = std::make_shared<ChColorAsset>();
    mcolor->SetColor(ChColor(0.2f,0.2f,0.2f));
    mrigidBodyB->AddAsset(mcolor);
}

int main(int argc, char* argv[]) {
    // Create a ChronoENGINE physical system
    ChSystemNSC mphysicalSystem;

    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(&mphysicalSystem, L"Convex decomposed wheel", core::dimension2d<u32>(800, 600),
                                  false);

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    ChIrrWizard::add_typical_Logo(application.GetDevice());
    ChIrrWizard::add_typical_Sky(application.GetDevice());
    ChIrrWizard::add_typical_Lights(application.GetDevice());
    ChIrrWizard::add_typical_Camera(application.GetDevice(), core::vector3df(3.5f, 2.5f, -2.4f));

    // Create some debris

    create_some_falling_items(mphysicalSystem, application.GetSceneManager(), application.GetVideoDriver());

    // Create the wheel

    std::shared_ptr<ChBody> mwheelBody = create_wheel(ChVector<>(0, 1, 0), application);


    // Use this function for adding a ChIrrNodeAsset to all items
    // Otherwise use application.AssetBind(myitem); on a per-item basis.
    application.AssetBindAll();

    // Use this function for 'converting' assets into Irrlicht meshes
    application.AssetUpdateAll();


    //
    // THE SOFT-REAL-TIME CYCLE
    //

    // This will help choosing an integration step which matches the
    // real-time step of the simulation, if possible.

    int nstep = 0;

    application.SetStepManage(true);
    application.SetTimestep(0.01);
    application.SetTryRealtime(true);

    while (application.GetDevice()->run()) {
        application.BeginScene(true, true, SColor(255, 140, 161, 192));

        application.DrawAll();

        application.DoStep();

        application.EndScene();
    }

    return 0;
}
