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
// Authors: Alessandro Tasora
// =============================================================================
//
// Demo code about collisions of triangle meshes
//
// =============================================================================

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChParticlesClones.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"

#include "chrono_irrlicht/ChIrrApp.h"

// Use the namespaces of Chrono
using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::irrlicht;

// Use the main namespaces of Irrlicht
using namespace irr;
using namespace irr::core;
using namespace irr::scene;
using namespace irr::video;




int main(int argc, char* argv[]) {
    // Create a ChronoENGINE physical system
    ChSystem mphysicalSystem;

    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(&mphysicalSystem, L"Collisions between objects", core::dimension2d<u32>(1280, 720), false);

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    ChIrrWizard::add_typical_Logo(application.GetDevice());
    ChIrrWizard::add_typical_Sky(application.GetDevice());
    ChIrrWizard::add_typical_Lights(application.GetDevice());
    ChIrrWizard::add_typical_Camera(application.GetDevice(), core::vector3df(0, 1, -1));

    application.AddLightWithShadow(core::vector3df(1.5f, 5.5f, -2.5f), core::vector3df(0, 0, 0), 3, 2.2, 7.2, 40, 512,
                                   video::SColorf(0.8f, 0.8f, 1.0f));

    application.SetContactsDrawMode(ChIrrTools::eCh_ContactsDrawMode::CONTACT_DISTANCES);

    //
    // Create all the rigid bodies.
    // 

    collision::ChCollisionModel::SetDefaultSuggestedEnvelope(0.0025);
    collision::ChCollisionModel::SetDefaultSuggestedMargin(0.0025);

    // - Create a floor
    
    auto mfloor2 = std::make_shared<ChBody>();
    mfloor2->SetPos(ChVector<>(0, -1, 0));
    mfloor2->SetBodyFixed(true);
    mphysicalSystem.Add(mfloor2);

    ChTriangleMeshConnected mmeshbox;
    mmeshbox.LoadWavefrontMesh(GetChronoDataFile("cube.obj"),true,true);

    mfloor2->GetCollisionModel()->ClearModel();
    mfloor2->GetCollisionModel()->AddTriangleMesh(mmeshbox,false, false, VNULL, ChMatrix33<>(1), 0.005);
    mfloor2->GetCollisionModel()->BuildModel();
    mfloor2->SetCollide(true);

    auto masset_meshbox = std::make_shared<ChTriangleMeshShape>();
    masset_meshbox->SetMesh(mmeshbox);
    mfloor2->AddAsset(masset_meshbox);

    auto masset_texture = std::make_shared<ChTexture>();
    masset_texture->SetTextureFilename(GetChronoDataFile("concrete.jpg"));
    mfloor2->AddAsset(masset_texture);

    // - Create a falling item with triangle mesh shape

    ChTriangleMeshConnected mmesh;
    mmesh.LoadWavefrontMesh(GetChronoDataFile("shoe_view.obj"),false,true);
    mmesh.Transform(ChVector<>(-0.15,0,0), ChMatrix33<>(1.2)); // scale to a smaller cube
    mmesh.RepairDuplicateVertexes(1e-9); // if meshes are not watertight
    // compute mass inertia from mesh
    double mmass;
    ChVector<>mcog;
    ChMatrix33<> minertia;
    mmesh.ComputeMassProperties(true,mmass,mcog,minertia); 

    for (int j= 0; j<10;++j) {
        auto mfalling = std::make_shared<ChBody>();
        //  mfalling->SetPos(ChVector<>(0.1, 0.24, 0));
        mfalling->SetPos(ChVector<>(ChRandom() * 0.4, 0.2 + j * 0.12, ChRandom() * 0.4));
        mphysicalSystem.Add(mfalling);
        mfalling->SetMass(mmass*1000);
        mfalling->SetInertia(minertia*1000);

        mfalling->GetCollisionModel()->ClearModel();
        mfalling->GetCollisionModel()->AddTriangleMesh(mmesh,false, false, VNULL, ChMatrix33<>(1), 0.005);
        mfalling->GetCollisionModel()->BuildModel();
        mfalling->SetCollide(true);

        auto masset_mesh = std::make_shared<ChTriangleMeshShape>();
        masset_mesh->SetMesh(mmesh);
        masset_mesh->SetBackfaceCull(true);
        mfalling->AddAsset(masset_mesh);

        auto masset_texturew = std::make_shared<ChTexture>();
        masset_texturew->SetTextureFilename(GetChronoDataFile("cubetexture_wood.png"));
        mfalling->AddAsset(masset_texturew);
    }

    for (int bi = 0; bi < 20; bi++) {
        // Create a bunch of ChronoENGINE rigid bodies (spheres and
        // boxes etc.) which will fall..

        auto msphereBody = std::make_shared<ChBodyEasySphere>(0.05,             // radius size
                                                              1000,             // density
                                                              true,             // collide enable?
                                                              true);            // visualization?
        msphereBody->SetPos(ChVector<>(-0.5 + ChRandom() * 1, 1.4, -0.5 + ChRandom()));
        msphereBody->GetMaterialSurface()->SetFriction(0.2f);

        auto mballcolor = std::make_shared<ChColorAsset>();
        mballcolor->SetColor(ChColor(0.3f, 0.3f, 0.6f));
        msphereBody->AddAsset(mballcolor);

        mphysicalSystem.Add(msphereBody);
    }

    //-----------

    // Use this function for adding a ChIrrNodeAsset to all items
    // Otherwise use application.AssetBind(myitem); on a per-item basis.
    application.AssetBindAll();

    // Use this function for 'converting' assets into Irrlicht meshes
    application.AssetUpdateAll();

    // Use shadows in realtime view
    application.AddShadowAll();


    application.SetTimestep(0.005);

    //
    // THE SOFT-REAL-TIME CYCLE
    //

    while (application.GetDevice()->run()) {
        application.GetVideoDriver()->beginScene(true, true, SColor(255, 140, 161, 192));

        application.DrawAll();

        application.DoStep();

        application.GetVideoDriver()->endScene();
    }

    return 0;
}
