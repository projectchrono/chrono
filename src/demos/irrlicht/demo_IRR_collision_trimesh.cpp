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
// Demo code about collisions of triangle meshes
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChInertiaUtils.h"
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
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create a ChronoENGINE physical system
    ChSystemNSC mphysicalSystem;

    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(&mphysicalSystem, L"Collisions between objects", core::dimension2d<u32>(1280, 720), VerticalDir::Y, false, true);
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(irr::core::vector3df(0, 1, -1));
    application.AddLightWithShadow(core::vector3df(1.5f, 5.5f, -2.5f), core::vector3df(0, 0, 0), 3, 2.2, 7.2, 40, 512,
                                   video::SColorf(0.8f, 0.8f, 1.0f));
    application.SetContactsDrawMode(IrrContactsDrawMode::CONTACT_DISTANCES);

    //
    // Create all the rigid bodies.
    // 

	collision::ChCollisionModel::SetDefaultSuggestedEnvelope(0.0025);
	collision::ChCollisionModel::SetDefaultSuggestedMargin(0.0025);

    // - Create a floor

    auto floor_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    auto mfloor2 = chrono_types::make_shared<ChBodyEasyBox>(5, 2, 5, 1000, true, true, floor_mat);
    
	//auto mfloor2 = chrono_types::make_shared<ChBody>();
    mfloor2->SetPos(ChVector<>(0, -1, 0));
    mfloor2->SetBodyFixed(true);
    mphysicalSystem.Add(mfloor2);

    auto masset_texture = chrono_types::make_shared<ChTexture>();
    masset_texture->SetTextureFilename(GetChronoDataFile("textures/concrete.jpg"));
    mfloor2->AddAsset(masset_texture);

    // - Create a falling item with triangle mesh shape

    // Shared contact material for all meshes
    auto mesh_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();

    // Note: one can create easily a colliding shape using the following
    // piece of code:
    //
    // auto mfalling = chrono_types::make_shared<ChBodyEasyMesh>(
    //	  GetChronoDataFile("models/bulldozer/shoe_view.obj"),  // file name for OBJ Wavefront mesh
    //	  1000,                                                 // density of the body
    //	  true,			                                        // automatic evaluation of mass, COG position, inertia tensor
    //    true,                                                 // attach visualization asset
    //	  true,			                                        // enable the collision detection
    //    mat,                                                  // surface contact material
    //	  0.005			                                        // radius of 'inflating' of mesh (for more robust collision detection)
    //	  );
    // mfalling->SetFrame_REF_to_abs(ChFrame<>(ChVector<>(-0.9 + ChRandom() * 1.4, 0.4 + j * 0.12, -0.9 + ChRandom()
    // * 1.4))); mphysicalSystem.Add(mfalling);
    //
    // but here we want to show a more low-level control of this process, for
    // various reasons, for example: we want to share a single ChTriangleMeshConnected 
	// between 15 falling shapes; also we want to call RepairDuplicateVertexes() on the
	// imported mesh; also we want to scale the imported mesh using Transform().

    auto mmesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    mmesh->LoadWavefrontMesh(GetChronoDataFile("models/bulldozer/shoe_view.obj"), false, true);
    mmesh->Transform(ChVector<>(0, 0, 0), ChMatrix33<>(1.2));  // scale to a different size
    mmesh->RepairDuplicateVertexes(1e-9);                      // if meshes are not watertight

    // compute mass inertia from mesh
    double mmass;
    ChVector<> mcog;
    ChMatrix33<> minertia;
    double mdensity = 1000;
    mmesh->ComputeMassProperties(true, mmass, mcog, minertia);
    ChMatrix33<> principal_inertia_rot;
    ChVector<> principal_I;
    ChInertiaUtils::PrincipalInertia(minertia, principal_I, principal_inertia_rot);

    for (int j= 0; j<15;++j) {
		
        auto mfalling = chrono_types::make_shared<ChBodyAuxRef>();

        // Set the COG coordinates to barycenter, without displacing the REF reference.
        // Make the COG frame a principal frame.
        mfalling->SetFrame_COG_to_REF(ChFrame<>(mcog, principal_inertia_rot));

        // Set inertia
        mfalling->SetMass(mmass * mdensity);
        mfalling->SetInertiaXX(mdensity * principal_I);

		// Set the absolute position of the body:
        mfalling->SetFrame_REF_to_abs(ChFrame<>(ChVector<>(-0.9+ChRandom() * 1.4, 0.4 + j * 0.12, -0.9+ChRandom() * 1.4)));
        mphysicalSystem.Add(mfalling);

        mfalling->GetCollisionModel()->ClearModel();
        mfalling->GetCollisionModel()->AddTriangleMesh(mesh_mat, mmesh, false, false, VNULL, ChMatrix33<>(1), 0.005);
        mfalling->GetCollisionModel()->BuildModel();
        mfalling->SetCollide(true);

        auto masset_mesh = chrono_types::make_shared<ChTriangleMeshShape>();
        masset_mesh->SetMesh(mmesh);
        masset_mesh->SetBackfaceCull(true);
        mfalling->AddAsset(masset_mesh);
    }

    // Shared contact material for falling objects
    auto obj_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    obj_mat->SetFriction(0.2f);

    // Create a falling rigid bodies
    for (int bi = 0; bi < 20; bi++) {
        auto msphereBody = chrono_types::make_shared<ChBodyEasySphere>(0.05,      // radius size
                                                                       1000,      // density
                                                                       true,      // visualization?
                                                                       true,      // collision?
                                                                       obj_mat);  // contact material
        msphereBody->SetPos(ChVector<>(-0.5 + ChRandom() * 1, 1.4, -0.5 + ChRandom()));

        auto mballcolor = chrono_types::make_shared<ChColorAsset>();
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
        application.BeginScene(true, true, SColor(255, 140, 161, 192));

        application.DrawAll();

        application.DoStep();

        application.EndScene();
    }

    return 0;
}
