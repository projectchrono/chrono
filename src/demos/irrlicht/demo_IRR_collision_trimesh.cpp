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
// Demo code about collisions of triangle meshes
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChInertiaUtils.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

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

    // Create a Chrono physical system
    ChSystemNSC sys;

    // Create all the rigid bodies.

    collision::ChCollisionModel::SetDefaultSuggestedEnvelope(0.0025);
    collision::ChCollisionModel::SetDefaultSuggestedMargin(0.0025);

    // - Create a floor

    auto floor_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    auto floor = chrono_types::make_shared<ChBodyEasyBox>(5, 2, 5, 1000, true, true, floor_mat);
    floor->SetPos(ChVector<>(0, -1, 0));
    floor->SetBodyFixed(true);
    floor->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/concrete.jpg"));
    sys.Add(floor);

    // - Create a falling item with triangle mesh shape

    // Shared contact material for all meshes
    auto mesh_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();

    // Note: one can create easily a colliding shape using the following
    // piece of code:
    //
    // auto falling = chrono_types::make_shared<ChBodyEasyMesh>(
    //	  GetChronoDataFile("models/bulldozer/shoe_view.obj"),  // file name for OBJ Wavefront mesh
    //	  1000,                                                 // density of the body
    //	  true,			                                        // automatic evaluation of mass, COG position, inertia
    // tensor
    //    true,                                                 // attach visualization asset
    //	  true,			                                        // enable the collision detection
    //    mat,                                                  // surface contact material
    //	  0.005			                                        // radius of 'inflating' of mesh (for more robust
    //collision detection)
    //	  );
    // falling->SetFrame_REF_to_abs(ChFrame<>(ChVector<>(-0.9 + ChRandom() * 1.4, 0.4 + j * 0.12, -0.9 + ChRandom()
    // * 1.4))); sys.Add(falling);
    //
    // but here we want to show a more low-level control of this process, for
    // various reasons, for example: we want to share a single ChTriangleMeshConnected
    // between 15 falling shapes; also we want to call RepairDuplicateVertexes() on the
    // imported mesh; also we want to scale the imported mesh using Transform().

    auto mesh = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile("models/bulldozer/shoe_view.obj"),
                                                                 false, true);
    mesh->Transform(ChVector<>(0, 0, 0), ChMatrix33<>(1.2));  // scale to a different size
    mesh->RepairDuplicateVertexes(1e-9);                      // if meshes are not watertight

    // compute mass inertia from mesh
    double mass;
    ChVector<> cog;
    ChMatrix33<> inertia;
    double density = 1000;
    mesh->ComputeMassProperties(true, mass, cog, inertia);
    ChMatrix33<> principal_inertia_rot;
    ChVector<> principal_I;
    ChInertiaUtils::PrincipalInertia(inertia, principal_I, principal_inertia_rot);

    // Create a shared visual model containing a visualizatoin mesh
    auto mesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
    mesh_shape->SetMesh(mesh);
    mesh_shape->SetMutable(false);
    mesh_shape->SetColor(ChColor(1.0f, 0.5f, 0.5f));
    mesh_shape->SetBackfaceCull(true);

    auto vis_model = chrono_types::make_shared<ChVisualModel>();
    vis_model->AddShape(mesh_shape);

    for (int j = 0; j < 15; ++j) {
        auto falling = chrono_types::make_shared<ChBodyAuxRef>();

        // Set the COG coordinates to barycenter, without displacing the REF reference.
        // Make the COG frame a principal frame.
        falling->SetFrame_COG_to_REF(ChFrame<>(cog, principal_inertia_rot));

        // Set inertia
        falling->SetMass(mass * density);
        falling->SetInertiaXX(density * principal_I);

        // Set the absolute position of the body:
        falling->SetFrame_REF_to_abs(
            ChFrame<>(ChVector<>(-0.9 + ChRandom() * 1.4, 0.4 + j * 0.12, -0.9 + ChRandom() * 1.4)));
        sys.Add(falling);

        falling->GetCollisionModel()->ClearModel();
        falling->GetCollisionModel()->AddTriangleMesh(mesh_mat, mesh, false, false, VNULL, ChMatrix33<>(1), 0.005);
        falling->GetCollisionModel()->BuildModel();
        falling->SetCollide(true);

        falling->AddVisualModel(vis_model);
    }

    // Shared contact material for falling objects
    auto obj_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    obj_mat->SetFriction(0.2f);

    // Create a falling rigid bodies
    for (int bi = 0; bi < 20; bi++) {
        auto sphereBody = chrono_types::make_shared<ChBodyEasySphere>(0.05,      // radius size
                                                                      1000,      // density
                                                                      true,      // visualization?
                                                                      true,      // collision?
                                                                      obj_mat);  // contact material
        sphereBody->SetPos(ChVector<>(-0.5 + ChRandom() * 1, 1.4, -0.5 + ChRandom()));
        sphereBody->GetVisualShape(0)->SetColor(ChColor(0.3f, 0.3f, 0.6f));
        sys.Add(sphereBody);
    }

    // Create the Irrlicht visualization system
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    sys.SetVisualSystem(vis);
    vis->SetWindowSize(1280, 720);
    vis->SetWindowTitle("Collisions between objects");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector<>(0, 1, -1));
    vis->AddTypicalLights();
    vis->AddLightWithShadow(ChVector<>(1.5, 5.5, -2.5), ChVector<>(0, 0, 0), 3, 2.2, 7.2, 40, 512,
                           ChColor(0.8f, 0.8f, 1.0f));
    vis->EnableShadows();

    ////application.SetContactsDrawMode(IrrContactsDrawMode::CONTACT_DISTANCES);

    // Simulation loop
    while (vis->Run()) {
        vis->BeginScene(true, true, ChColor(0.55, 0.63, 0.75));
        vis->DrawAll();
        vis->EndScene();
        sys.DoStepDynamics(0.005);
    }

    return 0;
}
