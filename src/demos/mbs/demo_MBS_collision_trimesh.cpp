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
// Demo code about collisions of triangle meshes (NSC)
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChMassProperties.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/core/ChRandom.h"

#include "chrono/assets/ChVisualSystem.h"
#ifdef CHRONO_IRRLICHT
    #include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
using namespace chrono::irrlicht;
#endif
#ifdef CHRONO_VSG
    #include "chrono_vsg/ChVisualSystemVSG.h"
using namespace chrono::vsg3d;
#endif

using namespace chrono;

// -----------------------------------------------------------------------------

ChContactMethod contact_method = ChContactMethod::NSC;
ChCollisionSystem::Type coll_sys_type = ChCollisionSystem::Type::BULLET;
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// -----------------------------------------------------------------------------

std::shared_ptr<ChTriangleMeshConnected> CreateShoe() {
    auto mesh = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile("models/bulldozer/shoe_view.obj"),
                                                                 false, true);
    mesh->Transform(VNULL, ChMatrix33<>(1.2));
    mesh->RepairDuplicateVertexes(1e-9);

    return mesh;
}

std::shared_ptr<ChTriangleMeshConnected> CreateBox() {
    auto mesh = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile("models/cube.obj"),
                                                                 false, true);
    mesh->Transform(VNULL, ChMatrix33<>(ChVector3d(0.15, 0.05, 0.15)));
    mesh->RepairDuplicateVertexes(1e-9);

    return mesh;
}

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    ChRandom::SetSeed(0);

    // ----------------------
    // Create a Chrono system
    // ----------------------

    double step = 0;
    std::shared_ptr<ChSystem> sys;
    if (contact_method == ChContactMethod::NSC) {
        ChCollisionModel::SetDefaultSuggestedEnvelope(0.025);
        ChCollisionModel::SetDefaultSuggestedMargin(0.025);
        sys = chrono_types::make_shared<ChSystemNSC>();
        step = 5e-3;
    } else {
        // Note: collision envelope is set to 0 for SMC
        ChCollisionModel::SetDefaultSuggestedMargin(0.025);
        sys = chrono_types::make_shared<ChSystemSMC>();
        step = 1e-4;
    }

    sys->SetCollisionSystemType(coll_sys_type);

    // -----------------
    // Create floor body
    // -----------------

    ChContactMaterialData floor_mat_data;
    auto floor_mat = floor_mat_data.CreateMaterial(contact_method);

    auto floor = chrono_types::make_shared<ChBodyEasyBox>(5, 0.2, 5, 1000, true, true, floor_mat);
    floor->SetPos(ChVector3d(0, -0.1, 0));
    floor->SetFixed(true);
    floor->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/pinkwhite.png"), 50, 50);

    sys->Add(floor);

    // ------------------------------------------------------------------------
    // Create a contact material, a visualization shape, and a collision shape,
    // shared among all bodies
    // ------------------------------------------------------------------------

    ChContactMaterialData body_mat_data;
    auto body_mat = body_mat_data.CreateMaterial(contact_method);

    // read mesh from file, scale, and repair
    auto mesh = CreateShoe();

    // compute mass properties from mesh
    double mass;
    ChVector3d cog;
    ChMatrix33<> inertia;
    double density = 1000;
    mesh->ComputeMassProperties(true, mass, cog, inertia);
    ChMatrix33<> principal_inertia_rot;
    ChVector3d principal_I;
    ChInertiaUtils::PrincipalInertia(inertia, principal_I, principal_inertia_rot);

    // visualization shape
    auto vis_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    vis_shape->SetMesh(mesh);
    vis_shape->SetColor(ChColor(0.25f, 0.5f, 0.25f));
    vis_shape->SetBackfaceCull(true);
    auto vis_model = chrono_types::make_shared<ChVisualModel>();
    vis_model->AddShape(vis_shape);

    // collision shape
    auto coll_shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(body_mat, mesh, false, false, 0.005);

    // ---------------------
    // Create falling bodies
    // ---------------------

    int num_bodies = 15;
    for (int j = 0; j < num_bodies; ++j) {
        auto falling = chrono_types::make_shared<ChBodyAuxRef>();
        falling->SetFixed(false);

        // set the COM coordinates to barycenter, without displacing the reference frame, and
        // make the COM frame a principal frame
        falling->SetFrameCOMToRef(ChFrame<>(cog, principal_inertia_rot));

        // set mass and inertia
        falling->SetMass(mass * density);
        falling->SetInertiaXX(density * principal_I);

        // set the absolute position of the body
        int layer = j % 3;
        int ix = (j / 3) % 3;
        int iz = (j / 3) / 3;
        double x = -0.5 + ix * 0.5 + layer * 0.1;
        double z = -0.5 + iz * 0.5 + layer * 0.1;
        double y = 0.25 + layer * 0.25;

        auto rot = QuatFromAngleX(ChRandom::Get() * 0.3) *  //
                   QuatFromAngleZ(ChRandom::Get() * 0.3) *  //
                   QuatFromAngleZ(ChRandom::Get() * 0.3);

        falling->SetFrameRefToAbs(ChFrame<>(ChVector3d(x, y, z), rot));

        // attach visualization and collision shapes
        falling->AddVisualModel(vis_model);
        falling->AddCollisionShape(coll_shape);
        falling->EnableCollision(true);

        sys->Add(falling);
    }

    // -------------------------------
    // Create the visualization window
    // -------------------------------

#ifndef CHRONO_IRRLICHT
    if (vis_type == ChVisualSystem::Type::IRRLICHT)
        vis_type = ChVisualSystem::Type::VSG;
#endif
#ifndef CHRONO_VSG
    if (vis_type == ChVisualSystem::Type::VSG)
        vis_type = ChVisualSystem::Type::IRRLICHT;
#endif

    std::shared_ptr<ChVisualSystem> vis;
    switch (vis_type) {
        case ChVisualSystem::Type::IRRLICHT: {
#ifdef CHRONO_IRRLICHT
            auto vis_irr = chrono_types::make_shared<ChVisualSystemIrrlicht>();
            vis_irr->AttachSystem(sys.get());
            vis_irr->SetWindowSize(1280, 800);
            vis_irr->SetWindowTitle("Collisions between objects");
            vis_irr->SetCameraVertical(CameraVerticalDir::Y);
            vis_irr->Initialize();
            vis_irr->AddLogo();
            vis_irr->AddCamera(ChVector3d(1, 0.5, -1));
            vis_irr->AddTypicalLights();

            vis = vis_irr;
#endif
            break;
        }
        default:
        case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
            auto vis_vsg = chrono_types::make_shared<ChVisualSystemVSG>();
            vis_vsg->AttachSystem(sys.get());
            vis_vsg->SetWindowTitle("Collisions between objects");
            vis_vsg->AddCamera(ChVector3d(-2.5, 1.0, 2.5));
            vis_vsg->SetWindowSize(1280, 800);
            vis_vsg->SetWindowPosition(100, 100);
            vis_vsg->SetBackgroundColor(ChColor(0.8f, 0.85f, 0.9f));
            vis_vsg->SetCameraVertical(CameraVerticalDir::Y);
            vis_vsg->SetCameraAngleDeg(40.0);
            vis_vsg->SetLightIntensity(1.0f);
            vis_vsg->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
            vis_vsg->EnableShadows();
            vis_vsg->Initialize();

            vis = vis_vsg;
#endif
            break;
        }
    }

    // ---------------
    // Simulation loop
    // ---------------

    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        vis->EndScene();
        sys->DoStepDynamics(step);
    }

    return 0;
}
