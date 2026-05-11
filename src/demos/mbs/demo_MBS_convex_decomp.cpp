// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Dario Fusai
// =============================================================================
//
// Demo code to illustrate various trimesh convex decomposition methods.
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/collision/bullet/ChCollisionUtilsBullet.h"
#include "chrono/collision/ChConvexDecomposition.h"
#include "chrono/core/ChRandom.h"
#include "chrono/core/ChTimer.h"

#ifdef CHRONO_IRRLICHT
    #include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
using namespace chrono::irrlicht;
#endif
#ifdef CHRONO_VSG
    #include "chrono_vsg/ChVisualSystemVSG.h"
using namespace chrono::vsg3d;
#endif

using namespace chrono;

ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2026 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Create a Chrono physical system -----------------------------------------
    ChSystemNSC sys;
    sys.SetGravitationalAcceleration(VNULL);
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    ChTimer timer;

    // Load mesh model
    auto trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile("models/support.obj"));

    // Bullet convex hull ------------------------------------------------------
    {
        auto body = chrono_types::make_shared<ChBody>();
        body->SetPos({-1, 0, 0});
        body->AddVisualShape(chrono_types::make_shared<ChVisualShapeTriangleMesh>(trimesh));
        sys.Add(body);

        auto trimesh_chull = chrono_types::make_shared<ChTriangleMeshConnected>();
        std::cout << "Computing Bullet convex hull... ";
        timer.start(true);
        bt_utils::ChConvexHullLibraryWrapper::ComputeHull(trimesh->GetCoordsVertices(), *trimesh_chull);
        std::cout << " completed in " << timer.GetTimeMilliseconds() << " ms\n";

        auto vshape = chrono_types::make_shared<ChVisualShapeTriangleMesh>(trimesh_chull);
        vshape->SetColor({1.f, 0.f, 0.f});
        body->AddVisualShape(vshape);
    }

    // HACD convex decomposition -----------------------------------------------
    {
        auto body = chrono_types::make_shared<ChBody>();
        body->SetPos({0, 0, 0});
        body->AddVisualShape(chrono_types::make_shared<ChVisualShapeTriangleMesh>(trimesh));
        sys.Add(body);

        ChConvexDecompositionHACD hacd;
        hacd.Reset();
        hacd.AddTriangleMesh(*trimesh);
        hacd.SetParameters(2, 0, 0.25, false, false, 100.0, 30.0, 0.0, 0.1, 32);
        std::cout << "Computing HACD convex decomposition... ";
        timer.start(true);
        unsigned int hull_count = hacd.ComputeConvexDecomposition();
        std::cout << " completed in " << timer.GetTimeMilliseconds() << " ms\n";
        
        for (unsigned int i = 0; i < hull_count; i++) {
            auto chull_mesh = chrono_types::make_shared<ChTriangleMeshConnected>();
            hacd.GetConvexHullResult(i, *chull_mesh);

            auto vshape = chrono_types::make_shared<ChVisualShapeTriangleMesh>(chull_mesh, false);
            vshape->SetColor(ChColor(ChRandom::Get(), ChRandom::Get(), ChRandom::Get()));
            body->AddVisualShape(vshape);
        }
    }

    // HACDv2 convex decomposition ---------------------------------------------
    {
        auto body = chrono_types::make_shared<ChBody>();
        body->SetPos({1, 0, 0});
        body->AddVisualShape(chrono_types::make_shared<ChVisualShapeTriangleMesh>(trimesh));
        sys.Add(body);

        ChConvexDecompositionHACDv2 hacdv2;
        hacdv2.Reset();
        hacdv2.AddTriangleMesh(*trimesh);
        hacdv2.SetParameters(256, 256, 32, 0.2f, 0.0f, 1e-9f, false);
        std::cout << "Computing HACDv2 convex decomposition... ";
        timer.start(true);
        unsigned int hull_count = hacdv2.ComputeConvexDecomposition();
        std::cout << " completed in " << timer.GetTimeMilliseconds() << " ms\n";
        
        for (unsigned int i = 0; i < hull_count; i++) {
            auto chull_mesh = chrono_types::make_shared<ChTriangleMeshConnected>();
            hacdv2.GetConvexHullResult(i, *chull_mesh);

            auto vshape = chrono_types::make_shared<ChVisualShapeTriangleMesh>(chull_mesh, false);
            vshape->SetColor(ChColor(ChRandom::Get(), ChRandom::Get(), ChRandom::Get()));
            body->AddVisualShape(vshape);
        }
    }

    // VHACD convex decomposition ----------------------------------------------
    {
        auto body = chrono_types::make_shared<ChBody>();
        body->SetPos({2, 0, 0});
        body->AddVisualShape(chrono_types::make_shared<ChVisualShapeTriangleMesh>(trimesh));
        sys.Add(body);

        ChConvexDecompositionVHACD vhacd;
        vhacd.Reset();
        vhacd.AddTriangleMesh(*trimesh);
        vhacd.SetParameters(64, 32, 1000, 5.0, 10, false);
        std::cout << "Computing VHACD convex decomposition... ";
        timer.start(true);
        unsigned int hull_count = vhacd.ComputeConvexDecomposition();
        std::cout << " completed in " << timer.GetTimeMilliseconds() << " ms\n";
        
        for (unsigned int i = 0; i < hull_count; i++) {
            auto chull_mesh = chrono_types::make_shared<ChTriangleMeshConnected>();
            vhacd.GetConvexHullResult(i, *chull_mesh);

            auto vshape = chrono_types::make_shared<ChVisualShapeTriangleMesh>(chull_mesh, false);
            vshape->SetColor(ChColor(ChRandom::Get(), ChRandom::Get(), ChRandom::Get()));
            body->AddVisualShape(vshape);
        }
    }

    // Create the run-time visualization system --------------------------------
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
            vis_irr->AttachSystem(&sys);
            vis_irr->SetWindowSize(800, 600);
            vis_irr->SetWindowTitle("Convex decomposition");
            vis_irr->Initialize();
            vis_irr->AddLogo();
            vis_irr->AddSkyBox();
            vis_irr->AddCamera(ChVector3d(0, 1, 3));
            vis_irr->AddTypicalLights();
            vis_irr->EnableShadows();

            vis = vis_irr;
#endif
            break;
        }
        default:
        case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
            auto vis_vsg = chrono_types::make_shared<ChVisualSystemVSG>();
            vis_vsg->AttachSystem(&sys);
            vis_vsg->SetCameraVertical(CameraVerticalDir::Y);
            vis_vsg->SetWindowSize(1280, 800);
            vis_vsg->SetWindowPosition(100, 100);
            vis_vsg->SetWindowTitle("Convex decomposition");
            vis_vsg->EnableSkyTexture(SkyMode::BOX);
            vis_vsg->AddCamera(ChVector3d(0, 1, 3));
            vis_vsg->SetCameraAngleDeg(50);
            vis_vsg->SetLightIntensity(1.0f);
            vis_vsg->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
            vis_vsg->EnableShadows();
            vis_vsg->Initialize();

            vis = vis_vsg;
#endif
            break;
        }
    }

    // Simulation loop ---------------------------------------------------------
    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        vis->EndScene();
    }

    return 0;
}
