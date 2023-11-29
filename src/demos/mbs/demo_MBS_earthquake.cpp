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
//   Demo code about
//     - ChEasyBody objects
//     - collisions and contacts
//     - imposing a ground-relative motion to a body
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/motion_functions/ChFunction_Sine.h"
#include "chrono/core/ChRealtimeStep.h"

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

ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// Utility function. Create a tapered column as a faceted convex hull.
// For convex hulls, you just need to build a vector of points, it does not matter the order,
// because they will be considered 'wrapped' in a convex hull anyway.
void create_column(ChSystemNSC& sys,
                   ChCoordsys<> base_pos,
                   std::shared_ptr<ChMaterialSurface> material,
                   int col_nedges = 10,
                   double col_radius_hi = 0.45,
                   double col_radius_lo = 0.5,
                   double col_height = 3,
                   double col_density = 3000) {
    double col_base = 0;

    std::vector<ChVector<> > mpoints;
    for (int i = 0; i < col_nedges; ++i) {
        double alpha = CH_C_2PI * ((double)i / (double)col_nedges);  // polar coord
        double x = col_radius_hi * cos(alpha);
        double z = col_radius_hi * sin(alpha);
        double y = col_base + col_height;
        mpoints.push_back(ChVector<>(x, y, z));
    }
    for (int i = 0; i < col_nedges; ++i) {
        double alpha = CH_C_2PI * ((double)i / (double)col_nedges);  // polar coord
        double x = col_radius_lo * cos(alpha);
        double z = col_radius_lo * sin(alpha);
        double y = col_base;
        mpoints.push_back(ChVector<>(x, y, z));
    }
    auto bodyColumn = chrono_types::make_shared<ChBodyEasyConvexHull>(mpoints, col_density, true, true, material);
    ChCoordsys<> cog_column(ChVector<>(0, col_base + col_height / 2, 0));
    ChCoordsys<> abs_cog_column = cog_column >> base_pos;
    bodyColumn->SetCoord(abs_cog_column);
    sys.Add(bodyColumn);
}

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create a Chrono physical system
    ChSystemNSC sys;
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // Create a floor that is fixed (that is used also to represent the absolute reference)

    auto floorBody = chrono_types::make_shared<ChBodyEasyBox>(20, 2, 20, 3000, true, false);
    floorBody->SetPos(ChVector<>(0, -2, 0));
    floorBody->SetBodyFixed(true);
    floorBody->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/blue.png"));
    sys.Add(floorBody);

    // Create the table that is subject to earthquake

    auto table_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();

    auto tableBody = chrono_types::make_shared<ChBodyEasyBox>(15, 1, 15, 3000, true, true, table_mat);
    tableBody->SetPos(ChVector<>(0, -0.5, 0));
    tableBody->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/concrete.jpg"));
    sys.Add(tableBody);

    // Create the constraint between ground and table. If no earthquacke, it just
    // keeps the table in position.

    auto linkEarthquake = chrono_types::make_shared<ChLinkLockLock>();
    linkEarthquake->Initialize(tableBody, floorBody, ChCoordsys<>(ChVector<>(0, 0, 0)));

    auto mmotion_x = chrono_types::make_shared<ChFunction_Sine>(0, 0.6, 0.2);  // phase freq ampl
    linkEarthquake->SetMotion_X(mmotion_x);

    sys.Add(linkEarthquake);

    // Create few column chunks

    // Contact material shared among all column chunks
    auto column_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();

    // Create three columns from 5 chunks each
    double spacing = 1.6;
    double density = 3000;
    for (int icol = 0; icol < 5; ++icol) {
        ChCoordsys<> base_position1(ChVector<>(icol * spacing, 0, 0));
        create_column(sys, base_position1, column_mat, 10, 0.45, 0.5, 1.5, density);

        ChCoordsys<> base_position2(ChVector<>(icol * spacing, 1.5, 0));
        create_column(sys, base_position2, column_mat, 10, 0.40, 0.45, 1.5, density);

        ChCoordsys<> base_position3(ChVector<>(icol * spacing, 3.0, 0));
        create_column(sys, base_position3, column_mat, 10, 0.35, 0.40, 1.5, density);

        if (icol < 4) {
            auto bodyTop = chrono_types::make_shared<ChBodyEasyBox>(spacing, 0.4, 1.2,  // x y z sizes
                                                                    density,            // density
                                                                    true, true,         // visualize?, collision?
                                                                    column_mat);        // contact material

            ChCoordsys<> cog_top(ChVector<>(icol * spacing + spacing / 2, 4.5 + 0.4 / 2, 0));
            bodyTop->SetCoord(cog_top);

            sys.Add(bodyTop);
        }
    }

    // Create the run-time visualization system
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
            vis_irr->SetWindowTitle("Collisions between objects");
            vis_irr->Initialize();
            vis_irr->AddLogo();
            vis_irr->AddSkyBox();
            vis_irr->AddCamera(ChVector<>(1, 3, -10));
            vis_irr->AddTypicalLights();
            vis_irr->AddLightWithShadow(ChVector<>(1.0, 25.0, -5.0), ChVector<>(0, 0, 0), 35, 0.2, 35, 35, 512,
                                    ChColor(0.6f, 0.8f, 1.0f));
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
            vis_vsg->SetWindowSize(ChVector2<int>(800, 600));
            vis_vsg->SetWindowPosition(ChVector2<int>(100, 300));
            vis_vsg->SetWindowTitle("Chrono VSG Assets");
            vis_vsg->SetUseSkyBox(true);
            vis_vsg->AddCamera(ChVector<>(1, 8, -15));
            vis_vsg->SetCameraAngleDeg(50);
            vis_vsg->SetLightIntensity(1.0f);
            vis_vsg->SetLightDirection(1.5 * CH_C_PI_2, CH_C_PI_4);
            vis_vsg->Initialize();

            vis = vis_vsg;
#endif
            break;
        }
    }

    // Modify some setting of the physical system for the simulation, if you want
    sys.SetSolverType(ChSolver::Type::PSOR);
    sys.SetSolverMaxIterations(50);

    // sys.SetUseSleeping(true);

    // Simulation loop
    double timestep = 0.005;
    ChRealtimeStepTimer m_realtime_timer;
    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        sys.DoStepDynamics(timestep);
        m_realtime_timer.Spin(timestep);
    }

    return 0;
}
