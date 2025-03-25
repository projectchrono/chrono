// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2025 projectchrono.org
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
// Demo for a 3D simple pendulum modeled using SOA relative coordinates
//
// =============================================================================

#include <cstdio>

#include "chrono/core/ChRealtimeStep.h"

#include "chrono/physics/ChSystemNSC.h"

#include "chrono/soa/ChSoaAssembly.h"
#include "chrono/soa/ChRevoluteBody.h"

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
using namespace chrono::soa;

using std::cout;
using std::endl;

// =============================================================================

ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// =============================================================================

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2025 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    ChSystemNSC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, 0));

    // Create an SOA assembly with a double pnedulum
    // ---------------------------------------------

    auto soa = chrono_types::make_shared<ChSoaAssembly>();
    std::shared_ptr<ChRevoluteBody> pendulum1;
    std::shared_ptr<ChRevoluteBody> pendulum2;
    double L1 = 2;
    double L2 = 1;
    double init1 = CH_PI_4;
    double init2 = CH_PI_2;

    // Create a contact material (shared by all objects)
    ChContactMaterialData material_data;
    material_data.cr = 0.1f;
    material_data.mu = 0.5f;
    auto material = material_data.CreateMaterial(sys.GetContactMethod());

    // First pendulum (with reference frame at inboard joint)
    {
        double mass1 = 2;
        ChMatrix33d inertia1(ChVector3d(0.01, mass1 * L1 * L1 / 12, mass1 * L1 * L1 / 12));
        ChMassProps pendulum1_mprops(mass1, ChVector3d(L1 / 2, 0, 0), inertia1);
        pendulum1 = chrono_types::make_shared<ChRevoluteBody>(soa->getGroundBody(), pendulum1_mprops,  //
                                                              ChFramed(VNULL, Q_ROTATE_Z_TO_Y),        //
                                                              ChFramed(VNULL, Q_ROTATE_Z_TO_Y),        //
                                                              "pendulum1");
        auto vis_shape = chrono_types::make_shared<ChVisualShapeBox>(L1, 0.1, 0.1);
        vis_shape->SetColor(ChColor(0, 0, 0.6f));
        pendulum1->AddVisualShape(vis_shape, ChFramed(ChVector3d(L1 / 2, 0, 0), QUNIT));

        auto coll_shape = chrono_types::make_shared<ChCollisionShapeBox>(material, L1, 0.1, 0.1);
        pendulum1->AddCollisionShape(coll_shape, ChFramed(ChVector3d(L1 / 2, 0, 0), QUNIT));

        pendulum1->setRelPos(init1);
        pendulum1->setRelVel(0.5);
        soa->AddBody(pendulum1);
    }

    // Second pendulum (with reference frame at inboard joint)
    {
        double mass2 = 1;
        ChMatrix33d inertia2(ChVector3d(0.01, mass2 * L2 * L2 / 12, mass2 * L2 * L2 / 12));
        ChMassProps pendulum2_mprops(mass2, ChVector3d(L2 / 2, 0, 0), inertia2);
        pendulum2 = chrono_types::make_shared<ChRevoluteBody>(pendulum1, pendulum2_mprops,             //
                                                              ChFramed(ChVector3d(+L1, 0, 0), QUNIT),  //
                                                              ChFramed(VNULL, QUNIT),                  //
                                                              "pendulum2");
        auto vis_shape = chrono_types::make_shared<ChVisualShapeBox>(L2, 0.1, 0.1);
        vis_shape->SetColor(ChColor(0.6f, 0, 0));
        pendulum2->AddVisualShape(vis_shape, ChFramed(ChVector3d(L2 / 2, 0, 0), QUNIT));

        pendulum2->setRelPos(init2);
        pendulum2->setRelVel(0.1);
        soa->AddBody(pendulum2);
    }

    // Attach SOA assembly to Chrono system
    sys.Add(soa);

    // Initialize assembly (perform position- and velocity-level traversal)
    soa->Initialize();

    // Traverse bodies in assembly and print their absolute position, orientation, and velocities
    cout << "Traverse SOA bodies" << endl;
    for (const auto& b : soa->getBodies()) {
        cout << "  " << b->getName() << endl;
        cout << "           p: " << b->getAbsLoc() << " | q: " << b->getAbsQuat() << endl;
        cout << "           v: " << b->getAbsLinVel() << " | o: " << b->getAbsAngVel() << endl;
        cout << "     COM   p: " << b->getAbsCOMLoc() << " | v: " << b->getAbsCOMVel() << endl;
    }

    cout << "Find SOA bodies by name" << endl;
    auto p1 = soa->findBody("pendulum1");
    auto p2 = soa->findBody("pendulum2");
    cout << "  pendulum1 " << (p1 ? " found" : " not found") << endl;
    cout << "  pendulum2 " << (p2 ? " found" : " not found") << endl;

    // Create the run-time visualization system
    // ----------------------------------------

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
            vis_irr->SetWindowTitle("SOA double pendulum demo");
            vis_irr->Initialize();
            vis_irr->AddLogo();
            vis_irr->AddSkyBox();
            vis_irr->AddCamera(ChVector3d(0, 0, 6));
            vis_irr->AddTypicalLights();

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
            vis_vsg->SetWindowSize(ChVector2i(800, 600));
            vis_vsg->SetWindowPosition(ChVector2i(100, 300));
            vis_vsg->SetWindowTitle("SOA double pendulum demo");
            vis_vsg->SetUseSkyBox(true);
            vis_vsg->AddCamera(ChVector3d(0, 0, 12));
            vis_vsg->SetCameraAngleDeg(40);
            vis_vsg->SetLightIntensity(1.0f);
            vis_vsg->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
            vis_vsg->SetShadows(true);
            vis_vsg->Initialize();

            vis = vis_vsg;
#endif
            break;
        }
    }

    // Simulation loop
    // ---------------

    double time = 0;
    double step = 1e-2;

    ChFunctionSine dof1(CH_PI, 1 / 4.0, 0.0);
    ChFunctionSine dof2(CH_PI_2, 1 / 2.0, 0.0);

    ChRealtimeStepTimer rt_timer;

    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        double angle1 = init1 + dof1(time);
        double angle2 = init2 + dof2(time);

        pendulum1->setRelPos(angle1);
        pendulum2->setRelPos(angle2);

        ////std::cout << "t: " << time << "  a1: " << angle1 << "  a2: " << angle2 << std::endl;
        ////std::cout << "  pendulum1: " << pendulum1->getAbsLoc() << std::endl;
        ////std::cout << "  pendulum2: " << pendulum2->getAbsLoc() << std::endl;

        soa->DoForwardKinematics();
        rt_timer.Spin(step);

        time += step;
    }
}
