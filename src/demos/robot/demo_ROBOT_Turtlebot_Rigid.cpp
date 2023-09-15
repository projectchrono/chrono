// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2021 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Jason Zhou
// =============================================================================
//
// Demo to show a Turtlebot Robot operated on Rigid Terrain
//
// =============================================================================

#include "chrono_models/robot/turtlebot/Turtlebot.h"

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"

#ifdef CHRONO_POSTPROCESS
    #include "chrono_postprocess/ChGnuPlot.h"
#endif

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
using namespace chrono::turtlebot;

// -----------------------------------------------------------------------------

// Run-time visualization system (IRRLICHT or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// Use custom material for the turtlebot wheels
bool use_custom_mat = false;

// Simulation time step
double time_step = 0.0005;

// -----------------------------------------------------------------------------

std::shared_ptr<ChMaterialSurface> CustomWheelMaterial(ChContactMethod contact_method) {
    float mu = 0.4f;   // coefficient of friction
    float cr = 0.2f;   // coefficient of restitution
    float Y = 2e7f;    // Young's modulus
    float nu = 0.3f;   // Poisson ratio
    float kn = 2e5f;   // normal stiffness
    float gn = 40.0f;  // normal viscous damping
    float kt = 2e5f;   // tangential stiffness
    float gt = 20.0f;  // tangential viscous damping

    switch (contact_method) {
        case ChContactMethod::NSC: {
            auto matNSC = chrono_types::make_shared<ChMaterialSurfaceNSC>();
            matNSC->SetFriction(mu);
            matNSC->SetRestitution(cr);
            return matNSC;
        }
        case ChContactMethod::SMC: {
            auto matSMC = chrono_types::make_shared<ChMaterialSurfaceSMC>();
            matSMC->SetFriction(mu);
            matSMC->SetRestitution(cr);
            matSMC->SetYoungModulus(Y);
            matSMC->SetPoissonRatio(nu);
            matSMC->SetKn(kn);
            matSMC->SetGn(gn);
            matSMC->SetKt(kt);
            matSMC->SetGt(gt);
            return matSMC;
        }
        default:
            return std::shared_ptr<ChMaterialSurface>();
    }
}

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";
    // Create a ChronoENGINE physical system
    ChSystemNSC sys;

    // set gravity
    sys.Set_G_acc(ChVector<>(0, 0, -9.81));

    collision::ChCollisionModel::SetDefaultSuggestedEnvelope(0.0025);
    collision::ChCollisionModel::SetDefaultSuggestedMargin(0.0025);

    // Create a floor
    auto floor_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    auto mfloor = chrono_types::make_shared<ChBodyEasyBox>(20, 20, 1, 1000, true, true, floor_mat);
    mfloor->SetPos(ChVector<>(0, 0, -1));
    mfloor->SetBodyFixed(true);
    mfloor->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/concrete.jpg"));
    sys.Add(mfloor);

    // Create a Turtlebot Robot with default parameters.
    // The default rotational speed of the Motor is speed w=3.145 rad/sec.
    std::shared_ptr<TurtleBot> robot;

    ChVector<double> body_pos(0, 0, -0.45);

    robot = chrono_types::make_shared<TurtleBot>(&sys, body_pos, QUNIT);

    robot->Initialize();

    // Create the run-time visualization interface
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
            vis_irr->SetCameraVertical(CameraVerticalDir::Z);
            vis_irr->SetWindowSize(800, 600);
            vis_irr->SetWindowTitle("Turtlebot Robot on Rigid Terrain");
            vis_irr->Initialize();
            vis_irr->AddLogo();
            vis_irr->AddSkyBox();
            vis_irr->AddCamera(ChVector<>(0, 0.5, 0.5));
            vis_irr->AddTypicalLights();
            vis_irr->AddLightWithShadow(ChVector<>(1.5, 1.5, 5.5), ChVector<>(0, 0, 0), 3, 4, 10, 40, 512,
                                        ChColor(0.8f, 0.8f, 1.0f));
            vis_irr->EnableContactDrawing(ContactsDrawMode::CONTACT_DISTANCES);
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
            vis_vsg->SetWindowSize(800, 600);
            vis_vsg->SetWindowTitle("Turtlebot Robot on Rigid Terrain");
            vis_vsg->AddCamera(ChVector<>(0, 2.5, 0.5));
            vis_vsg->Initialize();

            vis = vis_vsg;
#endif
            break;
        }
    }

    // Simulation loop
    float time = 0.0f;
    while (vis->Run()) {
#if defined(CHRONO_IRRLICHT) || defined(CHRONO_VSG)
        vis->BeginScene();
        vis->Render();
        vis->EndScene();
#endif

        // at time = 1 s, start left turn
        if (abs(time - 1.0f) < 1e-4) {
            robot->SetMotorSpeed(-0.f, WheelID::LD);
            robot->SetMotorSpeed(-CH_C_PI, WheelID::RD);
        }

        // at time = 2 s, start right turn
        if (abs(time - 2.0f) < 1e-4) {
            robot->SetMotorSpeed(-CH_C_PI, WheelID::LD);
            robot->SetMotorSpeed(-0.f, WheelID::RD);
        }

        // read and display angular velocities of two drive wheels
        ////ChVector<> L_Ang = robot->GetActiveWheelAngVel(WheelID::LD);
        ////ChVector<> R_Ang = robot->GetActiveWheelAngVel(WheelID::RD);
        ////std::cout << "time_step: " << time << " W_L: " << L_Ang.y() << " W_R: " << R_Ang.y() << std::endl;

        // increment time indicator
        time = time + time_step;

        sys.DoStepDynamics(time_step);
    }

    return 0;
}
