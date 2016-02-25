//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#include "chrono/ChConfig.h"
#include "chrono/lcp/ChLcpIterativeMINRES.h"
#include "chrono/lcp/ChLcpSolverDEM.h"
#include "chrono/physics/ChContactContainerDEM.h"
#include "chrono/physics/ChSystemDEM.h"

#include "chrono_irrlicht/ChIrrApp.h"

#ifdef CHRONO_MKL
#include "chrono_mkl/ChLcpMklSolver.h"
#endif

#include <irrlicht.h>

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

void AddWall(std::shared_ptr<ChBody> body, const ChVector<>& dim, const ChVector<>& loc) {
    body->GetCollisionModel()->AddBox(dim.x, dim.y, dim.z, loc);

    auto box = std::make_shared<ChBoxShape>();
    box->GetBoxGeometry().Size = dim;
    box->GetBoxGeometry().Pos = loc;
    box->SetColor(ChColor(1, 0, 0));
    box->SetFading(0.6f);
    body->AddAsset(box);
}

int main(int argc, char* argv[]) {
    // Simulation parameters
    double gravity = -9.81;
    double time_step = 1e-5; // integration step size
    int out_steps = 2000;  // render/output every out_step steps

    enum SolverType { DEFAULT_SOLVER, MINRES_SOLVER, MKL_SOLVER };
    enum IntegratorType { EULER_IMPLICIT_LINEARIZED, HHT };
    SolverType solver_type = MKL_SOLVER;
    IntegratorType integrator_type = HHT;

    bool stiff_contact = false;


    // Parameters for the falling ball
    int ballId = 100;
    double radius = 1;
    double mass = 1000;
    ChVector<> pos(0, 2, 0);
    ChQuaternion<> rot(1, 0, 0, 0);
    ChVector<> init_vel(0, 0, 0);

    // Parameters for the containing bin
    int binId = 200;
    double width = 2;
    double length = 2;
    double height = 1;
    double thickness = 0.1;

    // Create the system
    ChSystemDEM msystem;

    // The following two lines are optional, since they are the default options. They are added for future reference,
    // i.e. when needed to change those models.
    msystem.SetContactForceModel(ChSystemDEM::ContactForceModel::Hertz);
    msystem.SetAdhesionForceModel(ChSystemDEM::AdhesionForceModel::Constant);

    // Set contact forces as stiff (to force Jacobian computation)
    msystem.SetStiffContact(stiff_contact);

    msystem.Set_G_acc(ChVector<>(0, gravity, 0));

    // Create the Irrlicht visualization
    ChIrrApp application(&msystem, L"DEM demo", core::dimension2d<u32>(800, 600), false, true);

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(0, 3, -6));

    // This means that contactforces will be shown in Irrlicht application
    application.SetSymbolscale(1e-4);
    application.SetContactsDrawMode(ChIrrTools::eCh_ContactsDrawMode::CONTACT_FORCES);

    // Create a material (will be used by both objects)
    auto material = std::make_shared<ChMaterialSurfaceDEM>();
    material->SetRestitution(0.1f);
    material->SetFriction(0.4f);
    material->SetAdhesion(0);  // Magnitude of the adhesion in Constant adhesion model

    // Create the falling ball
    auto ball = std::make_shared<ChBody>(ChMaterialSurfaceBase::DEM);

    ball->SetIdentifier(ballId);
    ball->SetMass(mass);
    ball->SetPos(pos);
    ball->SetRot(rot);
    ball->SetPos_dt(init_vel);
    // ball->SetWvel_par(ChVector<>(0,0,3));
    ball->SetBodyFixed(false);
    ball->SetMaterialSurface(material);

    ball->SetCollide(true);

    ball->GetCollisionModel()->ClearModel();
    ball->GetCollisionModel()->AddSphere(radius);
    ball->GetCollisionModel()->BuildModel();

    ball->SetInertiaXX(0.4 * mass * radius * radius * ChVector<>(1, 1, 1));

    auto sphere = std::make_shared<ChSphereShape>();
    sphere->GetSphereGeometry().rad = radius;
    ball->AddAsset(sphere);

    auto mtexture = std::make_shared<ChTexture>();
    mtexture->SetTextureFilename(GetChronoDataFile("bluwhite.png"));
    ball->AddAsset(mtexture);

    msystem.AddBody(ball);

    // Create container
    auto bin = std::make_shared<ChBody>(ChMaterialSurfaceBase::DEM);

    bin->SetIdentifier(binId);
    bin->SetMass(1);
    bin->SetPos(ChVector<>(0, 0, 0));
    bin->SetRot(ChQuaternion<>(1, 0, 0, 0));
    bin->SetCollide(true);
    bin->SetBodyFixed(true);
    bin->SetMaterialSurface(material);

    bin->GetCollisionModel()->ClearModel();
    AddWall(bin, ChVector<>(width, thickness, length), ChVector<>(0, 0, 0));
    // AddWall(bin, ChVector<>(thickness, height, length), ChVector<>(-width + thickness, height, 0));
    // AddWall(bin, ChVector<>(thickness, height, length), ChVector<>(width - thickness, height, 0));
    // AddWall(bin, ChVector<>(width, height, thickness), ChVector<>(0, height, -length + thickness));
    // AddWall(bin, ChVector<>(width, height, thickness), ChVector<>(0, height, length - thickness));
    bin->GetCollisionModel()->BuildModel();

    msystem.AddBody(bin);

    // Complete asset construction
    application.AssetBindAll();
    application.AssetUpdateAll();


    // Setup linear solver.
    // Note that not all solvers support stiffness matrices (that includes the default LcpSolverDEM).
#ifndef CHRONO_MKL
    if (solver_type == MKL_SOLVER) {
        GetLog() << "MKL support not enabled.  Solver reset to default.\n";
        solver_type = DEFAULT_SOLVER;
    }
#endif

    switch (solver_type) {
        case DEFAULT_SOLVER: {
            GetLog() << "Using DEFAULT solver.\n";
            msystem.SetIterLCPmaxItersSpeed(100);
            msystem.SetTolForce(1e-6);
            break;
        }
        case MINRES_SOLVER: {
            GetLog() << "Using MINRES solver.\n";
            ChLcpIterativeMINRES* minres_solver = new ChLcpIterativeMINRES;
            minres_solver->SetDiagonalPreconditioning(true);
            msystem.ChangeLcpSolverSpeed(minres_solver);
            msystem.SetIterLCPmaxItersSpeed(100);
            msystem.SetTolForce(1e-6);
            break;
        }
        case MKL_SOLVER: {
#ifdef CHRONO_MKL
            GetLog() << "Using MKL solver.\n";
            ChLcpMklSolver* mkl_solver = new ChLcpMklSolver;
            msystem.ChangeLcpSolverSpeed(mkl_solver);
            mkl_solver->SetSparsityPatternLock(true);
#endif
            break;
        }
    }

    // Setup integrator
    switch (integrator_type) {
        case HHT: {
            GetLog() << "Using HHT integrator.\n";
            msystem.SetIntegrationType(ChSystem::INT_HHT);
            auto mystepper = std::static_pointer_cast<ChTimestepperHHT>(msystem.GetTimestepper());
            mystepper->SetAlpha(0.0);
            mystepper->SetMaxiters(100);
            mystepper->SetAbsTolerances(1e-08);
            ////mystepper->SetMode(ChTimestepperHHT::POSITION);
            mystepper->SetScaling(false);
            ////mystepper->SetStepControl(false);
            ////mystepper->SetVerbose(true);
            break;
        }
    }

    // The soft-real-time cycle
    double time = 0.0;
    double out_time = 0.0;

    while (application.GetDevice()->run()) {
        application.BeginScene();

        application.DrawAll();

        ChIrrTools::drawGrid(application.GetVideoDriver(), 0.2, 0.2, 20, 20,
                             ChCoordsys<>(ChVector<>(0, 0, 0), Q_from_AngX(CH_C_PI_2)),
                             video::SColor(255, 80, 100, 100), true);

        if (integrator_type == HHT) {
            auto mystepper = std::static_pointer_cast<ChTimestepperHHT>(msystem.GetTimestepper());
            int num_iters = 0;
            for (int i = 0; i < out_steps; i++) {
                msystem.DoStepDynamics(time_step);
                num_iters += mystepper->GetNumIterations();
            }
            GetLog() << "t = " << msystem.GetChTime() << "  avg. NR iters. = " << num_iters * (1.0 / out_steps) << "\n";
        }

        application.EndScene();
    }

    return 0;
}
