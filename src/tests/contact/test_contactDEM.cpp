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

using namespace chrono;
using namespace chrono::irrlicht;
using namespace irr;

// Custom contact container -- get access to the contact lists in the base class.
class MyContactContainer : public ChContactContainerDEM {
public:
    MyContactContainer() {}

    // Traverse the list contactlist_6_6
    void ScanContacts(std::shared_ptr<ChBody> ball) {
        auto iter = contactlist_6_6.begin();
        while (iter != contactlist_6_6.end()) {
            ChContactable* objA = (*iter)->GetObjA();
            ChContactable* objB = (*iter)->GetObjB();
            ChVector<> pA = (*iter)->GetContactP1();
            ChVector<> pB = (*iter)->GetContactP2();


            GetLog().SetNumFormat("%12.3e");
            
            int iball;
            if (objA == ball.get()) {
                iball = 0;
                GetLog() << "pA = " << pA << "\n";
            }
            else if (objB == ball.get()) {
                iball = 1;
                GetLog() << "pB = " << pB << "\n";
            }

            
            const ChLcpKblockGeneric* KRM = (*iter)->GetJacobianKRM();
            const ChMatrixDynamic<double>* K = (*iter)->GetJacobianK();
            const ChMatrixDynamic<double>* R = (*iter)->GetJacobianR();

            if (KRM) {
                GetLog() << "K = " << *K << "\n";
                GetLog() << "R = " << *R << "\n";

                ChMatrixNM<double, 6, 6> Kball;
                ChMatrixNM<double, 6, 6> Rball;
                Kball.PasteClippedMatrix(K, iball * 6, iball * 6, 6, 6, 0, 0);
                Rball.PasteClippedMatrix(R, iball * 6, iball * 6, 6, 6, 0, 0);

                GetLog() << "Kball = " << Kball << "\n";
                GetLog() << "Rball = " << Rball << "\n";
            }

            ++iter;
        }
    }
};

// ====================================================================================

int main(int argc, char* argv[]) {
    // ---------------------
    // Simulation parameters
    // ---------------------

    double gravity = -9.81;   // gravitational acceleration
    double time_step = 1e-4;  // integration step size

    enum SolverType { DEFAULT_SOLVER, MINRES_SOLVER, MKL_SOLVER };
    SolverType solver_type = MKL_SOLVER;

    bool stiff_contact = true;

    // ---------------------------
    // Contact material properties
    // ---------------------------

    bool use_mat_properties = false;
    bool use_history = false;
    ChSystemDEM::ContactForceModel force_model = ChSystemDEM::Hooke;

    float young_modulus = 2e9f;
    float friction = 0.4f;
    float restitution = 0.1f;
    float adhesion = 0.0f;

    float kn = 1e8;
    float gn = 0;
    float kt = 0;
    float gt = 0;

    // -------------------------------
    // Parameters for the falling ball
    // -------------------------------
    
    int ballId = 100;
    double radius = 1;
    double mass = 1000;
    ChVector<> pos(0, 2, 0);
    ChQuaternion<> rot(1, 0, 0, 0);
    ChVector<> init_vel(0, 0, 0);
    ChVector<> init_omg(0, 0, 0);

    // ---------------------------------
    // Parameters for the containing bin
    // ---------------------------------

    int binId = 200;
    double width = 2;
    double length = 2;
    double thickness = 0.1;

    // -----------------
    // Create the system
    // -----------------

    ChSystemDEM system(use_mat_properties, use_history);

    // Set the DEM contact force model 
    system.SetContactForceModel(force_model);

    // Set contact forces as stiff (to force Jacobian computation) or non-stiff
    system.SetStiffContact(stiff_contact);

    system.Set_G_acc(ChVector<>(0, gravity, 0));

    // Create the Irrlicht visualization
    ChIrrApp application(&system, L"DEM demo", core::dimension2d<u32>(800, 600), false, true);

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
    material->SetYoungModulus(young_modulus);
    material->SetRestitution(restitution);
    material->SetFriction(friction);
    material->SetAdhesion(adhesion);
    material->SetKn(kn);
    material->SetGn(gn);
    material->SetKt(kt);
    material->SetGt(gt);

    // Create the falling ball
    auto ball = std::make_shared<ChBody>(ChMaterialSurfaceBase::DEM);

    ball->SetIdentifier(ballId);
    ball->SetMass(mass);
    ball->SetInertiaXX(0.4 * mass * radius * radius * ChVector<>(1, 1, 1));
    ball->SetPos(pos);
    ball->SetRot(rot);
    ball->SetPos_dt(init_vel);
    ball->SetWvel_par(init_omg);
    ball->SetCollide(true);
    ball->SetBodyFixed(false);
    ball->SetMaterialSurface(material);

    ball->GetCollisionModel()->ClearModel();
    ball->GetCollisionModel()->AddSphere(radius);
    ball->GetCollisionModel()->BuildModel();

    auto sphere = std::make_shared<ChSphereShape>();
    sphere->GetSphereGeometry().rad = radius;
    ball->AddAsset(sphere);

    auto mtexture = std::make_shared<ChTexture>();
    mtexture->SetTextureFilename(GetChronoDataFile("bluwhite.png"));
    ball->AddAsset(mtexture);

    system.AddBody(ball);

    // Create ground
    auto ground = std::make_shared<ChBody>(ChMaterialSurfaceBase::DEM);

    ground->SetIdentifier(binId);
    ground->SetMass(1);
    ground->SetPos(ChVector<>(0, 0, 0));
    ground->SetRot(ChQuaternion<>(1, 0, 0, 0));
    ground->SetCollide(true);
    ground->SetBodyFixed(true);
    ground->SetMaterialSurface(material);

    ground->GetCollisionModel()->ClearModel();
    ground->GetCollisionModel()->AddBox(width, thickness, length, ChVector<>(0, -thickness, 0));
    ground->GetCollisionModel()->BuildModel();

    auto box = std::make_shared<ChBoxShape>();
    box->GetBoxGeometry().Size = ChVector<>(width, thickness, length);
    box->GetBoxGeometry().Pos = ChVector<>(0, -thickness, 0);
    ground->AddAsset(box);

    system.AddBody(ground);

    // Complete asset construction
    application.AssetBindAll();
    application.AssetUpdateAll();

    // ----------------------------
    // Use custom contact container
    // ----------------------------

    auto container = std::make_shared<MyContactContainer>();
    system.ChangeContactContainer(container);

    // -------------------
    // Setup linear solver
    // -------------------

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
            system.SetIterLCPmaxItersSpeed(100);
            system.SetTolForce(1e-6);
            break;
        }
        case MINRES_SOLVER: {
            GetLog() << "Using MINRES solver.\n";
            ChLcpIterativeMINRES* minres_solver = new ChLcpIterativeMINRES;
            minres_solver->SetDiagonalPreconditioning(true);
            system.ChangeLcpSolverSpeed(minres_solver);
            system.SetIterLCPmaxItersSpeed(100);
            system.SetTolForce(1e-6);
            break;
        }
        case MKL_SOLVER: {
#ifdef CHRONO_MKL
            GetLog() << "Using MKL solver.\n";
            ChLcpMklSolver* mkl_solver = new ChLcpMklSolver;
            system.ChangeLcpSolverSpeed(mkl_solver);
            mkl_solver->SetSparsityPatternLock(true);
#endif
            break;
        }
    }

    // ----------------
    // Setup integrator
    // ----------------

    system.SetIntegrationType(ChSystem::INT_HHT);
    auto integrator = std::static_pointer_cast<ChTimestepperHHT>(system.GetTimestepper());
    integrator->SetAlpha(0.0);
    integrator->SetMaxiters(100);
    integrator->SetAbsTolerances(1e-08);
    ////integrator->SetMode(ChTimestepperHHT::POSITION);
    integrator->SetScaling(false);
    ////integrator->SetStepControl(false);
    ////integrator->SetVerbose(true);

    // ---------------
    // Simulation loop
    // ---------------
    while (application.GetDevice()->run()) {
        application.BeginScene();
        application.DrawAll();

        system.DoStepDynamics(time_step);
        if (ball->GetPos().y <= radius) {
            container->ScanContacts(ball);
            GetLog() << "t = " << system.GetChTime() << "  NR iters. = " << integrator->GetNumIterations() << "\n";
        }

        application.EndScene();
    }

    return 0;
}
