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
// Author: Antonio Recuero
// =============================================================================
//
// Unit test for ComputeContactForces utility.
// Method system->GetContactContainer()->ComputeContactForces(); iterates over
// all bodies into contact and stores resultant contact force in an unordered map.
// Upon invocation of myBody->GetContactForce(), the user can retrieve the resultant
// of all (!) contact forces acting on the body. In this unit test, the overall
// contact force applied to a contact container is compared to the total weight
// of a number of balls.
//
// =============================================================================

#include "chrono/ChConfig.h"
#include "chrono/lcp/ChLcpIterativeMINRES.h"
#include "chrono/lcp/ChLcpSolverDEM.h"
#include "chrono/physics/ChContactContainerDEM.h"
#include "chrono/physics/ChSystemDEM.h"
#include "chrono/utils/ChUtilsCreators.h"

using namespace chrono;

// ====================================================================================
// Parameters for simulations

double endtime = 3.0;

// Relative tolerance
double rtol = 0.01;

// ---------------------
// Simulation parameters
// ---------------------

double gravity = -9.81;   // gravitational acceleration
double time_step = 5e-3;  // integration step size

enum SolverType { DEFAULT_SOLVER, MINRES_SOLVER, MKL_SOLVER };
SolverType solver_type = MINRES_SOLVER;

bool stiff_contact = true;

// ---------------------------
// Contact material properties
// ---------------------------

bool use_mat_properties = false;
bool use_history = false;
ChSystemDEM::ContactForceModel force_model = ChSystemDEM::Hooke;

float young_modulus = 2e4f;
float friction = 0.4f;
float restitution = 0.99f;
float adhesion = 0.0f;

float kn = 2e4;
float gn = 5e2;
float kt = 0;
float gt = 0;

// -------------------------------
// Parameters for the falling ball
// -------------------------------

int ballId = 100;
double radius = 0.05;
double mass = 5;
ChVector<> pos(0, 0.051, 0);
ChQuaternion<> rot(1, 0, 0, 0);
ChVector<> init_vel(0, 0, 0);
ChVector<> init_omg(0, 0, 0);

// ---------------------------------
// Parameters for the containing bin
// ---------------------------------

int binId = 200;
double width_ = 20;
double length_ = 20;
double thickness = 0.1;

// Forward declaration
bool test_computecontact(ChMaterialSurfaceBase::ContactMethod method, std::shared_ptr<ChMaterialSurfaceBase> mat);

int main(int argc, char* argv[]) {

    auto matDEM = std::make_shared<ChMaterialSurfaceDEM>();
    auto matDVI = std::make_shared<ChMaterialSurface>();

    bool DEM_passed = test_computecontact(ChMaterialSurfaceBase::DEM, matDEM);
    bool DVI_passed = test_computecontact(ChMaterialSurfaceBase::DVI, matDVI);
    if (DEM_passed && DVI_passed) {
        return 0;
    } else
        return 1;
}

// ====================================================================================

bool test_computecontact(ChMaterialSurfaceBase::ContactMethod method, std::shared_ptr<ChMaterialSurfaceBase> mat) {
    ChSystem* system =
        (method == ChMaterialSurfaceBase::DVI) ? new ChSystem : new ChSystemDEM(use_mat_properties, use_history);

    if (method == ChMaterialSurfaceBase::DEM) {
        if (auto material = std::dynamic_pointer_cast<ChMaterialSurfaceDEM>(mat)) {
            // Set the DEM contact force model
            dynamic_cast<ChSystemDEM*>(system)->SetContactForceModel(force_model);
            // Set contact forces as stiff (to force Jacobian computation) or non-stiff
            dynamic_cast<ChSystemDEM*>(system)->SetStiffContact(stiff_contact);
            // Create a material (will be used by both objects)
            material->SetYoungModulus(young_modulus);
            material->SetRestitution(restitution);
            material->SetFriction(friction);
            material->SetAdhesion(adhesion);
            material->SetKn(kn);
            material->SetGn(gn);
            material->SetKt(kt);
            material->SetGt(gt);
        }
    } else if (method == ChMaterialSurfaceBase::DVI) {
        if (auto material = std::dynamic_pointer_cast<ChMaterialSurface>(mat)) {
            // Create a material (will be used by both objects)
            material->SetRestitution(restitution);
            material->SetFriction(friction);
        }
    }
    system->Set_G_acc(ChVector<>(0, gravity, 0));

    // Establish number of falling balls
    unsigned int noBalls = 8;

    // Create the falling balls

    for (unsigned int i = 0; i < noBalls; i++) {
        auto ball = std::make_shared<ChBody>(ChMaterialSurfaceBase::DEM);  // use also complementarity

        ball->SetIdentifier(ballId);
        ball->SetMass(mass);
        ball->SetInertiaXX(0.4 * mass * radius * radius * ChVector<>(1, 1, 1));
        ball->SetPos(pos + ChVector<>(double(i) * 0.1, 0, double(i) * 0.1));
        ball->SetRot(rot);
        ball->SetPos_dt(init_vel);
        ball->SetWvel_par(init_omg);
        ball->SetCollide(true);
        ball->SetBodyFixed(false);
        ball->SetMaterialSurface(mat);

        ball->GetCollisionModel()->ClearModel();
        ball->GetCollisionModel()->AddSphere(radius);
        ball->GetCollisionModel()->BuildModel();

        system->AddBody(ball);
    }

    // Create container box
    auto ground = utils::CreateBoxContainer(system, binId, mat, ChVector<>(width_, length_, 0), thickness,
                                            ChVector<>(0, 0, 0), ChQuaternion<>(1, 0, 0, 0), true, true, false, false);
    // -------------------
    // Setup linear solver
    // -------------------

    switch (solver_type) {
        case DEFAULT_SOLVER: {
            GetLog() << "Using DEFAULT solver.\n";
            system->SetIterLCPmaxItersSpeed(100);
            system->SetTolForce(1e-6);
            break;
        }
        case MINRES_SOLVER: {
            GetLog() << "Using MINRES solver.\n";
            ChLcpIterativeMINRES* minres_solver = new ChLcpIterativeMINRES;
            minres_solver->SetDiagonalPreconditioning(true);
            system->ChangeLcpSolverSpeed(minres_solver);
            system->SetIterLCPmaxItersSpeed(100);
            system->SetTolForce(1e-6);
            break;
        }
    }

    // ----------------
    // Setup integrator
    // ----------------

    system->SetIntegrationType(ChSystem::INT_HHT);
    auto integrator = std::static_pointer_cast<ChTimestepperHHT>(system->GetTimestepper());
    integrator->SetAlpha(0.0);
    integrator->SetMaxiters(100);
    integrator->SetAbsTolerances(1e-08);
    integrator->SetScaling(false);

    // ---------------
    // Simulation loop
    double simtime = 0.0;
    // ---------------
    while (simtime < endtime) {
        system->DoStepDynamics(time_step);

        GetLog() << "t = " << system->GetChTime() << "  NR iters. = " << integrator->GetNumIterations() << "\n";
        system->GetContactContainer()->ComputeContactForces();
        GetLog() << "Total force on ground:  " << ground->GetContactForce() << "\n";

        if (simtime > 0.25) {
            if (std::abs(noBalls * mass * gravity - ground->GetContactForce().y) <
                rtol * std::abs(noBalls * mass * gravity))
                return true;
            else
                return false;
        }
        simtime += time_step;
    }
    return false;
}
