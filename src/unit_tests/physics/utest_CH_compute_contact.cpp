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
// Author: Antonio Recuero, Radu Serban
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

#include <vector>

#include "chrono/ChConfig.h"
#include "chrono/solver/ChSolverMINRES.h"
#include "chrono/solver/ChSolverDEM.h"
#include "chrono/physics/ChContactContainerDEM.h"
#include "chrono/physics/ChSystemDEM.h"
#include "chrono/utils/ChUtilsCreators.h"

using namespace chrono;

// ====================================================================================

// ---------------------
// Simulation parameters
// ---------------------

double end_time = 3.0;     // total simulation time
double start_time = 0.25;  // start check after this period
double time_step = 5e-3;   // integration step size
double gravity = -9.81;    // gravitational acceleration

enum SolverType { DEFAULT_SOLVER, MINRES_SOLVER, MKL_SOLVER };
SolverType solver_type = DEFAULT_SOLVER;

bool stiff_contact = true;

double rtol = 1e-3;  // validation relative error

// ---------------------------
// Contact material properties
// ---------------------------

bool use_mat_properties = false;
ChSystemDEM::ContactForceModel force_model = ChSystemDEM::Hooke;
ChSystemDEM::TangentialDisplacementModel tdispl_model = ChSystemDEM::OneStep;

float young_modulus = 2e4f;
float friction = 0.4f;
float restitution = 0;
float adhesion = 0;

float kn = 2e4;
float gn = 5e2;
float kt = 0;
float gt = 0;

// --------------------------------
// Parameters for the falling balls
// --------------------------------

unsigned int num_balls = 8;

int ballId = 1;
double radius = 0.05;
double mass = 5;
ChVector<> pos(0, 0.06, 0);
ChQuaternion<> rot(1, 0, 0, 0);
ChVector<> init_vel(0, 0, 0);
ChVector<> init_omg(0, 0, 0);

// ---------------------------------
// Parameters for the containing bin
// ---------------------------------

int binId = 0;
double bin_width = 20;
double bin_length = 20;
double bin_thickness = 0.1;

// Forward declaration
bool test_computecontact(ChMaterialSurfaceBase::ContactMethod method);

// ====================================================================================

int main(int argc, char* argv[]) {

    bool passed = true;
    passed &= test_computecontact(ChMaterialSurfaceBase::DEM);
    passed &= test_computecontact(ChMaterialSurfaceBase::DVI);

    // Return 0 if all tests passed.
    return !passed;
}

// ====================================================================================

bool test_computecontact(ChMaterialSurfaceBase::ContactMethod method) {
    // Create system and contact material.
    ChSystem* system;
    std::shared_ptr<ChMaterialSurfaceBase> material;

    switch (method) {
        case ChMaterialSurfaceBase::DEM: {
            GetLog() << "Using PENALTY method.\n";

            ChSystemDEM* sys = new ChSystemDEM;
            sys->UseMaterialProperties(use_mat_properties);
            sys->SetContactForceModel(force_model);
            sys->SetTangentialDisplacementModel(tdispl_model);
            sys->SetStiffContact(stiff_contact);
            system = sys;

            auto mat = std::make_shared<ChMaterialSurfaceDEM>();
            mat->SetYoungModulus(young_modulus);
            mat->SetRestitution(restitution);
            mat->SetFriction(friction);
            mat->SetAdhesion(adhesion);
            mat->SetKn(kn);
            mat->SetGn(gn);
            mat->SetKt(kt);
            mat->SetGt(gt);
            material = mat;

            break;
        }
        case ChMaterialSurfaceBase::DVI: {
            GetLog() << "Using COMPLEMENTARITY method.\n";

            system = new ChSystem;

            auto mat = std::make_shared<ChMaterialSurface>();
            mat->SetRestitution(restitution);
            mat->SetFriction(friction);
            material = mat;

            break;
        }
    }

    system->Set_G_acc(ChVector<>(0, gravity, 0));

    // Create the falling balls
    std::vector<std::shared_ptr<ChBody>> balls(num_balls);
    double total_weight = 0;

    for (unsigned int i = 0; i < num_balls; i++) {
        auto ball = std::shared_ptr<ChBody>(system->NewBody());

        ball->SetIdentifier(ballId++);
        ball->SetMass(mass);
        ball->SetInertiaXX(0.4 * mass * radius * radius * ChVector<>(1, 1, 1));
        ball->SetPos(pos + ChVector<>(i * 2 * radius, 0, i * 2 * radius));
        ball->SetRot(rot);
        ball->SetPos_dt(init_vel);
        ball->SetWvel_par(init_omg);
        ball->SetCollide(true);
        ball->SetBodyFixed(false);
        ball->SetMaterialSurface(material);

        ball->GetCollisionModel()->ClearModel();
        ball->GetCollisionModel()->AddSphere(radius);
        ball->GetCollisionModel()->BuildModel();

        system->AddBody(ball);
        balls[i] = ball;

        total_weight += ball->GetMass();
    }
    total_weight *= gravity;
    GetLog() << "Total weight = " << total_weight << "\n";

    // Create container box
    auto ground = utils::CreateBoxContainer(system, binId, material, ChVector<>(bin_width, bin_length, 2 * radius), bin_thickness,
                                            ChVector<>(0, 0, 0), ChQuaternion<>(1, 0, 0, 0), true, true, false, false);

    // -------------------
    // Setup linear solver
    // -------------------

    switch (solver_type) {
        case DEFAULT_SOLVER: {
            GetLog() << "Using DEFAULT solver.\n";
            system->SetMaxItersSolverSpeed(100);
            system->SetTolForce(1e-6);
            break;
        }
        case MINRES_SOLVER: {
            GetLog() << "Using MINRES solver.\n";
            auto minres_solver = std::make_shared<ChSolverMINRES>();
            minres_solver->SetDiagonalPreconditioning(true);
            system->SetSolver(minres_solver);
            system->SetMaxItersSolverSpeed(100);
            system->SetTolForce(1e-6);
            break;
        }
        default:
            break;
    }

    // ----------------
    // Setup integrator
    // ----------------

    if (method == ChMaterialSurfaceBase::DEM) {
        GetLog() << "Using HHT integrator.\n";
        system->SetTimestepperType(ChTimestepper::Type::HHT);
        auto integrator = std::static_pointer_cast<ChTimestepperHHT>(system->GetTimestepper());
        integrator->SetAlpha(0.0);
        integrator->SetMaxiters(100);
        integrator->SetAbsTolerances(1e-08);
        integrator->SetScaling(false);
    } else {
        GetLog() << "Using default integrator.\n";
    }

    // ---------------
    // Simulation loop
    // ---------------

    bool passed = true;
    while (system->GetChTime() < end_time) {
        system->DoStepDynamics(time_step);

        system->GetContactContainer()->ComputeContactForces();
        ChVector<> contact_force = ground->GetContactForce();
        ////GetLog() << "t = " << system->GetChTime() << " num contacts = " << system->GetContactContainer()->GetNcontacts()
        ////         << "  force =  " << contact_force.y << "\n";

        if (system->GetChTime() > start_time) {
            if (std::abs(1 - contact_force.y() / total_weight) > rtol) {
                GetLog() << "t = " << system->GetChTime() << "  force =  " << contact_force.y() << "\n";
                passed = false;
                break;
            }
        }
    }

    GetLog() << "Test " << (passed ? "PASSED" : "FAILED") << "\n\n\n";

    delete system;
    return passed;
}
