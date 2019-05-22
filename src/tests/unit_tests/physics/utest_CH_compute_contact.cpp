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
// Author: Antonio Recuero, Radu Serban
// =============================================================================
//
// Unit test for ComputeContactForces utility.
// By calling ChBody::GetContactForce(), the user can retrieve the resultant
// of all (!) contact forces acting on the body. In this unit test, the overall
// contact force applied to a contact container is compared to the total weight
// of a number of balls.
//
// =============================================================================

#include <vector>

#include "chrono/ChConfig.h"
#include "chrono/physics/ChContactContainerSMC.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/solver/ChSolverMINRES.h"
#include "chrono/solver/ChSolverSMC.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "gtest/gtest.h"

using namespace chrono;

// ====================================================================================

class ContactForceTest : public ::testing::TestWithParam<ChMaterialSurface::ContactMethod> {
  protected:
    ContactForceTest();
    ~ContactForceTest() { delete system; }

    ChSystem* system;
    std::shared_ptr<ChBody> ground;
    double total_weight;
};

ContactForceTest::ContactForceTest() {
    auto method = GetParam();
    std::shared_ptr<ChMaterialSurface> material;

    switch (method) {
        case ChMaterialSurface::SMC: {
            std::cout << "Using PENALTY method." << std::endl;

            bool use_mat_properties = false;
            bool stiff_contact = true;
            ChSystemSMC::ContactForceModel force_model = ChSystemSMC::Hooke;
            ChSystemSMC::TangentialDisplacementModel tdispl_model = ChSystemSMC::OneStep;

            ChSystemSMC* sys = new ChSystemSMC;
            sys->UseMaterialProperties(use_mat_properties);
            sys->SetContactForceModel(force_model);
            sys->SetTangentialDisplacementModel(tdispl_model);
            sys->SetStiffContact(stiff_contact);
            system = sys;

            float young_modulus = 2e4f;
            float friction = 0.4f;
            float restitution = 0;
            float adhesion = 0;

            float kn = 2e4;
            float gn = 5e2;
            float kt = 0;
            float gt = 0;

            auto mat = std::make_shared<ChMaterialSurfaceSMC>();
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
        case ChMaterialSurface::NSC: {
            std::cout << "Using COMPLEMENTARITY method." << std::endl;

            system = new ChSystemNSC;

            float friction = 0.4f;
            float restitution = 0;

            auto mat = std::make_shared<ChMaterialSurfaceNSC>();
            mat->SetRestitution(restitution);
            mat->SetFriction(friction);
            material = mat;

            break;
        }
    }

    double gravity = -9.81;
    system->Set_G_acc(ChVector<>(0, gravity, 0));

    // Create the falling balls
    unsigned int num_balls = 8;
    std::vector<std::shared_ptr<ChBody>> balls(num_balls);
    total_weight = 0;

    double radius = 0.05;
    double mass = 5;
    ChVector<> pos(0, 0.06, 0);
    ChQuaternion<> rot(1, 0, 0, 0);
    ChVector<> init_vel(0, 0, 0);
    ChVector<> init_omg(0, 0, 0);

    int ballId = 1;
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
    std::cout << "Total weight = " << total_weight << std::endl;

    // Create container box
    int binId = 0;
    double bin_width = 20;
    double bin_length = 20;
    double bin_thickness = 0.1;
    ground = utils::CreateBoxContainer(system, binId, material,                                       //
                                       ChVector<>(bin_width, bin_length, 2 * radius), bin_thickness,  //
                                       ChVector<>(0, 0, 0), ChQuaternion<>(1, 0, 0, 0),               //
                                       true, true, false, false);

    // -------------------
    // Setup linear solver
    // -------------------

    std::cout << "Using default solver." << std::endl;
    system->SetMaxItersSolverSpeed(100);
    system->SetTolForce(1e-6);

    ////std::cout << "Using MINRES solver." << std::endl;
    ////auto minres_solver = std::make_shared<ChSolverMINRES>();
    ////minres_solver->SetDiagonalPreconditioning(true);
    ////system->SetSolver(minres_solver);
    ////system->SetMaxItersSolverSpeed(100);
    ////system->SetTolForce(1e-6);

    // ----------------
    // Setup integrator
    // ----------------

    if (method == ChMaterialSurface::SMC) {
        std::cout << "Using HHT integrator." << std::endl;
        system->SetTimestepperType(ChTimestepper::Type::HHT);
        auto integrator = std::static_pointer_cast<ChTimestepperHHT>(system->GetTimestepper());
        integrator->SetAlpha(0.0);
        integrator->SetMaxiters(100);
        integrator->SetAbsTolerances(1e-08);
        integrator->SetScaling(false);
    } else {
        std::cout << "Using default integrator." << std::endl;
    }
}

// ====================================================================================

TEST_P(ContactForceTest, simulate) {
    double end_time = 3.0;     // total simulation time
    double start_time = 0.25;  // start check after this period
    double time_step = 5e-3;

    double rtol = 1e-3;  // validation relative error

    while (system->GetChTime() < end_time) {
        system->DoStepDynamics(time_step);

        ChVector<> contact_force = ground->GetContactForce();
        ////std::cout << "t = " << system->GetChTime() << " num contacts = " <<
        ///system->GetContactContainer()->GetNcontacts() /          << "  force =  " << contact_force.y() << std::endl;

        if (system->GetChTime() > start_time) {
            ASSERT_LT(std::abs(1 - contact_force.y() / total_weight), rtol);
        }
    }
}

INSTANTIATE_TEST_CASE_P(Chrono, ContactForceTest, ::testing::Values(ChMaterialSurface::NSC, ChMaterialSurface::SMC));
