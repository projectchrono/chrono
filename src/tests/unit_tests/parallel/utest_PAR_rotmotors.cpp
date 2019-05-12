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
// Authors: Radu Serban
// =============================================================================
//
// Test for motors with Chrono::Parallel
//
// The mechanism consists of three bodies (ground, sled, and pendulum) with a
// prismatic joint between ground and sled and a revolute joint between sled and
// pendulum.
// The system is simulated with different combinations of solver settings
// (type of solver, solver mode, maximum number of iterations).  Constraint
// violations are monitored and verified.
//
// =============================================================================

#include "chrono_parallel/physics/ChSystemParallel.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"

#include "chrono/ChConfig.h"

#include "unit_testing.h"

using namespace chrono;
using namespace chrono::collision;

enum MotorType {ANGLE, SPEED};
struct Options {
    MotorType mot_type;
    double speed;
};

class RotMotors : public ::testing::TestWithParam<Options> {
  protected:
    RotMotors() {
        opts = GetParam();

        system = new ChSystemParallelNSC();
        system->Set_G_acc(ChVector<>(0, 0, -9.81));
        system->GetSettings()->solver.tolerance = 1e-5;
        system->ChangeSolverType(SolverType::BB);

        auto ground = std::shared_ptr<ChBody>(system->NewBody());
        ground->SetBodyFixed(true);
        system->AddBody(ground);

        auto body = std::shared_ptr<ChBody>(system->NewBody());
        system->AddBody(body);

        switch (opts.mot_type) {
            case MotorType::ANGLE: {
                auto motor_fun = std::make_shared<ChFunction_Ramp>(0.0, opts.speed);
                auto motorA = std::make_shared<ChLinkMotorRotationAngle>();
                motorA->SetAngleFunction(motor_fun);
                motor = motorA;
                break;
            }
            case MotorType::SPEED: {
                auto motor_fun = std::make_shared<ChFunction_Const>(opts.speed);
                auto motorS = std::make_shared<ChLinkMotorRotationSpeed>();
                motorS->SetSpeedFunction(motor_fun);
                motor = motorS;
                break;
            }
        }
        motor->Initialize(ground, body, ChFrame<>(ChVector<>(0, 0, 0), QUNIT));
        system->AddLink(motor);
    }

    Options opts;
    ChSystemParallelNSC* system;
    std::shared_ptr<ChLinkMotorRotation> motor;
};

TEST_P(RotMotors, simulate) {
    while (system->GetChTime() < 2) {
        system->DoStepDynamics(1e-3);
        if (system->GetChTime() > 0.1) {
            ASSERT_NEAR(motor->GetMotorRot_dt(), opts.speed, 1e-6);
        }
    }
}

std::vector<Options> options{
    {MotorType::ANGLE, +0.5},
    {MotorType::SPEED, +0.5},
    {MotorType::ANGLE, -0.5},
    {MotorType::SPEED, -0.5},
};

INSTANTIATE_TEST_CASE_P(ChronoParallel, RotMotors, ::testing::ValuesIn(options));
