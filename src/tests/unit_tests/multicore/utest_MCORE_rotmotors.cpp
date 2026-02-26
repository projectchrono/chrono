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
// Test for motors with Chrono and Chrono::Multicore
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono_multicore/physics/ChSystemMulticore.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"

#include "chrono/ChConfig.h"

#include "../ut_utils.h"

using namespace chrono;

enum MotorType { ANGLE, SPEED };

struct Options {
    MotorType mot_type;
    double speed;
};

class RotMotors : public ::testing::TestWithParam<Options> {
  public:
    RotMotors() { opts = GetParam(); }

    void Populate(ChSystem& sys) {
        auto ground = chrono_types::make_shared<ChBody>();
        ground->SetFixed(true);
        sys.AddBody(ground);

        auto body = chrono_types::make_shared<ChBody>();
        sys.AddBody(body);

        switch (opts.mot_type) {
            case MotorType::ANGLE: {
                auto motor_fun = chrono_types::make_shared<ChFunctionRamp>(0.0, opts.speed);
                auto motorA = chrono_types::make_shared<ChLinkMotorRotationAngle>();
                motorA->SetAngleFunction(motor_fun);
                motor = motorA;
                break;
            }
            case MotorType::SPEED: {
                auto motor_fun = chrono_types::make_shared<ChFunctionConst>(opts.speed);
                auto motorS = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
                motorS->SetSpeedFunction(motor_fun);
                motor = motorS;
                break;
            }
        }
        motor->Initialize(ground, body, ChFrame<>(ChVector3d(0, 0, 0), QUNIT));
        sys.AddLink(motor);
    }

    Options opts;
    std::shared_ptr<ChLinkMotorRotation> motor;
};

class RotMotors_SEQ : public RotMotors {
  protected:
    RotMotors_SEQ() {
        std::cout << "SEQ system | motor: "                                   //
                  << (opts.mot_type == MotorType::ANGLE ? "ANGLE" : "SPEED")  //
                  << " | speed: " << opts.speed << std::endl;
        system = new ChSystemNSC();
        system->SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
        system->SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
        system->GetSolver()->AsIterative()->SetTolerance(1e-5);
        Populate(*system);
    }

    ~RotMotors_SEQ() { delete system; }

    ChSystemNSC* system;
};

TEST_P(RotMotors_SEQ, simulate) {
    while (system->GetChTime() < 2) {
        system->DoStepDynamics(1e-3);
        if (system->GetChTime() > 0.1) {
            ASSERT_NEAR(motor->GetMotorAngleDt(), opts.speed, 1e-6);
        }
    }
}

class RotMotors_MCORE : public RotMotors {
  protected:
    RotMotors_MCORE() {
        std::cout << "MCORE system | motor: "                                 //
                  << (opts.mot_type == MotorType::ANGLE ? "ANGLE" : "SPEED")  //
                  << " | speed: " << opts.speed << std::endl;
        system = new ChSystemMulticoreNSC();
        system->SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
        system->GetSettings()->solver.tolerance = 1e-5;
        system->ChangeSolverType(SolverType::BB);

        Populate(*system);
    }

    ~RotMotors_MCORE() { delete system; }

    ChSystemMulticoreNSC* system;
};

TEST_P(RotMotors_MCORE, simulate) {
    while (system->GetChTime() < 2) {
        system->DoStepDynamics(1e-3);
        if (system->GetChTime() > 0.1) {
            ASSERT_NEAR(motor->GetMotorAngleDt(), opts.speed, 1e-6);
        }
    }
}

std::vector<Options> options{
    {MotorType::ANGLE, +0.5},
    {MotorType::SPEED, +0.5},
    {MotorType::ANGLE, -0.5},
    {MotorType::SPEED, -0.5},
};

INSTANTIATE_TEST_SUITE_P(ChronoMulticore, RotMotors_SEQ, ::testing::ValuesIn(options));
INSTANTIATE_TEST_SUITE_P(ChronoMulticore, RotMotors_MCORE, ::testing::ValuesIn(options));
