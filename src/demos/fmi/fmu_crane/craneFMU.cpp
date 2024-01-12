// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================

// #define FMI2_FUNCTION_PREFIX MyModel_
#include <cassert>
#include <map>
#include <algorithm>

#include "chrono/physics/ChLoadContainer.h"
#include "chrono/solver/ChDirectSolverLS.h"
#include "chrono/serialization/ChArchive.h"

#include "craneFMU.h"

using namespace chrono;

FmuComponent::FmuComponent(fmi2String _instanceName, fmi2Type _fmuType, fmi2String _fmuGUID)
    : FmuChronoComponentBase(_instanceName, _fmuType, _fmuGUID) {
    // Initialize FMU type
    initializeType(_fmuType);

    // Set initial values for FMU input variables
    F = 0;

    // Set configuration flags for this FMU
    AddFmuVariable(&vis, "vis", FmuVariable::Type::Boolean, "1", "enable visualization",         //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);  //

    // Set FIXED PARAMETERS for this FMU
    AddFmuVariable(&crane_mass, "crane_mass", FmuVariable::Type::Real, "kg", "crane mass",                   //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);              //
    AddFmuVariable(&crane_length, "crane_length", FmuVariable::Type::Real, "m", "crane length",              //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);              //
    AddFmuVariable(&pend_mass, "pend_mass", FmuVariable::Type::Real, "kg", "pendulum mass",                  //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);              //
    AddFmuVariable(&pend_length, "pend_length", FmuVariable::Type::Real, "m", "pendulum length",             //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);              //
    AddFmuVariable(&init_crane_angle, "crane_angle", FmuVariable::Type::Real, "rad", "initial crane angle",  //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);              //

    // Set CONTINOUS INPUTS and OUTPUTS for this FMU
    AddFmuVariable(&init_F, "init_F", FmuVariable::Type::Real, "N", "initial load",                //
                   FmuVariable::CausalityType::output, FmuVariable::VariabilityType::continuous);  //
    AddFmuVariable(&s, "s", FmuVariable::Type::Real, "m", "actuator length",                       //
                   FmuVariable::CausalityType::output, FmuVariable::VariabilityType::continuous);  //
    AddFmuVariable(&sd, "sd", FmuVariable::Type::Real, "m/s", "actuator length rate",              //
                   FmuVariable::CausalityType::output, FmuVariable::VariabilityType::continuous);  //
    AddFmuVariable(&F, "F", FmuVariable::Type::Real, "N", "actuator force",                        //
                   FmuVariable::CausalityType::input, FmuVariable::VariabilityType::continuous);   //

    // Hardcoded mount points
    m_point_ground = ChVector<>(std::sqrt(3.0) / 2, 0, 0);
    m_point_crane = ChVector<>(0, 0, 0);

    // Initial crane and pendulum positions
    ChVector<> crane_pos(0.5 * crane_length * std::cos(init_crane_angle), 0,
                         0.5 * crane_length * std::sin(init_crane_angle));
    ChVector<> pend_pos = 2.0 * crane_pos + ChVector<>(0, 0, -pend_length);

    // Set gravitational acceleration
    ChVector<> Gacc(0, 0, -9.8);
    sys.Set_G_acc(Gacc);

    // Estimate initial required force (moment balance about crane pivot)
    auto Gtorque = Vcross(crane_mass * Gacc, crane_pos) + Vcross(pend_mass * Gacc, pend_pos);
    auto dir = (crane_pos - m_point_ground).GetNormalized();
    init_F = Gtorque.Length() / Vcross(dir, crane_pos).Length();  // mass balance about crane pivot

    // Visualization of connection points
    auto connection_sph = chrono_types::make_shared<ChVisualShapeSphere>(0.02);
    connection_sph->SetColor(ChColor(0.7f, 0.3f, 0.3f));

    // Create bodies
    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetBodyFixed(true);
    ground->AddVisualShape(connection_sph, ChFrame<>());
    ground->AddVisualShape(connection_sph, ChFrame<>(m_point_ground, QUNIT));
    sys.AddBody(ground);

    m_crane = chrono_types::make_shared<ChBody>();
    m_crane->SetMass(crane_mass);
    m_crane->SetPos(crane_pos);
    m_crane->SetRot(Q_from_AngY(-init_crane_angle));
    m_crane->AddVisualShape(connection_sph, ChFrame<>(m_point_crane, QUNIT));
    m_crane->AddVisualShape(connection_sph, ChFrame<>(ChVector<>(crane_length / 2, 0, 0), QUNIT));
    auto crane_cyl = chrono_types::make_shared<ChVisualShapeCylinder>(0.015, crane_length);
    m_crane->AddVisualShape(crane_cyl, ChFrame<>(VNULL, Q_from_AngY(CH_C_PI_2)));
    sys.AddBody(m_crane);

    auto ball = chrono_types::make_shared<ChBody>();
    ball->SetMass(pend_mass);
    ball->SetPos(pend_pos);
    auto ball_sph = chrono_types::make_shared<ChVisualShapeSphere>(0.04);
    ball->AddVisualShape(ball_sph);
    auto ball_cyl = chrono_types::make_shared<ChVisualShapeCylinder>(0.005, pend_length);
    ball->AddVisualShape(ball_cyl, ChFrame<>(ChVector<>(0, 0, pend_length / 2), QUNIT));
    sys.AddBody(ball);

    // Create joints
    auto rev_joint = chrono_types::make_shared<ChLinkRevolute>();
    rev_joint->Initialize(ground, m_crane, ChFrame<>(VNULL, Q_from_AngX(CH_C_PI_2)));
    sys.AddLink(rev_joint);

    auto sph_joint = chrono_types::make_shared<ChLinkLockSpherical>();
    sph_joint->Initialize(m_crane, ball, ChCoordsys<>(2.0 * crane_pos, QUNIT));
    sys.AddLink(sph_joint);

    // Create an external force load on crane
    auto load_container = std::make_shared<ChLoadContainer>();
    m_external_load = chrono_types::make_shared<ChLoadBodyForce>(m_crane, VNULL, false, VNULL, false);
    load_container->Add(m_external_load);
    sys.Add(load_container);

    // Set solver and integrator
    auto solver = chrono_types::make_shared<ChSolverSparseQR>();
    sys.SetSolver(solver);
    solver->UseSparsityPatternLearner(true);
    solver->LockSparsityPattern(true);
    solver->SetVerbose(false);

    sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT);
    auto integrator = std::static_pointer_cast<chrono::ChTimestepperEulerImplicit>(sys.GetTimestepper());
    integrator->SetMaxiters(50);
    integrator->SetAbsTolerances(1e-4, 1e2);

    // Initialize FMU outputs (in case they are queried before the first step)
    CalculateActuatorLength();

    // Specify functions to process input variables (at beginning of step)
    preStepCallbacks.push_back([this]() { this->ProcessActuatorForce(); });

    // Specify functions to calculate FMU outputs (at end of step)
    postStepCallbacks.push_back([this]() { this->CalculateActuatorLength(); });
}

void FmuComponent::ProcessActuatorForce() {
    // Set actuator force (F received from outside)
    const auto& P1 = m_point_ground;
    auto P2 = m_crane->TransformPointLocalToParent(m_point_crane);
    ChVector<> dir = (P2 - P1).GetNormalized();
    ChVector<> force = F * dir;
    m_external_load->SetForce(force, false);
    m_external_load->SetApplicationPoint(P2, false);
}

void FmuComponent::CalculateActuatorLength() {
    const auto& P1 = m_point_ground;
    const auto& V1 = VNULL;

    auto P2 = this->m_crane->TransformPointLocalToParent(m_point_crane);
    auto V2 = this->m_crane->PointSpeedLocalToParent(m_point_crane);

    ChVector<> dir = (P2 - P1).GetNormalized();

    s = (P2 - P1).Length();   // actuator length
    sd = Vdot(dir, V2 - V1);  // actuator length rate
}

void FmuComponent::_preModelDescriptionExport() {
    _exitInitializationMode();
    ////ChArchiveFmu archive_fmu(*this);
    ////archive_fmu << CHNVP(sys);
}

void FmuComponent::_postModelDescriptionExport() {}

void FmuComponent::_enterInitializationMode() {}

void FmuComponent::_exitInitializationMode() {
    // Initialize runtime visualization (if requested and if available)
    if (vis) {
#ifdef CHRONO_IRRLICHT
        sendToLog("Enable run-time visualization", fmi2Status::fmi2OK, "logAll");
        vissys = chrono_types::make_shared<irrlicht::ChVisualSystemIrrlicht>();
        vissys->AttachSystem(&sys);
        vissys->SetWindowSize(800, 600);
        vissys->SetWindowTitle("Hydraulic crane");
        vissys->SetCameraVertical(CameraVerticalDir::Z);
        vissys->Initialize();
        vissys->AddCamera(ChVector<>(0.5, -1, 0.5), ChVector<>(0.5, 0, 0.5));
        vissys->AddTypicalLights();
#else
        sendToLog("Run-time visualization not available", fmi2Status::fmi2OK, "logAll");
#endif
    }

    sys.DoFullAssembly();
}

fmi2Status FmuComponent::_doStep(fmi2Real currentCommunicationPoint,
                                 fmi2Real communicationStepSize,
                                 fmi2Boolean noSetFMUStatePriorToCurrentPoint) {
    while (time < currentCommunicationPoint + communicationStepSize) {
        fmi2Real step_size = std::min((currentCommunicationPoint + communicationStepSize - time),
                                      std::min(communicationStepSize, stepSize));

        if (vis) {
#ifdef CHRONO_IRRLICHT
            vissys->Run();
            vissys->BeginScene(true, true, ChColor(0.33f, 0.6f, 0.78f));
            vissys->Render();
            vissys->EndScene();
#endif
        }

        sys.DoStepDynamics(step_size);
        sendToLog("time: " + std::to_string(time) + "\n", fmi2Status::fmi2OK, "logAll");

        time = time + step_size;
    }

    return fmi2Status::fmi2OK;
}
