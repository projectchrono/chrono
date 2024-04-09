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
// External project template for building a Chrono co-simulation FMU.
// =============================================================================

#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMate.h"

#include "FmuComponentChrono.h"

using namespace chrono;

FmuComponent::FmuComponent(fmi2String instanceName,
                           fmi2Type fmuType,
                           fmi2String fmuGUID,
                           fmi2String fmuResourceLocation,
                           const fmi2CallbackFunctions* functions,
                           fmi2Boolean visible,
                           fmi2Boolean loggingOn)
    : FmuChronoComponentBase(instanceName, fmuType, fmuGUID, fmuResourceLocation, functions, visible, loggingOn) {
    initializeType(fmuType);

    SetChronoDataPath(std::string(m_resources_location));

    auto x_tt_funpair = MAKE_GETSET_PAIR(fmi2Real, { return this->sys.SearchBody("cart")->GetPosDt2().x(); }, {});
    auto x_t_funpair = MAKE_GETSET_PAIR(fmi2Real, { return this->sys.SearchBody("cart")->GetPosDt().x(); }, {});
    auto x_funpair = MAKE_GETSET_PAIR(fmi2Real, { return this->sys.SearchBody("cart")->GetPos().x(); }, {});

    auto theta_tt_funpair =
        MAKE_GETSET_PAIR(fmi2Real,
                         {
                             return std::dynamic_pointer_cast<ChLinkMateRevolute>(this->sys.SearchLink("pendulum_rev"))
                                 ->GetRelativeAngleDt2();
                         },
                         {});
    auto theta_t_funpair =
        MAKE_GETSET_PAIR(fmi2Real,
                         {
                             return std::dynamic_pointer_cast<ChLinkMateRevolute>(this->sys.SearchLink("pendulum_rev"))
                                 ->GetRelativeAngleDt();
                         },
                         {});
    auto theta_funpair =
        MAKE_GETSET_PAIR(fmi2Real,
                         {
                             return std::dynamic_pointer_cast<ChLinkMateRevolute>(this->sys.SearchLink("pendulum_rev"))
                                 ->GetRelativeAngle();
                         },
                         {});

    /// FMU_ACTION: declare relevant variables
    AddFmuVariable(x_tt_funpair, "x_tt", FmuVariable::Type::Real, "m/s2", "cart acceleration");
    AddFmuVariable(x_t_funpair, "x_t", FmuVariable::Type::Real, "m/s", "cart velocity");
    AddFmuVariable(x_funpair, "x", FmuVariable::Type::Real, "m", "cart position");
    AddFmuVariable(theta_tt_funpair, "theta_tt", FmuVariable::Type::Real, "rad/s2", "pendulum ang acceleration");
    AddFmuVariable(theta_t_funpair, "theta_t", FmuVariable::Type::Real, "rad/s", "pendulum ang velocity");
    AddFmuVariable(theta_funpair, "theta", FmuVariable::Type::Real, "rad", "pendulum angle");
    AddFmuVariable(&pendulum_length, "pendulum_length", FmuVariable::Type::Real, "m", "pendulum length",
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);
    AddFmuVariable(&cart_mass, "cart_mass", FmuVariable::Type::Real, "kg", "pendulum mass",
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);
    AddFmuVariable(&pendulum_mass, "pendulum_mass", FmuVariable::Type::Real, "kg", "cart mass",
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);

#ifdef CHRONO_IRRLICHT
    if (visible == fmi2True) {
        vis = chrono_types::make_shared<irrlicht::ChVisualSystemIrrlicht>();
        vis->AttachSystem(&sys);
        vis->SetWindowSize(800, 600);
        vis->SetWindowTitle("CartPendulumFMU");
        vis->Initialize();
        vis->AddLogo();
        vis->AddSkyBox();
        vis->AddCamera(ChVector3d(-0.5, -0.5, -1.0));
        vis->AddTypicalLights();
    }
#endif
};

void FmuComponent::_preModelDescriptionExport() {
    _exitInitializationMode();
}

void FmuComponent::_postModelDescriptionExport() {}

void FmuComponent::_exitInitializationMode() {
    sys.Clear();

    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetFixed(true);
    sys.Add(ground);

    // Cart is moving along X axis, Pendulum rotates along Z axis

    auto cart = chrono_types::make_shared<ChBodyEasyBox>(0.2, 0.1, 0.1, 750, true, false);
    cart->SetName("cart");
    cart->SetPos(VNULL);
    cart->SetRot(QUNIT);
    cart->SetMass(cart_mass);
    cart->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/blue.png"));
    sys.Add(cart);

    double rotZ = 15 * CH_DEG_TO_RAD;
    auto pendulum = chrono_types::make_shared<ChBodyEasyBox>(0.025, pendulum_length, 0.01, 750, true, false);
    pendulum->SetName("pendulum");
    pendulum->SetMass(pendulum_mass);
    pendulum->SetRot(QuatFromAngleZ(rotZ));
    pendulum->SetPos(ChVector3d(pendulum_length / 2.0 * sin(rotZ), pendulum_length / 2.0 * cos(rotZ), 0.0));
    pendulum->SetInertiaXX(ChVector3d(0.01, 0.01, 0.01));
    pendulum->GetVisualShape(0)->SetColor(ChColor(0.8f, 0.8f, 0.0f));
    sys.Add(pendulum);

    auto cart_prism = chrono_types::make_shared<ChLinkLockPrismatic>();
    cart_prism->Initialize(cart, ground, ChFrame<>(VNULL, Q_ROTATE_Z_TO_X));
    cart_prism->SetName("cart_prism");
    sys.Add(cart_prism);

    auto pendulum_rev = chrono_types::make_shared<ChLinkMateRevolute>();
    pendulum_rev->Initialize(pendulum, cart, true, ChFrame<>(ChVector3d(0, -pendulum_length / 2, 0), QUNIT),
                             ChFrame<>(VNULL, QUNIT));
    pendulum_rev->SetName("pendulum_rev");
    sys.Add(pendulum_rev);

    sys.DoAssembly(AssemblyLevel::FULL);

#ifdef CHRONO_IRRLICHT
    if (vis)
        vis->BindAll();
#endif

    ChOutputFMU archive_fmu(*this);
    archive_fmu << CHNVP(sys);
};

fmi2Status FmuComponent::_doStep(fmi2Real currentCommunicationPoint,
                                 fmi2Real communicationStepSize,
                                 fmi2Boolean noSetFMUStatePriorToCurrentPoint) {
    while (m_time < currentCommunicationPoint + communicationStepSize) {
        fmi2Real step_size = std::min((currentCommunicationPoint + communicationStepSize - m_time),
                                      std::min(communicationStepSize, m_stepSize));

#ifdef CHRONO_IRRLICHT
        if (vis) {
            vis->Run();
            vis->BeginScene();
            vis->Render();
            vis->EndScene();
        }
#endif

        sys.DoStepDynamics(step_size);
        sendToLog("Step at time: " + std::to_string(m_time) + " with timestep: " + std::to_string(step_size) +
                      "ms succeeded.\n",
                  fmi2Status::fmi2OK, "logAll");

        m_time += step_size;

        realtime_timer.Spin(step_size);
    }

    return fmi2Status::fmi2OK;
}
