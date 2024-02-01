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
// Authors: Radu Serban
// =============================================================================
//
// Wrapper classes for modeling an entire UAZBUS vehicle assembly
// (including the vehicle itself, the powertrain, and the tires).
//
// =============================================================================

#include "chrono/ChConfig.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/wheeled_vehicle/driveline/ChShaftsDriveline4WD.h"

#include "chrono_vehicle/ChPowertrainAssembly.h"

#include "chrono_models/vehicle/kraz/Kraz.h"

namespace chrono {
namespace vehicle {
namespace kraz {

Kraz::Kraz()
    : m_system(nullptr),
      m_tractor(nullptr),
      m_trailer(nullptr),
      m_contactMethod(ChContactMethod::NSC),
      m_chassisCollisionType(CollisionType::NONE),
      m_fixed(false),
      m_engineType(EngineModelType::SIMPLE_MAP),
      m_transmissionType(TransmissionModelType::AUTOMATIC_SIMPLE_MAP),
      m_tire_step_size(-1),
      m_initFwdVel(0),
      m_initPos(ChCoordsys<>(ChVector<>(0, 0, 1), QUNIT)),
      m_initOmega({0, 0, 0, 0}) {}

Kraz::Kraz(ChSystem* system)
    : m_system(system),
      m_tractor(nullptr),
      m_trailer(nullptr),
      m_contactMethod(ChContactMethod::NSC),
      m_chassisCollisionType(CollisionType::NONE),
      m_fixed(false),
      m_engineType(EngineModelType::SIMPLE_MAP),
      m_transmissionType(TransmissionModelType::AUTOMATIC_SIMPLE_MAP),
      m_tire_step_size(-1),
      m_initFwdVel(0),
      m_initPos(ChCoordsys<>(ChVector<>(0, 0, 1), QUNIT)),
      m_initOmega({0, 0, 0, 0}) {}

Kraz::~Kraz() {
    delete m_tractor;
    delete m_trailer;
}

void Kraz::SetChassisVisualizationType(VisualizationType vis_tractor, VisualizationType vis_trailer) {
    m_tractor->SetChassisVisualizationType(vis_tractor);
    m_trailer->SetChassisVisualizationType(vis_trailer);
}

void Kraz::SetSuspensionVisualizationType(VisualizationType vis_tractor, VisualizationType vis_trailer) {
    m_tractor->SetSuspensionVisualizationType(vis_tractor);
    m_trailer->SetSuspensionVisualizationType(vis_trailer);
}

void Kraz::SetSteeringVisualizationType(VisualizationType vis) {
    m_tractor->SetSteeringVisualizationType(vis);
}

void Kraz::SetWheelVisualizationType(VisualizationType vis_tractor, VisualizationType vis_trailer) {
    m_tractor->SetWheelVisualizationType(vis_tractor);
    m_trailer->SetWheelVisualizationType(vis_trailer);
}

void Kraz::SetTireVisualizationType(VisualizationType vis_tractor, VisualizationType vis_trailer) {
    m_tractor->SetTireVisualizationType(vis_tractor);
    m_trailer->SetTireVisualizationType(vis_trailer);
}

void Kraz::Initialize() {
    // Create and initialize the tractor
    m_tractor = m_system ? new Kraz_tractor(m_system, m_fixed) : new Kraz_tractor(m_fixed, m_contactMethod);
    m_tractor->Initialize(m_initPos, m_initFwdVel);

    auto drvLine = std::static_pointer_cast<ChShaftsDriveline4WD>(m_tractor->GetDriveline());
    drvLine->LockCentralDifferential(0, false);

    // Create and initialize the trailer
    m_trailer = new Kraz_trailer(m_system);
    m_trailer->Initialize(m_tractor->GetChassis());

    // Create and initialize the powertrain system
    std::shared_ptr<ChEngine> engine;
    std::shared_ptr<ChTransmission> transmission;
    switch (m_engineType) {
        case EngineModelType::SHAFTS:
            // engine = chrono_types::make_shared<Kraz_tractor_EngineShafts>("Engine");
            break;
        case EngineModelType::SIMPLE_MAP:
            engine = chrono_types::make_shared<Kraz_tractor_EngineSimpleMap>("Engine");
            break;
        case EngineModelType::SIMPLE:
            // engine = chrono_types::make_shared<Kraz_tractor_EngineSimple>("Engine");
            break;
    }

    switch (m_transmissionType) {
        case TransmissionModelType::AUTOMATIC_SHAFTS:
            // transmission = chrono_types::make_shared<Kraz_tractor_AutomaticTransmissionShafts>("Transmission");
            break;
        case TransmissionModelType::AUTOMATIC_SIMPLE_MAP:
            transmission = chrono_types::make_shared<Kraz_tractor_AutomaticTransmissionSimpleMap>("Transmission");
            break;
    }

    if (engine && transmission) {
        auto powertrain = chrono_types::make_shared<ChPowertrainAssembly>(engine, transmission);
        m_tractor->InitializePowertrain(powertrain);
    }

    // Create the tractor tires
    auto tire_FL = chrono_types::make_shared<Kraz_tractor_Tire>("TractorTire_FL");
    auto tire_FR = chrono_types::make_shared<Kraz_tractor_Tire>("TractorTire_FR");

    auto tire_RL1i = chrono_types::make_shared<Kraz_tractor_Tire>("TractorTire_RL1i");
    auto tire_RR1i = chrono_types::make_shared<Kraz_tractor_Tire>("TractorTire_RR1i");
    auto tire_RL1o = chrono_types::make_shared<Kraz_tractor_Tire>("TractorTire_RL1o");
    auto tire_RR1o = chrono_types::make_shared<Kraz_tractor_Tire>("TractorTire_RR1o");

    auto tire_RL2i = chrono_types::make_shared<Kraz_tractor_Tire>("TractorTire_RL2i");
    auto tire_RR2i = chrono_types::make_shared<Kraz_tractor_Tire>("TractorTire_RR2i");
    auto tire_RL2o = chrono_types::make_shared<Kraz_tractor_Tire>("TractorTire_RL2o");
    auto tire_RR2o = chrono_types::make_shared<Kraz_tractor_Tire>("TractorTire_RR2o");

    m_tractor->InitializeTire(tire_FL, m_tractor->GetAxle(0)->m_wheels[0], VisualizationType::NONE);
    m_tractor->InitializeTire(tire_FR, m_tractor->GetAxle(0)->m_wheels[1], VisualizationType::NONE);

    m_tractor->InitializeTire(tire_RL1i, m_tractor->GetAxle(1)->m_wheels[0], VisualizationType::NONE);
    m_tractor->InitializeTire(tire_RR1i, m_tractor->GetAxle(1)->m_wheels[1], VisualizationType::NONE);
    m_tractor->InitializeTire(tire_RL1o, m_tractor->GetAxle(1)->m_wheels[2], VisualizationType::NONE);
    m_tractor->InitializeTire(tire_RR1o, m_tractor->GetAxle(1)->m_wheels[3], VisualizationType::NONE);

    m_tractor->InitializeTire(tire_RL2i, m_tractor->GetAxle(2)->m_wheels[0], VisualizationType::NONE);
    m_tractor->InitializeTire(tire_RR2i, m_tractor->GetAxle(2)->m_wheels[1], VisualizationType::NONE);
    m_tractor->InitializeTire(tire_RL2o, m_tractor->GetAxle(2)->m_wheels[2], VisualizationType::NONE);
    m_tractor->InitializeTire(tire_RR2o, m_tractor->GetAxle(2)->m_wheels[3], VisualizationType::NONE);

    // Create the trailer tires
    auto tr_tire_FL = chrono_types::make_shared<Kraz_trailer_Tire>("FL");
    auto tr_tire_FR = chrono_types::make_shared<Kraz_trailer_Tire>("FR");
    auto tr_tire_ML = chrono_types::make_shared<Kraz_trailer_Tire>("ML");
    auto tr_tire_MR = chrono_types::make_shared<Kraz_trailer_Tire>("MR");
    auto tr_tire_RL = chrono_types::make_shared<Kraz_trailer_Tire>("RL");
    auto tr_tire_RR = chrono_types::make_shared<Kraz_trailer_Tire>("RR");

    m_trailer->InitializeTire(tr_tire_FL, m_trailer->GetAxle(0)->m_wheels[0], VisualizationType::NONE);
    m_trailer->InitializeTire(tr_tire_FR, m_trailer->GetAxle(0)->m_wheels[1], VisualizationType::NONE);
    m_trailer->InitializeTire(tr_tire_ML, m_trailer->GetAxle(1)->m_wheels[0], VisualizationType::NONE);
    m_trailer->InitializeTire(tr_tire_MR, m_trailer->GetAxle(1)->m_wheels[1], VisualizationType::NONE);
    m_trailer->InitializeTire(tr_tire_RL, m_trailer->GetAxle(2)->m_wheels[0], VisualizationType::NONE);
    m_trailer->InitializeTire(tr_tire_RR, m_trailer->GetAxle(2)->m_wheels[1], VisualizationType::NONE);

    for (auto& axle : m_tractor->GetAxles()) {
        for (auto& wheel : axle->GetWheels()) {
            if (m_tire_step_size > 0)
                wheel->GetTire()->SetStepsize(m_tire_step_size);
        }
    }

    for (auto& axle : m_trailer->GetAxles()) {
        for (auto& wheel : axle->GetWheels()) {
            if (m_tire_step_size > 0)
                wheel->GetTire()->SetStepsize(m_tire_step_size);
        }
    }

    // Recalculate vehicle mass, to properly account for all subsystems
    m_tractor->InitializeInertiaProperties();
}

void Kraz::Synchronize(double time, const DriverInputs& driver_inputs, const ChTerrain& terrain) {
    m_tractor->Synchronize(time, driver_inputs, terrain);
    m_trailer->Synchronize(time, driver_inputs, terrain);
}

void Kraz::Advance(double step) {
    m_tractor->Advance(step);
    m_trailer->Advance(step);
}

}  // end namespace kraz
}  // end namespace vehicle
}  // end namespace chrono
