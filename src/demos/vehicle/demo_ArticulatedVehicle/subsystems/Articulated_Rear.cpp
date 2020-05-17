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
// Rear of the articulated vehicle model. This base class has a structure similar
// to that of a ChWheeledVehicle.
//
// =============================================================================

#include "chrono/assets/ChBoxShape.h"
#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChColorAsset.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/wheeled_vehicle/ChTire.h"

#include "chrono_models/vehicle/generic/Generic_RigidSuspension.h"
#include "chrono_models/vehicle/generic/Generic_RigidPinnedAxle.h"
#include "chrono_models/vehicle/generic/Generic_Wheel.h"
#include "chrono_models/vehicle/generic/Generic_BrakeSimple.h"

#include "subsystems/Articulated_Rear.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::generic;

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double Articulated_Rear::m_chassisMass = 2000;                 // chassis sprung mass
const ChVector<> Articulated_Rear::m_chassisCOM(0, 0, 0.4);          // COM location
const ChVector<> Articulated_Rear::m_chassisInertia(100, 400, 500);  // chassis inertia (roll,pitch,yaw)

const ChVector<> Articulated_Rear::m_offset(1.0, 0, 0.1);  // connection to front side

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
Articulated_Rear::Articulated_Rear(std::shared_ptr<Articulated_Chassis> front) : m_front(front) {
    // -------------------------------------------
    // Create the chassis body
    // -------------------------------------------
    ChSystem* system = front->GetBody()->GetSystem();
    m_chassis = std::shared_ptr<ChBodyAuxRef>(system->NewBodyAuxRef());

    m_chassis->SetIdentifier(100);
    m_chassis->SetName("rear");
    m_chassis->SetMass(m_chassisMass);
    m_chassis->SetFrame_COG_to_REF(ChFrame<>(m_chassisCOM, ChQuaternion<>(1, 0, 0, 0)));
    m_chassis->SetInertiaXX(m_chassisInertia);

    auto box = chrono_types::make_shared<ChBoxShape>();
    box->GetBoxGeometry().SetLengths(ChVector<>(1.0, 1.0, 0.2));
    box->GetBoxGeometry().Pos = ChVector<>(0.07, 0, 0.1);
    m_chassis->AddAsset(box);

    auto cyl1 = chrono_types::make_shared<ChCylinderShape>();
    cyl1->GetCylinderGeometry().rad = 0.05;
    cyl1->GetCylinderGeometry().p1 = ChVector<>(0.45, 0.45, 0.1);
    cyl1->GetCylinderGeometry().p2 = m_offset;
    m_chassis->AddAsset(cyl1);

    auto cyl2 = chrono_types::make_shared<ChCylinderShape>();
    cyl2->GetCylinderGeometry().rad = 0.05;
    cyl2->GetCylinderGeometry().p1 = ChVector<>(0.45, -0.45, 0.1);
    cyl2->GetCylinderGeometry().p2 = m_offset;
    m_chassis->AddAsset(cyl2);

    m_chassis->AddAsset(chrono_types::make_shared<ChColorAsset>(0.6f, 0.2f, 0.2f));

    system->Add(m_chassis);

    // -------------------------------------------
    // Create the axle subsystem
    // -------------------------------------------
    m_axle = chrono_types::make_shared<ChAxle>();

    ////m_axle->m_suspension = chrono_types::make_shared<Generic_RigidPinnedAxle>("RearSusp");
    m_axle->m_suspension = chrono_types::make_shared<Generic_RigidSuspension>("RearSusp");

    m_axle->m_wheels.resize(2);
    m_axle->m_wheels[0] = chrono_types::make_shared<Generic_Wheel>("Wheel_RL");
    m_axle->m_wheels[1] = chrono_types::make_shared<Generic_Wheel>("Wheel_RR");

    m_axle->m_brake_left = chrono_types::make_shared<Generic_BrakeSimple>("Brake_RL");
    m_axle->m_brake_right = chrono_types::make_shared<Generic_BrakeSimple>("Brake_RR");
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Articulated_Rear::Initialize() {
    // Absolute location of connection point
    ChVector<> connection = m_front->GetConnectionPoint();

    // Calculate position of rear side (assume same orientation as front side)
    const ChQuaternion<>& rot = m_front->GetRot();
    ChVector<> pos = connection - rot.Rotate(m_offset);

    m_chassis->SetFrame_REF_to_abs(ChFrame<>(pos, rot));

    // Initialize the axle subsystem
    m_axle->Initialize(m_chassis, ChVector<>(-0.5, 0, 0), ChVector<>(0, 0, 0), m_chassis, -1, 0.0);

    // Create the connection to the front side.
    m_motor = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    m_motor->SetAngleFunction(chrono_types::make_shared<ChFunction_Const>());
    m_motor->Initialize(m_chassis, m_front->GetBody(), ChFrame<>(connection, rot));
    m_chassis->GetSystem()->AddLink(m_motor);
}

// -----------------------------------------------------------------------------
// Initialize the given tire and attach to the specified wheel
// -----------------------------------------------------------------------------
void Articulated_Rear::InitializeTire(std::shared_ptr<ChTire> tire,
                                      std::shared_ptr<ChWheel> wheel,
                                      VisualizationType tire_vis,
                                      ChTire::CollisionType tire_coll) {
    wheel->SetTire(tire);
    tire->Initialize(wheel);
    tire->SetVisualizationType(tire_vis);
    tire->SetCollisionType(tire_coll);
}

// -----------------------------------------------------------------------------
// Set visualization type for the various subsystems
// -----------------------------------------------------------------------------
void Articulated_Rear::SetSuspensionVisualizationType(VisualizationType vis) {
    m_axle->m_suspension->SetVisualizationType(vis);
}

void Articulated_Rear::SetWheelVisualizationType(VisualizationType vis) {
    for (auto& wheel : m_axle->m_wheels) {
        wheel->SetVisualizationType(vis);
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Articulated_Rear::Synchronize(double time, double steering, double braking, const ChTerrain& terrain) {
    // Synchronize the tires
    for (auto& wheel : m_axle->m_wheels) {
        wheel->GetTire()->Synchronize(time, terrain);
    }

    // Synchronize the vehicle's axle subsystems
    // (this applies tire forces to suspension spindles and braking input)
    m_axle->Synchronize(braking);

    // Apply steering
    double max_angle = CH_C_PI / 6;
    auto fun = std::static_pointer_cast<ChFunction_Const>(m_motor->GetAngleFunction());
    fun->Set_yconst(-max_angle * steering);
}

void Articulated_Rear::Advance(double step) {
    for (auto& wheel : m_axle->m_wheels) {
        wheel->GetTire()->Advance(step);
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
const ChVector<>& Articulated_Rear::GetSpindlePos(VehicleSide side) const {
    return m_axle->m_suspension->GetSpindlePos(side);
}

ChQuaternion<> Articulated_Rear::GetSpindleRot(VehicleSide side) const {
    return m_axle->m_suspension->GetSpindleRot(side);
}

const ChVector<>& Articulated_Rear::GetSpindleLinVel(VehicleSide side) const {
    return m_axle->m_suspension->GetSpindleLinVel(side);
}

ChVector<> Articulated_Rear::GetSpindleAngVel(VehicleSide side) const {
    return m_axle->m_suspension->GetSpindleAngVel(side);
}
