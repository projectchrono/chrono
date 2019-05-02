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

    auto box = std::make_shared<ChBoxShape>();
    box->GetBoxGeometry().SetLengths(ChVector<>(1.0, 1.0, 0.2));
    box->GetBoxGeometry().Pos = ChVector<>(0.07, 0, 0.1);
    m_chassis->AddAsset(box);

    auto cyl1 = std::make_shared<ChCylinderShape>();
    cyl1->GetCylinderGeometry().rad = 0.05;
    cyl1->GetCylinderGeometry().p1 = ChVector<>(0.45, 0.45, 0.1);
    cyl1->GetCylinderGeometry().p2 = m_offset;
    m_chassis->AddAsset(cyl1);

    auto cyl2 = std::make_shared<ChCylinderShape>();
    cyl2->GetCylinderGeometry().rad = 0.05;
    cyl2->GetCylinderGeometry().p1 = ChVector<>(0.45, -0.45, 0.1);
    cyl2->GetCylinderGeometry().p2 = m_offset;
    m_chassis->AddAsset(cyl2);

    m_chassis->AddAsset(std::make_shared<ChColorAsset>(0.6f, 0.2f, 0.2f));

    system->Add(m_chassis);

    // -------------------------------------------
    // Create the suspension subsystems
    // -------------------------------------------
    m_suspensions.resize(1);
    ////m_suspensions[0] = std::make_shared<Generic_RigidPinnedAxle>("RearSusp");
    m_suspensions[0] = std::make_shared<Generic_RigidSuspension>("RearSusp");

    // -----------------
    // Create the wheels
    // -----------------
    m_wheels.resize(2);
    m_wheels[0] = std::make_shared<Generic_Wheel>("Wheel_FL");
    m_wheels[1] = std::make_shared<Generic_Wheel>("Wheel_FR");

    // -----------------
    // Create the brakes
    // -----------------
    m_brakes.resize(2);
    m_brakes[0] = std::make_shared<Generic_BrakeSimple>("Brake_FL");
    m_brakes[1] = std::make_shared<Generic_BrakeSimple>("Brake_FR");
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

    // Initialize the suspension subsystems (specify the suspension subsystems'
    // frames relative to the chassis reference frame).
    m_suspensions[0]->Initialize(m_chassis, ChVector<>(-0.5, 0, 0), m_chassis, -1);

    // Initialize wheels
    m_wheels[0]->Initialize(m_suspensions[0]->GetSpindle(LEFT));
    m_wheels[1]->Initialize(m_suspensions[0]->GetSpindle(RIGHT));

    // Initialize the four brakes
    m_brakes[0]->Initialize(m_suspensions[0]->GetRevolute(LEFT));
    m_brakes[1]->Initialize(m_suspensions[0]->GetRevolute(RIGHT));

    // Create the connection to the front side.
    m_motor = std::make_shared<ChLinkMotorRotationAngle>();
    m_motor->SetAngleFunction(std::make_shared<ChFunction_Const>());
    m_motor->Initialize(m_chassis, m_front->GetBody(), ChFrame<>(connection, rot));
    m_chassis->GetSystem()->AddLink(m_motor);
}

// -----------------------------------------------------------------------------
// Set visualization type for the various subsystems
// -----------------------------------------------------------------------------
void Articulated_Rear::SetSuspensionVisualizationType(VisualizationType vis) {
    for (size_t i = 0; i < m_suspensions.size(); ++i) {
        m_suspensions[i]->SetVisualizationType(vis);
    }
}

void Articulated_Rear::SetWheelVisualizationType(VisualizationType vis) {
    for (size_t i = 0; i < m_wheels.size(); ++i) {
        m_wheels[i]->SetVisualizationType(vis);
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Articulated_Rear::Synchronize(double time, double steering, double braking, const TerrainForces& tire_forces) {
    // Apply tire forces to spindle bodies.
    m_suspensions[0]->Synchronize(LEFT, tire_forces[FRONT_LEFT.id()]);
    m_suspensions[0]->Synchronize(RIGHT, tire_forces[FRONT_RIGHT.id()]);

    // Apply braking
    m_brakes[0]->Synchronize(braking);
    m_brakes[1]->Synchronize(braking);

    // Apply steering
    double max_angle = CH_C_PI / 6;
    auto fun = std::static_pointer_cast<ChFunction_Const>(m_motor->GetAngleFunction());
    fun->Set_yconst(-max_angle * steering);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
std::shared_ptr<ChBody> Articulated_Rear::GetWheelBody(const WheelID& wheel_id) const {
    return m_suspensions[wheel_id.axle()]->GetSpindle(wheel_id.side());
}

const ChVector<>& Articulated_Rear::GetWheelPos(const WheelID& wheel_id) const {
    return m_suspensions[wheel_id.axle()]->GetSpindlePos(wheel_id.side());
}

const ChQuaternion<>& Articulated_Rear::GetWheelRot(const WheelID& wheel_id) const {
    return m_suspensions[wheel_id.axle()]->GetSpindleRot(wheel_id.side());
}

const ChVector<>& Articulated_Rear::GetWheelLinVel(const WheelID& wheel_id) const {
    return m_suspensions[wheel_id.axle()]->GetSpindleLinVel(wheel_id.side());
}

ChVector<> Articulated_Rear::GetWheelAngVel(const WheelID& wheel_id) const {
    return m_suspensions[wheel_id.axle()]->GetSpindleAngVel(wheel_id.side());
}

WheelState Articulated_Rear::GetWheelState(const WheelID& wheel_id) const {
    WheelState state;

    state.pos = GetWheelPos(wheel_id);
    state.rot = GetWheelRot(wheel_id);
    state.lin_vel = GetWheelLinVel(wheel_id);
    state.ang_vel = GetWheelAngVel(wheel_id);

    ChVector<> ang_vel_loc = state.rot.RotateBack(state.ang_vel);
    state.omega = ang_vel_loc.y();

    return state;
}
