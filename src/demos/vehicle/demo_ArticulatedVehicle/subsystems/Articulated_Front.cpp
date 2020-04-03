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
// Front of the articulated vehicle model. Implemented as a ChWheeledVehicle.
//
// =============================================================================

#include "subsystems/Articulated_Front.h"

#include "chrono_models/vehicle/generic/Generic_RigidSuspension.h"
#include "chrono_models/vehicle/generic/Generic_Wheel.h"
#include "chrono_models/vehicle/generic/Generic_Driveline2WD.h"
#include "chrono_models/vehicle/generic/Generic_BrakeSimple.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::generic;

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double Articulated_Chassis::m_mass = 2000;
const ChVector<> Articulated_Chassis::m_inertiaXX(100, 400, 500);
const ChVector<> Articulated_Chassis::m_inertiaXY(0, 0, 0);
const ChVector<> Articulated_Chassis::m_COM_loc(0, 0, 0.4);

const ChCoordsys<> Articulated_Chassis::m_driverCsys(ChVector<>(0.0, 0.5, 1.2), ChQuaternion<>(1, 0, 0, 0));

const ChVector<> Articulated_Chassis::m_offset(-1.0, 0, 0.1);

// -----------------------------------------------------------------------------
// Chassis of the front side of the articulated vehicle
// -----------------------------------------------------------------------------
Articulated_Chassis::Articulated_Chassis(const std::string& name, bool fixed) : ChRigidChassis(name, fixed) {
    m_inertia(0, 0) = m_inertiaXX.x();
    m_inertia(1, 1) = m_inertiaXX.y();
    m_inertia(2, 2) = m_inertiaXX.z();

    m_inertia(0, 1) = m_inertiaXY.x();
    m_inertia(0, 2) = m_inertiaXY.y();
    m_inertia(1, 2) = m_inertiaXY.z();
    m_inertia(1, 0) = m_inertiaXY.x();
    m_inertia(2, 0) = m_inertiaXY.y();
    m_inertia(2, 1) = m_inertiaXY.z();

    // Visualization primitives
    BoxShape box(ChVector<>(-0.25, 0.0, 0.1), ChQuaternion<>(1, 0, 0, 0), ChVector<>(1.5, 1.0, 0.2));
    CylinderShape cyl1(m_offset, Q_from_AngX(CH_C_PI_2), 0.1, 0.2);
    CylinderShape cyl2(ChVector<>(0.5, 0, 0), QUNIT, 0.05, 2);

    m_has_primitives = true;
    m_vis_boxes.push_back(box);
    m_vis_cylinders.push_back(cyl1);
    m_vis_cylinders.push_back(cyl2);
}

ChVector<> Articulated_Chassis::GetConnectionPoint() const {
    return m_body->GetFrame_REF_to_abs().TransformPointLocalToParent(m_offset);
}

// -----------------------------------------------------------------------------
// Front side of the articulated vehicle
// -----------------------------------------------------------------------------
Articulated_Front::Articulated_Front(const bool fixed, ChContactMethod contactMethod)
    : ChWheeledVehicle("ArticulatedVehicle", contactMethod) {
    // Create the chassis subsystem
    m_chassis = chrono_types::make_shared<Articulated_Chassis>("Chassis", fixed);

    // Create the axle subsystem (suspension + wheels + brakes)
    m_axles.resize(1);
    m_axles[0] = chrono_types::make_shared<ChAxle>();
    m_axles[0]->m_suspension = chrono_types::make_shared<Generic_RigidSuspension>("FrontSusp");
    m_axles[0]->m_wheels.resize(2);
    m_axles[0]->m_wheels[0] = chrono_types::make_shared<Generic_Wheel>("Wheel_FL");
    m_axles[0]->m_wheels[1] = chrono_types::make_shared<Generic_Wheel>("Wheel_FR");
    m_axles[0]->m_brake_left = chrono_types::make_shared<Generic_BrakeSimple>("Brake_FL");
    m_axles[0]->m_brake_right = chrono_types::make_shared<Generic_BrakeSimple>("Brake_FR");

    // Create the driveline
    m_driveline = chrono_types::make_shared<Generic_Driveline2WD>("driveline");
}

void Articulated_Front::Initialize(const ChCoordsys<>& chassisPos, double chassisFwdVel) {
    // Initialize the chassis subsystem.
    m_chassis->Initialize(m_system, chassisPos, chassisFwdVel, WheeledCollisionFamily::CHASSIS);

    // Initialize the axle subsystem
    m_axles[0]->Initialize(m_chassis->GetBody(), ChVector<>(0.5, 0, 0), ChVector<>(0, 0, 0), m_chassis->GetBody(), -1,
                           0.0);

    // Initialize the driveline subsystem
    std::vector<int> driven_axles = {0};
    m_driveline->Initialize(m_chassis->GetBody(), m_axles, driven_axles);
}
