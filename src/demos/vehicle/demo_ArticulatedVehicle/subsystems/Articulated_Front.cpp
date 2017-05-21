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
    m_inertia.SetElement(0, 0, m_inertiaXX.x());
    m_inertia.SetElement(1, 1, m_inertiaXX.y());
    m_inertia.SetElement(2, 2, m_inertiaXX.z());

    m_inertia.SetElement(0, 1, m_inertiaXY.x());
    m_inertia.SetElement(0, 2, m_inertiaXY.y());
    m_inertia.SetElement(1, 2, m_inertiaXY.z());
    m_inertia.SetElement(1, 0, m_inertiaXY.x());
    m_inertia.SetElement(2, 0, m_inertiaXY.y());
    m_inertia.SetElement(2, 1, m_inertiaXY.z());

    // Visualization primitives
    BoxShape box(ChVector<>(0.0, 0.0, 0.1), ChQuaternion<>(1, 0, 0, 0), ChVector<>(2.0, 1.0, 0.2));
    CylinderShape cyl(m_offset, Q_from_AngX(CH_C_PI_2), 0.1, 0.2);

    m_has_primitives = true;
    m_vis_boxes.push_back(box);
    m_vis_cylinders.push_back(cyl);
}

ChVector<> Articulated_Chassis::GetConnectionPoint() const {
    return m_body->GetFrame_REF_to_abs().TransformPointLocalToParent(m_offset);
}

// -----------------------------------------------------------------------------
// Front side of the articulated vehicle
// -----------------------------------------------------------------------------
Articulated_Front::Articulated_Front(const bool fixed, ChMaterialSurface::ContactMethod contactMethod)
    : ChWheeledVehicle(contactMethod) {
    // -------------------------------------------
    // Create the chassis subsystem
    // -------------------------------------------
    m_chassis = std::make_shared<Articulated_Chassis>("Chassis", fixed);

    // -------------------------------------------
    // Create the suspension subsystems
    // -------------------------------------------
    m_suspensions.resize(1);
    m_suspensions[0] = std::make_shared<Generic_RigidSuspension>("FrontSusp");

    // -----------------
    // Create the wheels
    // -----------------
    m_wheels.resize(2);
    m_wheels[0] = std::make_shared<Generic_Wheel>("Wheel_FL");
    m_wheels[1] = std::make_shared<Generic_Wheel>("Wheel_FR");

    // --------------------
    // Create the driveline
    // --------------------
    m_driveline = std::make_shared<Generic_Driveline2WD>("driveline");

    // -----------------
    // Create the brakes
    // -----------------
    m_brakes.resize(2);
    m_brakes[0] = std::make_shared<Generic_BrakeSimple>("Brake_FL");
    m_brakes[1] = std::make_shared<Generic_BrakeSimple>("Brake_FR");
}

void Articulated_Front::Initialize(const ChCoordsys<>& chassisPos, double chassisFwdVel) {
    // Invoke base class method to initialize the chassis.
    ChWheeledVehicle::Initialize(chassisPos, chassisFwdVel);

    // Initialize the suspension subsystem (specify the suspension's frame, relative
    // to the chassis reference frame).
    m_suspensions[0]->Initialize(m_chassis->GetBody(), ChVector<>(0.5, 0, 0), m_chassis->GetBody());

    // Initialize wheels
    m_wheels[0]->Initialize(m_suspensions[0]->GetSpindle(LEFT));
    m_wheels[1]->Initialize(m_suspensions[0]->GetSpindle(RIGHT));

    // Initialize the driveline subsystem (RWD)
    std::vector<int> driven_susp(1, 0);
    m_driveline->Initialize(m_chassis->GetBody(), m_suspensions, driven_susp);

    // Initialize the brakes
    m_brakes[0]->Initialize(m_suspensions[0]->GetRevolute(LEFT));
    m_brakes[1]->Initialize(m_suspensions[0]->GetRevolute(RIGHT));
}
