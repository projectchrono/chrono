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
// Authors: Alessandro Tasora
// =============================================================================
//
// Trailer for articulated vehicle model.
//
// =============================================================================

#include "chrono/assets/ChSphereShape.h"
#include "chrono/assets/ChBoxShape.h"

#include "chrono_vehicle/wheeled_vehicle/ChTire.h"

#include "subsystems/SemiTrailer.h"
#include "subsystems/SemiTrailer_axle.h"
#include "subsystems/SemiTrailer_wheel.h"
#include "subsystems/SemiTrailer_brake.h"

using namespace chrono;
using namespace chrono::vehicle;

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double SemiTrailer::m_chassisMass = 20000;                        // chassis sprung mass
const ChVector<> SemiTrailer::m_chassisCOM(-6, 0, 0.8);                 // COM location
const ChVector<> SemiTrailer::m_chassisInertia(23904, 322240, 320011);  // chassis inertia (roll,pitch,yaw)
const ChVector<> SemiTrailer::m_chassisPullerJoint(-4.64, 0, 0.8);      // joint between fifth wheel and trailer chassis

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
SemiTrailer::SemiTrailer(ChSystem* mysystem, const bool fixed) {
    // -------------------------------------------
    // Create the chassis body
    // -------------------------------------------
    m_chassis = std::shared_ptr<ChBodyAuxRef>(mysystem->NewBodyAuxRef());

    m_chassis->SetIdentifier(100);
    m_chassis->SetName("trailer");
    m_chassis->SetMass(m_chassisMass);
    m_chassis->SetFrame_COG_to_REF(ChFrame<>(m_chassisCOM, ChQuaternion<>(1, 0, 0, 0)));
    m_chassis->SetInertiaXX(m_chassisInertia);
    m_chassis->SetBodyFixed(fixed);

    auto sphere = chrono_types::make_shared<ChSphereShape>();
    sphere->GetSphereGeometry().rad = 0.1;
    sphere->Pos = m_chassisCOM;
    m_chassis->AddAsset(sphere);

    auto boxB = chrono_types::make_shared<ChBoxShape>();
    boxB->Pos = ChVector<>(-5, 0, 1.4 + 0.7);
    boxB->GetBoxGeometry().SetLengths(ChVector<>(13.62, 2.55, 2.8));
    m_chassis->AddAsset(boxB);

    mysystem->Add(m_chassis);

    // -------------------------------------------
    // Create the axle subsystems
    // -------------------------------------------
    m_axles.resize(3);
    m_axles[0] = chrono_types::make_shared<ChAxle>();
    m_axles[1] = chrono_types::make_shared<ChAxle>();
    m_axles[2] = chrono_types::make_shared<ChAxle>();

    m_axles[0]->m_suspension = chrono_types::make_shared<SemiTrailer_axle>("FrontSusp");
    m_axles[1]->m_suspension = chrono_types::make_shared<SemiTrailer_axle>("MidSusp");
    m_axles[2]->m_suspension = chrono_types::make_shared<SemiTrailer_axle>("RearSusp");

    // -----------------
    // Create the wheels
    // -----------------
    m_axles[0]->m_wheels.resize(2);
    m_axles[0]->m_wheels[0] = chrono_types::make_shared<SemiTrailer_wheel>("Wheel_FL");
    m_axles[0]->m_wheels[1] = chrono_types::make_shared<SemiTrailer_wheel>("Wheel_FR");
    m_axles[1]->m_wheels.resize(2);
    m_axles[1]->m_wheels[0] = chrono_types::make_shared<SemiTrailer_wheel>("Wheel_ML");
    m_axles[1]->m_wheels[1] = chrono_types::make_shared<SemiTrailer_wheel>("Wheel_MR");
    m_axles[2]->m_wheels.resize(2);
    m_axles[2]->m_wheels[0] = chrono_types::make_shared<SemiTrailer_wheel>("Wheel_RL");
    m_axles[2]->m_wheels[1] = chrono_types::make_shared<SemiTrailer_wheel>("Wheel_RR");

    // -----------------
    // Create the brakes
    // -----------------
    m_axles[0]->m_brake_left = chrono_types::make_shared<SemiTrailer_brake>("Brake_FL");
    m_axles[0]->m_brake_right = chrono_types::make_shared<SemiTrailer_brake>("Brake_FR");
    m_axles[1]->m_brake_left = chrono_types::make_shared<SemiTrailer_brake>("Brake_ML");
    m_axles[1]->m_brake_right = chrono_types::make_shared<SemiTrailer_brake>("Brake_MR");
    m_axles[2]->m_brake_left = chrono_types::make_shared<SemiTrailer_brake>("Brake_RL");
    m_axles[2]->m_brake_right = chrono_types::make_shared<SemiTrailer_brake>("Brake_RR");
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void SemiTrailer::Initialize(const ChCoordsys<>& chassisPos,
                             const bool connect_to_puller,
                             std::shared_ptr<chrono::ChBodyAuxRef> pulling_vehicle) {
    m_chassis->SetFrame_REF_to_abs(ChFrame<>(chassisPos));
    // -------------------------------------------
    // Create the frontaxle - puller chassis joint
    // -------------------------------------------

    if (connect_to_puller) {
        ChCoordsys<> hitch_sys;
        hitch_sys.pos = ChVector<>(-4.68, 0, 0.8);
        hitch_sys.rot = ChQuaternion<>(1, 0, 0, 0);
        m_puller = chrono_types::make_shared<ChLinkLockSpherical>();
        m_puller->SetNameString("kingpin");
        m_puller->Initialize(this->m_chassis, pulling_vehicle, hitch_sys);
        pulling_vehicle->GetSystem()->Add(m_puller);
    }

    // Initialize the axle subsystems.
    m_axles[0]->Initialize(m_chassis, ChVector<>(-6.32, 0, 0), ChVector<>(0), m_chassis, -1, 0.0);
    m_axles[1]->Initialize(m_chassis, ChVector<>(-7.63, 0, 0), ChVector<>(0), m_chassis, -1, 0.0);
    m_axles[2]->Initialize(m_chassis, ChVector<>(-8.94, 0, 0), ChVector<>(0), m_chassis, -1, 0.0);
}

// -----------------------------------------------------------------------------
// Initialize the given tire and attach to the specified wheel
// -----------------------------------------------------------------------------
void SemiTrailer::InitializeTire(std::shared_ptr<ChTire> tire,
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
void SemiTrailer::SetSuspensionVisualizationType(VisualizationType vis) {
    for (auto& axle : m_axles) {
        axle->m_suspension->SetVisualizationType(vis);
    }
}

void SemiTrailer::SetWheelVisualizationType(VisualizationType vis) {
    for (auto& axle : m_axles) {
        for (auto& wheel : axle->m_wheels) {
            wheel->SetVisualizationType(vis);
        }
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double SemiTrailer::GetSpringForce(int axle, VehicleSide side) const {
    return std::static_pointer_cast<ChLeafspringAxle>(m_axles[axle]->m_suspension)->GetSpringForce(side);
}

double SemiTrailer::GetSpringLength(int axle, VehicleSide side) const {
    return std::static_pointer_cast<ChLeafspringAxle>(m_axles[axle]->m_suspension)->GetSpringLength(side);
}

double SemiTrailer::GetSpringDeformation(int axle, VehicleSide side) const {
    return std::static_pointer_cast<ChLeafspringAxle>(m_axles[axle]->m_suspension)->GetSpringDeformation(side);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double SemiTrailer::GetShockForce(int axle, VehicleSide side) const {
    return std::static_pointer_cast<ChLeafspringAxle>(m_axles[axle]->m_suspension)->GetShockForce(side);
}

double SemiTrailer::GetShockLength(int axle, VehicleSide side) const {
    return std::static_pointer_cast<ChLeafspringAxle>(m_axles[axle]->m_suspension)->GetShockLength(side);
}

double SemiTrailer::GetShockVelocity(int axle, VehicleSide side) const {
    return std::static_pointer_cast<ChLeafspringAxle>(m_axles[axle]->m_suspension)->GetShockVelocity(side);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void SemiTrailer::Synchronize(double time, double braking, const ChTerrain& terrain) {
    // Synchronize the trailer's axle subsystems
    // (this applies tire forces to suspension spindles and braking input)
    for (auto axle : m_axles) {
        for (auto& wheel : axle->GetWheels()) {
            wheel->GetTire()->Synchronize(time, terrain);
            axle->Synchronize(braking);
        }
    }
}

void SemiTrailer::Advance(double step) {
    for (auto axle : m_axles) {
        for (auto& wheel : axle->GetWheels()) {
            wheel->GetTire()->Advance(step);
        }
    }
}

// -----------------------------------------------------------------------------
// Log the hardpoint locations for the front-right and rear-right suspension
// subsystems (display in inches)
// -----------------------------------------------------------------------------
void SemiTrailer::LogHardpointLocations() {
    GetLog().SetNumFormat("%7.3f");

    GetLog() << "\n---- FRONT suspension hardpoint locations (RIGHT side)\n";
    std::static_pointer_cast<ChLeafspringAxle>(m_axles[0]->m_suspension)
        ->LogHardpointLocations(ChVector<>(0, 0, 0), true);
    GetLog() << "\n---- MID suspension hardpoint locations (RIGHT side)\n";
    std::static_pointer_cast<ChLeafspringAxle>(m_axles[1]->m_suspension)
        ->LogHardpointLocations(ChVector<>(0, 0, 0), true);
    GetLog() << "\n---- REAR suspension hardpoint locations (RIGHT side)\n";
    std::static_pointer_cast<ChLeafspringAxle>(m_axles[2]->m_suspension)
        ->LogHardpointLocations(ChVector<>(0, 0, 0), true);
    GetLog() << "\n\n";

    GetLog().SetNumFormat("%g");
}

// -----------------------------------------------------------------------------
// Log the spring length, deformation, and force.
// Log the shock length, velocity, and force.
// Log constraint violations of suspension joints.
//
// Lengths are reported in inches, velocities in inches/s, and forces in lbf
// -----------------------------------------------------------------------------
void SemiTrailer::DebugLog(int what) {
    GetLog().SetNumFormat("%10.2f");

    if (what & OUT_SPRINGS) {
        GetLog() << "\n---- Spring (front-left, front-right, mid-left, mid-right, rear-left, rear-right)\n";
        GetLog() << "Length [inch]       " << GetSpringLength(0, LEFT) << "  " << GetSpringLength(0, RIGHT) << "  "
                 << GetSpringLength(1, LEFT) << "  " << GetSpringLength(1, RIGHT) << "  " << GetSpringLength(2, LEFT)
                 << "  " << GetSpringLength(2, RIGHT) << "\n";
        GetLog() << "Deformation [inch]  " << GetSpringDeformation(0, LEFT) << "  " << GetSpringDeformation(0, RIGHT)
                 << "  " << GetSpringDeformation(1, LEFT) << "  " << GetSpringDeformation(1, RIGHT) << "  "
                 << GetSpringDeformation(2, LEFT) << "  " << GetSpringDeformation(2, RIGHT) << "\n";
        GetLog() << "Force [lbf]         " << GetSpringForce(0, LEFT) << "  " << GetSpringForce(0, RIGHT) << "  "
                 << GetSpringForce(1, LEFT) << "  " << GetSpringForce(1, RIGHT) << "  " << GetSpringForce(2, LEFT)
                 << "  " << GetSpringForce(2, RIGHT) << "\n";
    }

    if (what & OUT_SHOCKS) {
        GetLog() << "\n---- Shock (front-left, front-right, mid-left, mid-right, rear-left, rear-right)\n";
        GetLog() << "Length [inch]       " << GetShockLength(0, LEFT) << "  " << GetShockLength(0, RIGHT) << "  "
                 << GetShockLength(1, LEFT) << "  " << GetShockLength(1, RIGHT) << "  " << GetShockLength(2, LEFT)
                 << "  " << GetShockLength(2, RIGHT) << "\n";
        GetLog() << "Velocity [inch/s]   " << GetShockVelocity(0, LEFT) << "  " << GetShockVelocity(0, RIGHT) << "  "
                 << GetShockVelocity(1, LEFT) << "  " << GetShockVelocity(1, RIGHT) << "  " << GetShockVelocity(2, LEFT)
                 << "  " << GetShockVelocity(2, RIGHT) << "\n";
        GetLog() << "Force [lbf]         " << GetShockForce(0, LEFT) << "  " << GetShockForce(0, RIGHT) << "  "
                 << GetShockForce(1, LEFT) << "  " << GetShockForce(1, RIGHT) << "  " << GetShockForce(2, LEFT) << "  "
                 << GetShockForce(2, RIGHT) << "\n";
    }

    GetLog().SetNumFormat("%g");
}
