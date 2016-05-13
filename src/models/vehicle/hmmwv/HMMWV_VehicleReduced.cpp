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
// Authors: Radu Serban, Justin Madsen
// =============================================================================
//
// HMMWV 9-body vehicle model...
//
// =============================================================================

#include "chrono/assets/ChSphereShape.h"
#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "models/vehicle/hmmwv/HMMWV_VehicleReduced.h"

using namespace chrono;
using namespace chrono::vehicle;

namespace hmmwv {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

static const double in2m = 0.0254;
static const double lb2kg = 0.453592;
static const double lbf2N = 4.44822162;

const double HMMWV_VehicleReduced::m_chassisMass = lb2kg * 7740.7;                               // chassis sprung mass
const ChVector<> HMMWV_VehicleReduced::m_chassisCOM = in2m * ChVector<>(-18.8, -0.585, 33.329);  // COM location
const ChVector<> HMMWV_VehicleReduced::m_chassisInertia(125.8, 497.4, 531.4);  // chassis inertia (roll,pitch,yaw)

const std::string HMMWV_VehicleReduced::m_chassisMeshName = "hmmwv_chassis_POV_geom";
const std::string HMMWV_VehicleReduced::m_chassisMeshFile = vehicle::GetDataFile("hmmwv/hmmwv_chassis.obj");

const ChCoordsys<> HMMWV_VehicleReduced::m_driverCsys(ChVector<>(0.0, 0.5, 1.2), ChQuaternion<>(1, 0, 0, 0));

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
HMMWV_VehicleReduced::HMMWV_VehicleReduced(const bool fixed,
                                           DrivelineType driveType,
                                           VisualizationType chassisVis,
                                           VisualizationType wheelVis,
                                           ChMaterialSurfaceBase::ContactMethod contactMethod)
    : ChWheeledVehicle(contactMethod), m_driveType(driveType) {
    Create(fixed, chassisVis, wheelVis);
}

HMMWV_VehicleReduced::HMMWV_VehicleReduced(ChSystem* system,
                                           const bool fixed,
                                           DrivelineType driveType,
                                           VisualizationType chassisVis,
                                           VisualizationType wheelVis)
    : ChWheeledVehicle(system), m_driveType(driveType) {
    Create(fixed, chassisVis, wheelVis);
}

void HMMWV_VehicleReduced::Create(bool fixed, VisualizationType chassisVis, VisualizationType wheelVis) {
    // -------------------------------------------
    // Create the chassis body
    // -------------------------------------------
    m_chassis = std::shared_ptr<ChBodyAuxRef>(m_system->NewBodyAuxRef());

    m_chassis->SetIdentifier(0);
    m_chassis->SetName("chassis");
    m_chassis->SetMass(m_chassisMass);
    m_chassis->SetFrame_COG_to_REF(ChFrame<>(m_chassisCOM, ChQuaternion<>(1, 0, 0, 0)));
    m_chassis->SetInertiaXX(m_chassisInertia);
    m_chassis->SetBodyFixed(fixed);

    switch (chassisVis) {
        case PRIMITIVES: {
            auto sphere = std::make_shared<ChSphereShape>();
            sphere->GetSphereGeometry().rad = 0.1;
            sphere->Pos = m_chassisCOM;
            m_chassis->AddAsset(sphere);

            break;
        }
        case MESH: {
            geometry::ChTriangleMeshConnected trimesh;
            trimesh.LoadWavefrontMesh(m_chassisMeshFile, false, false);

            auto trimesh_shape = std::make_shared<ChTriangleMeshShape>();
            trimesh_shape->SetMesh(trimesh);
            trimesh_shape->SetName(m_chassisMeshName);
            m_chassis->AddAsset(trimesh_shape);

            break;
        }
    }

    m_system->Add(m_chassis);

    // -------------------------------------------
    // Create the suspension subsystems
    // -------------------------------------------
    m_suspensions.resize(2);
    m_suspensions[0] = std::make_shared<HMMWV_DoubleWishboneReducedFront>("FrontSusp");
    m_suspensions[1] = std::make_shared<HMMWV_DoubleWishboneReducedRear>("RearSusp");

    // -----------------------------
    // Create the steering subsystem
    // -----------------------------
    m_steerings.resize(1);
    m_steerings[0] = std::make_shared<HMMWV_RackPinion>("Steering");

    // -----------------
    // Create the wheels
    // -----------------
    m_wheels.resize(4);
    m_wheels[0] = std::make_shared<HMMWV_WheelLeft>(wheelVis);
    m_wheels[1] = std::make_shared<HMMWV_WheelRight>(wheelVis);
    m_wheels[2] = std::make_shared<HMMWV_WheelLeft>(wheelVis);
    m_wheels[3] = std::make_shared<HMMWV_WheelRight>(wheelVis);

    // --------------------
    // Create the driveline
    // --------------------
    switch (m_driveType) {
        case FWD:
        case RWD:
            m_driveline = std::make_shared<HMMWV_Driveline2WD>();
            break;
        case AWD:
            m_driveline = std::make_shared<HMMWV_Driveline4WD>();
            break;
    }

    // -----------------
    // Create the brakes
    // -----------------
    m_brakes.resize(4);
    m_brakes[0] = std::make_shared<HMMWV_BrakeSimple>();
    m_brakes[1] = std::make_shared<HMMWV_BrakeSimple>();
    m_brakes[2] = std::make_shared<HMMWV_BrakeSimple>();
    m_brakes[3] = std::make_shared<HMMWV_BrakeSimple>();
}

HMMWV_VehicleReduced::~HMMWV_VehicleReduced() {
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void HMMWV_VehicleReduced::Initialize(const ChCoordsys<>& chassisPos) {
    m_chassis->SetFrame_REF_to_abs(ChFrame<>(chassisPos));

    // Initialize the steering subsystem (specify the steering subsystem's frame
    // relative to the chassis reference frame).
    ChVector<> offset = in2m * ChVector<>(56.735, 0, 3.174);
    m_steerings[0]->Initialize(m_chassis, offset, ChQuaternion<>(1, 0, 0, 0));

    // Initialize the suspension subsystems (specify the suspension subsystems'
    // frames relative to the chassis reference frame).
    m_suspensions[0]->Initialize(m_chassis, in2m * ChVector<>(66.59, 0, 1.039), m_steerings[0]->GetSteeringLink());
    m_suspensions[1]->Initialize(m_chassis, in2m * ChVector<>(-66.4, 0, 1.039), m_chassis);

    // Initialize wheels
    m_wheels[0]->Initialize(m_suspensions[0]->GetSpindle(LEFT));
    m_wheels[1]->Initialize(m_suspensions[0]->GetSpindle(RIGHT));
    m_wheels[2]->Initialize(m_suspensions[1]->GetSpindle(LEFT));
    m_wheels[3]->Initialize(m_suspensions[1]->GetSpindle(RIGHT));

    // Initialize the driveline subsystem.
    std::vector<int> driven_susp_indexes(m_driveline->GetNumDrivenAxles());

    switch (m_driveType) {
        case FWD:
            driven_susp_indexes[0] = 0;
            break;
        case RWD:
            driven_susp_indexes[0] = 1;
            break;
        case AWD:
            driven_susp_indexes[0] = 0;
            driven_susp_indexes[1] = 1;
            break;
    }

    m_driveline->Initialize(m_chassis, m_suspensions, driven_susp_indexes);

    // Initialize the four brakes
    m_brakes[0]->Initialize(m_suspensions[0]->GetRevolute(LEFT));
    m_brakes[1]->Initialize(m_suspensions[0]->GetRevolute(RIGHT));
    m_brakes[2]->Initialize(m_suspensions[1]->GetRevolute(LEFT));
    m_brakes[3]->Initialize(m_suspensions[1]->GetRevolute(RIGHT));
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void HMMWV_VehicleReduced::ExportMeshPovray(const std::string& out_dir) {
    utils::WriteMeshPovray(m_chassisMeshFile, m_chassisMeshName, out_dir, ChColor(0.82f, 0.7f, 0.5f));

    HMMWV_Wheel* wheelFL = static_cast<HMMWV_Wheel*>(m_wheels[0].get());
    HMMWV_Wheel* wheelFR = static_cast<HMMWV_Wheel*>(m_wheels[1].get());
    wheelFL->ExportMeshPovray(out_dir);
    wheelFR->ExportMeshPovray(out_dir);
}

}  // end namespace hmmwv
