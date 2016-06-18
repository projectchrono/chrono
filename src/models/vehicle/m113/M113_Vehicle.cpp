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
// M113 vehicle model.
//
// =============================================================================

#include "chrono/assets/ChSphereShape.h"
#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "models/vehicle/m113/M113_DrivelineBDS.h"
#include "models/vehicle/m113/M113_SimpleDriveline.h"
#include "models/vehicle/m113/M113_TrackAssemblySinglePin.h"
#include "models/vehicle/m113/M113_TrackAssemblyDoublePin.h"
#include "models/vehicle/m113/M113_Vehicle.h"

namespace chrono {
namespace vehicle {
namespace m113 {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double M113_Vehicle::m_chassisMass = 5489.24;
const ChVector<> M113_Vehicle::m_chassisCOM(-2.006, 0, 0.406);
const ChVector<> M113_Vehicle::m_chassisInertia(1786.92, 10449.67, 10721.22);

const std::string M113_Vehicle::m_chassisMeshName = "Chassis_POV_geom";
const std::string M113_Vehicle::m_chassisMeshFile = "M113/Chassis.obj";

const ChCoordsys<> M113_Vehicle::m_driverCsys(ChVector<>(0.0, 0.5, 1.2), ChQuaternion<>(1, 0, 0, 0));

// -----------------------------------------------------------------------------
// Constructors
// -----------------------------------------------------------------------------
M113_Vehicle::M113_Vehicle(bool fixed, TrackShoeType shoe_type, ChMaterialSurfaceBase::ContactMethod contactMethod)
    : ChTrackedVehicle("M113 Vehicle", contactMethod), m_type(shoe_type), m_chassisVisType(PRIMITIVES) {
    Create(fixed);
}

M113_Vehicle::M113_Vehicle(bool fixed, TrackShoeType shoe_type, ChSystem* system)
    : ChTrackedVehicle("M113 Vehicle", system), m_type(shoe_type), m_chassisVisType(PRIMITIVES) {
    Create(fixed);
}

void M113_Vehicle::Create(bool fixed) {
    // Create the chassis body
    m_chassis = std::shared_ptr<ChBodyAuxRef>(m_system->NewBodyAuxRef());

    m_chassis->SetIdentifier(0);
    m_chassis->SetName("chassis");
    m_chassis->SetMass(m_chassisMass);
    m_chassis->SetFrame_COG_to_REF(ChFrame<>(m_chassisCOM, ChQuaternion<>(1, 0, 0, 0)));
    m_chassis->SetInertiaXX(m_chassisInertia);
    m_chassis->SetBodyFixed(fixed);

    m_system->Add(m_chassis);

    // Create the track assembly subsystems
    switch (m_type) {
        case SINGLE_PIN:
            m_tracks[0] = std::make_shared<M113_TrackAssemblySinglePin>(LEFT);
            m_tracks[1] = std::make_shared<M113_TrackAssemblySinglePin>(RIGHT);
            break;
        case DOUBLE_PIN:
            m_tracks[0] = std::make_shared<M113_TrackAssemblyDoublePin>(LEFT);
            m_tracks[1] = std::make_shared<M113_TrackAssemblyDoublePin>(RIGHT);
            break;
    }

    // Create the driveline
    m_driveline = std::make_shared<M113_SimpleDriveline>();
    ////m_driveline = std::make_shared<M113_DrivelineBDS>();

    GetLog() << "M113 vehicle mass = " << GetVehicleMass() << " kg.\n";
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void M113_Vehicle::SetChassisVisType(VisualizationType vis) {
    m_chassisVisType = vis;
}

void M113_Vehicle::SetSprocketVisType(VisualizationType vis) {
    switch (m_type) {
        case SINGLE_PIN:
            std::static_pointer_cast<M113_TrackAssemblySinglePin>(m_tracks[0])->SetSprocketVisType(vis);
            std::static_pointer_cast<M113_TrackAssemblySinglePin>(m_tracks[1])->SetSprocketVisType(vis);
            break;
        case DOUBLE_PIN:
            std::static_pointer_cast<M113_TrackAssemblyDoublePin>(m_tracks[0])->SetSprocketVisType(vis);
            std::static_pointer_cast<M113_TrackAssemblyDoublePin>(m_tracks[1])->SetSprocketVisType(vis);
            break;
    }
}

void M113_Vehicle::SetIdlerVisType(VisualizationType vis) {
    switch (m_type) {
        case SINGLE_PIN:
            std::static_pointer_cast<M113_TrackAssemblySinglePin>(m_tracks[0])->SetIdlerVisType(vis);
            std::static_pointer_cast<M113_TrackAssemblySinglePin>(m_tracks[1])->SetIdlerVisType(vis);
            break;
        case DOUBLE_PIN:
            std::static_pointer_cast<M113_TrackAssemblyDoublePin>(m_tracks[0])->SetIdlerVisType(vis);
            std::static_pointer_cast<M113_TrackAssemblyDoublePin>(m_tracks[1])->SetIdlerVisType(vis);
            break;
    }
}

void M113_Vehicle::SetRoadWheelVisType(VisualizationType vis) {
    switch (m_type) {
        case SINGLE_PIN:
            std::static_pointer_cast<M113_TrackAssemblySinglePin>(m_tracks[0])->SetRoadWheelVisType(vis);
            std::static_pointer_cast<M113_TrackAssemblySinglePin>(m_tracks[1])->SetRoadWheelVisType(vis);
            break;
        case DOUBLE_PIN:
            std::static_pointer_cast<M113_TrackAssemblyDoublePin>(m_tracks[0])->SetRoadWheelVisType(vis);
            std::static_pointer_cast<M113_TrackAssemblyDoublePin>(m_tracks[1])->SetRoadWheelVisType(vis);
            break;
    }
}

void M113_Vehicle::SetTrackShoeVisType(VisualizationType vis) {
    switch (m_type) {
        case SINGLE_PIN:
            std::static_pointer_cast<M113_TrackAssemblySinglePin>(m_tracks[0])->SetTrackShoeVisType(vis);
            std::static_pointer_cast<M113_TrackAssemblySinglePin>(m_tracks[1])->SetTrackShoeVisType(vis);
            break;
        case DOUBLE_PIN:
            std::static_pointer_cast<M113_TrackAssemblyDoublePin>(m_tracks[0])->SetTrackShoeVisType(vis);
            std::static_pointer_cast<M113_TrackAssemblyDoublePin>(m_tracks[1])->SetTrackShoeVisType(vis);
            break;
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void M113_Vehicle::Initialize(const ChCoordsys<>& chassisPos) {
    // Set chassis position and visualization assets.
    m_chassis->SetFrame_REF_to_abs(ChFrame<>(chassisPos));

    switch (m_chassisVisType) {
        case PRIMITIVES: {
            auto sphere = std::make_shared<ChSphereShape>();
            sphere->GetSphereGeometry().rad = 0.1;
            sphere->Pos = m_chassisCOM;
            m_chassis->AddAsset(sphere);
            break;
        }
        case MESH: {
            geometry::ChTriangleMeshConnected trimesh;
            trimesh.LoadWavefrontMesh(vehicle::GetDataFile(m_chassisMeshFile), false, false);
            auto trimesh_shape = std::make_shared<ChTriangleMeshShape>();
            trimesh_shape->SetMesh(trimesh);
            trimesh_shape->SetName(m_chassisMeshName);
            m_chassis->AddAsset(trimesh_shape);
            break;
        }
    }

    double track_offset = 1.0795;
    ChVector<> sprocket_loc;
    ChVector<> idler_loc;
    std::vector<ChVector<> > susp_locs(5);

    // Initialize the left track assembly.
    sprocket_loc = ChVector<>(0, track_offset, 0);
    idler_loc = ChVector<>(-3.92, track_offset, -0.12);  //// Original x value: -3.97   
    susp_locs[0] = ChVector<>(-0.655, track_offset, -0.215);
    susp_locs[1] = ChVector<>(-1.322, track_offset, -0.215);
    susp_locs[2] = ChVector<>(-1.989, track_offset, -0.215);
    susp_locs[3] = ChVector<>(-2.656, track_offset, -0.215);
    susp_locs[4] = ChVector<>(-3.322, track_offset, -0.215);

    m_tracks[0]->Initialize(m_chassis, sprocket_loc, idler_loc, susp_locs);

    // Initialize the right track assembly.
    sprocket_loc = ChVector<>(0, -track_offset, 0);
    idler_loc = ChVector<>(-3.92, -track_offset, -0.12);  //// Original x value: -3.97  
    susp_locs[0] = ChVector<>(-0.740, -track_offset, -0.215);
    susp_locs[1] = ChVector<>(-1.407, -track_offset, -0.215);
    susp_locs[2] = ChVector<>(-2.074, -track_offset, -0.215);
    susp_locs[3] = ChVector<>(-2.740, -track_offset, -0.215);
    susp_locs[4] = ChVector<>(-3.407, -track_offset, -0.215);

    m_tracks[1]->Initialize(m_chassis, sprocket_loc, idler_loc, susp_locs);

    // Initialize the driveline subsystem
    m_driveline->Initialize(m_chassis, m_tracks[0], m_tracks[1]);

    //// TODO
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void M113_Vehicle::ExportMeshPovray(const std::string& out_dir) {
    utils::WriteMeshPovray(vehicle::GetDataFile(m_chassisMeshFile), m_chassisMeshName, out_dir,
                           ChColor(0.82f, 0.7f, 0.5f));
}

}  // end namespace m113
}  // end namespace vehicle
}  // end namespace chrono
