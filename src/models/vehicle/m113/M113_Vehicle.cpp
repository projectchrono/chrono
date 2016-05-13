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
#include "models/vehicle/m113/M113_Idler.h"
#include "models/vehicle/m113/M113_RoadWheel.h"
#include "models/vehicle/m113/M113_SimpleDriveline.h"
#include "models/vehicle/m113/M113_Sprocket.h"
#include "models/vehicle/m113/M113_TrackAssembly.h"
#include "models/vehicle/m113/M113_TrackShoe.h"
#include "models/vehicle/m113/M113_Vehicle.h"

using namespace chrono;
using namespace chrono::vehicle;

namespace m113 {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double M113_Vehicle::m_chassisMass = 5489.24;
const ChVector<> M113_Vehicle::m_chassisCOM(-2.006, 0, 0.406);
const ChVector<> M113_Vehicle::m_chassisInertia(1786.92, 10449.67, 10721.22);

const std::string M113_Vehicle::m_chassisMeshName = "Chassis_POV_geom";
const std::string M113_Vehicle::m_chassisMeshFile = vehicle::GetDataFile("M113/Chassis.obj");

const ChCoordsys<> M113_Vehicle::m_driverCsys(ChVector<>(0.0, 0.5, 1.2), ChQuaternion<>(1, 0, 0, 0));

// -----------------------------------------------------------------------------
// Constructors
// -----------------------------------------------------------------------------
M113_Vehicle::M113_Vehicle(bool fixed, ChMaterialSurfaceBase::ContactMethod contactMethod)
    : ChTrackedVehicle("M113 Vehicle", contactMethod), m_chassisVisType(PRIMITIVES) {
    Create(fixed);
}

M113_Vehicle::M113_Vehicle(bool fixed, ChSystem* system)
    : ChTrackedVehicle("M113 Vehicle", system), m_chassisVisType(PRIMITIVES) {
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
    m_tracks[0] = std::make_shared<M113_TrackAssembly>(LEFT);
    m_tracks[1] = std::make_shared<M113_TrackAssembly>(RIGHT);

    // Create the driveline
    m_driveline = std::make_shared<M113_SimpleDriveline>();
    ////m_driveline = std::make_shared<M113_DrivelineBDS>();

    GetLog() << "M113 vehicle mass = " << GetVehicleMass() << " kg.\n";
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void M113_Vehicle::SetChassisVisType(chrono::vehicle::VisualizationType vis) {
    m_chassisVisType = vis;
}

void M113_Vehicle::SetSprocketVisType(chrono::vehicle::VisualizationType vis) {
    std::static_pointer_cast<M113_Sprocket>(m_tracks[0]->GetSprocket())->SetVisType(vis);
    std::static_pointer_cast<M113_Sprocket>(m_tracks[1]->GetSprocket())->SetVisType(vis);
}

void M113_Vehicle::SetIdlerVisType(chrono::vehicle::VisualizationType vis) {
    std::static_pointer_cast<M113_Idler>(m_tracks[0]->GetIdler())->SetVisType(vis);
    std::static_pointer_cast<M113_Idler>(m_tracks[1]->GetIdler())->SetVisType(vis);
}

void M113_Vehicle::SetRoadWheelVisType(chrono::vehicle::VisualizationType vis) {
    for (size_t is = 0; is < 5; is++) {
        std::static_pointer_cast<M113_RoadWheel>(m_tracks[0]->GetRoadWheel(is))->SetVisType(vis);
        std::static_pointer_cast<M113_RoadWheel>(m_tracks[1]->GetRoadWheel(is))->SetVisType(vis);
    }
}

void M113_Vehicle::SetTrackShoeVisType(chrono::vehicle::VisualizationType vis) {
    for (size_t is = 0; is < m_tracks[0]->GetNumTrackShoes(); is++)
        std::static_pointer_cast<M113_TrackShoe>(m_tracks[0]->GetTrackShoe(is))->SetVisType(vis);
    for (size_t is = 0; is < m_tracks[1]->GetNumTrackShoes(); is++)
        std::static_pointer_cast<M113_TrackShoe>(m_tracks[1]->GetTrackShoe(is))->SetVisType(vis);
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
            trimesh.LoadWavefrontMesh(m_chassisMeshFile, false, false);
            auto trimesh_shape = std::make_shared<ChTriangleMeshShape>();
            trimesh_shape->SetMesh(trimesh);
            trimesh_shape->SetName(m_chassisMeshName);
            m_chassis->AddAsset(trimesh_shape);
            break;
        }
    }

    double track_offset = 1.0795;

    // Initialize the left track assembly.
    ChVector<> sprocket_loc(0, track_offset, 0);
    ChVector<> idler_loc(-3.92, track_offset, -0.12);  //// Original x value: -3.97
    std::vector<ChVector<> > susp_locs(5);
    susp_locs[0] = ChVector<>(-0.655, track_offset, -0.215);
    susp_locs[1] = ChVector<>(-1.322, track_offset, -0.215);
    susp_locs[2] = ChVector<>(-1.989, track_offset, -0.215);
    susp_locs[3] = ChVector<>(-2.656, track_offset, -0.215);
    susp_locs[4] = ChVector<>(-3.322, track_offset, -0.215);

    m_tracks[0]->Initialize(m_chassis, sprocket_loc, idler_loc, susp_locs);

    // Initialize the right track assembly.
    sprocket_loc.y = -track_offset;
    idler_loc.y = -track_offset;
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
    utils::WriteMeshPovray(m_chassisMeshFile, m_chassisMeshName, out_dir, ChColor(0.82f, 0.7f, 0.5f));
}

}  // end namespace m113
