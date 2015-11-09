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

#include "m113/M113_Vehicle.h"
#include "m113/M113_TrackAssembly.h"
#include "m113/M113_SimpleDriveline.h"

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
const std::string M113_Vehicle::m_chassisMeshFile = vehicle::GetDataFile("m113/Chassis.obj");

const ChCoordsys<> M113_Vehicle::m_driverCsys(ChVector<>(0.0, 0.5, 1.2), ChQuaternion<>(1, 0, 0, 0));

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
M113_Vehicle::M113_Vehicle(const bool fixed,
                           chrono::vehicle::VisualizationType chassisVis,
                           chrono::vehicle::VisualizationType trackVis,
                           chrono::ChMaterialSurfaceBase::ContactMethod contactMethod)
    : ChTrackedVehicle("M113 Vehicle", contactMethod) {
    // -------------------------------------------
    // Create the chassis body
    // -------------------------------------------
    m_chassis = ChSharedPtr<ChBodyAuxRef>(new ChBodyAuxRef(m_system->GetContactMethod()));

    m_chassis->SetIdentifier(0);
    m_chassis->SetName("chassis");
    m_chassis->SetMass(m_chassisMass);
    m_chassis->SetFrame_COG_to_REF(ChFrame<>(m_chassisCOM, ChQuaternion<>(1, 0, 0, 0)));
    m_chassis->SetInertiaXX(m_chassisInertia);
    m_chassis->SetBodyFixed(fixed);

    switch (chassisVis) {
        case PRIMITIVES: {
            ChSharedPtr<ChSphereShape> sphere(new ChSphereShape);
            sphere->GetSphereGeometry().rad = 0.1;
            sphere->Pos = m_chassisCOM;
            m_chassis->AddAsset(sphere);
            break;
        }
        case MESH: {
            geometry::ChTriangleMeshConnected trimesh;
            trimesh.LoadWavefrontMesh(m_chassisMeshFile, false, false);
            ChSharedPtr<ChTriangleMeshShape> trimesh_shape(new ChTriangleMeshShape);
            trimesh_shape->SetMesh(trimesh);
            trimesh_shape->SetName(m_chassisMeshName);
            m_chassis->AddAsset(trimesh_shape);
            break;
        }
    }

    m_system->Add(m_chassis);

    // -------------------------------------------
    // Create the track assembly subsystems
    // -------------------------------------------
    m_tracks[0] = ChSharedPtr<ChTrackAssembly>(new M113_TrackAssembly(LEFT, trackVis));
    m_tracks[1] = ChSharedPtr<ChTrackAssembly>(new M113_TrackAssembly(RIGHT, trackVis));

    // --------------------
    // Create the driveline
    // --------------------
    m_driveline = ChSharedPtr<ChTrackDriveline>(new M113_SimpleDriveline);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void M113_Vehicle::Initialize(const ChCoordsys<>& chassisPos) {
    m_chassis->SetFrame_REF_to_abs(ChFrame<>(chassisPos));

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
