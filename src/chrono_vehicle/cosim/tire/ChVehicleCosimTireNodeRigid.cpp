// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2021 projectchrono.org
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
// Definition of the vehicle co-simulation rigid TIRE NODE class.
// This type of tire communicates with the terrain node through a BODY
// communication interface.
//
// The global reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

//// RADU TODO
//// - relax assumption of SMC contact material (ask a tire to specify ChContactMaterialData)
//// - allow for different geometry (currently rigid mesh only)

#include "chrono_vehicle/cosim/tire/ChVehicleCosimTireNodeRigid.h"
#include "chrono/utils/ChUtilsInputOutput.h"

using std::cout;
using std::endl;

namespace chrono {
namespace vehicle {

ChVehicleCosimTireNodeRigid::ChVehicleCosimTireNodeRigid(int index, const std::string& tire_json)
    : ChVehicleCosimTireNode(index, tire_json) {
    assert(GetTireTypeFromSpecfile(tire_json) == TireType::RIGID);
    assert(m_tire);
    m_tire_rgd = std::static_pointer_cast<ChRigidTire>(m_tire);  // cache tire as ChRigidTire
}

void ChVehicleCosimTireNodeRigid::InitializeTire(std::shared_ptr<ChWheel> wheel, const ChVector3d& init_loc) {
    assert(m_tire_rgd->UseContactMesh());

    m_spindle->SetPos(init_loc);
    wheel->SetTire(m_tire);
    m_tire->Initialize(wheel);

    m_tire_rgd->SetVisualizationType(VisualizationType::MESH);

    // Tire mesh geometry and contact material
    auto trimesh = m_tire_rgd->GetContactMesh();
    auto cmat = std::static_pointer_cast<ChContactMaterialSMC>(m_tire_rgd->GetContactMaterial());
    m_geometry.coll_meshes.push_back(utils::ChBodyGeometry::TrimeshShape(VNULL, trimesh, 0.0, 0));
    m_geometry.materials.push_back(ChContactMaterialData(cmat->GetSlidingFriction(), cmat->GetRestitution(),
                                                         cmat->GetYoungModulus(), cmat->GetPoissonRatio(),
                                                         cmat->GetKn(), cmat->GetGn(), cmat->GetKt(), cmat->GetGt()));

    // Preprocess the tire mesh and store neighbor element information for each vertex.
    // Calculate mesh triangle areas.
    auto num_verts = trimesh->GetNumVertices();
    auto num_triangles = trimesh->GetNumTriangles();
    auto& verts = trimesh->GetCoordsVertices();
    auto& idx_verts = trimesh->GetIndicesVertexes();
    m_adjElements.resize(num_verts);
    std::vector<double> triArea(num_triangles);
    for (unsigned int ie = 0; ie < num_triangles; ie++) {
        unsigned int iv1 = idx_verts[ie].x();
        unsigned int iv2 = idx_verts[ie].y();
        unsigned int iv3 = idx_verts[ie].z();
        ChVector3d v1 = verts[iv1];
        ChVector3d v2 = verts[iv2];
        ChVector3d v3 = verts[iv3];
        triArea[ie] = 0.5 * Vcross(v2 - v1, v3 - v1).Length();
        m_adjElements[iv1].push_back(ie);
        m_adjElements[iv2].push_back(ie);
        m_adjElements[iv3].push_back(ie);
    }

    // Preprocess the tire mesh and store representative area for each vertex.
    m_vertexArea.resize(num_verts);
    for (unsigned int in = 0; in < num_verts; in++) {
        double area = 0;
        for (unsigned int ie = 0; ie < m_adjElements[in].size(); ie++) {
            area += triArea[m_adjElements[in][ie]];
        }
        m_vertexArea[in] = area / m_adjElements[in].size();
    }
}

void ChVehicleCosimTireNodeRigid::ApplySpindleState(const BodyState& spindle_state) {
    m_spindle->SetPos(spindle_state.pos);
    m_spindle->SetRot(spindle_state.rot);
    m_spindle->SetPosDt(spindle_state.lin_vel);
    m_spindle->SetAngVelParent(spindle_state.ang_vel);
}

void ChVehicleCosimTireNodeRigid::ApplySpindleForce(const TerrainForce& spindle_force) {
    // Cache spindle force for reporting
    m_force = spindle_force;
}

void ChVehicleCosimTireNodeRigid::OnOutputData(int frame) {
    // Append to results output file
    if (m_outf.is_open()) {
        std::string del("  ");

        m_outf << frame << del;
        m_outf << m_force.point.x() << del << m_force.point.y() << del << m_force.point.z() << del;
        m_outf << m_force.force.x() << del << m_force.force.y() << del << m_force.force.z() << del;
        m_outf << m_force.moment.x() << del << m_force.moment.y() << del << m_force.moment.z() << del;
        m_outf << endl;
    }

    // Create and write frame output file.
    utils::ChWriterCSV csv(" ");
    WriteTireStateInformation(csv);
    WriteTireMeshInformation(csv);

    std::string filename = OutputFilename(m_node_out_dir + "/simulation", "data", "dat", frame + 1, 5);
    csv.WriteToFile(filename);

    if (m_verbose)
        cout << "[Tire node   ] write output file ==> " << filename << endl;
}

void ChVehicleCosimTireNodeRigid::WriteTireStateInformation(utils::ChWriterCSV& csv) {
    // Write number of vertices
    int num_vertices = m_tire_rgd->GetContactMesh()->GetNumVertices();
    csv << num_vertices << endl;

    // Write mesh vertex positions and velocities
    std::vector<ChVector3d> pos;
    std::vector<ChVector3d> vel;
    m_tire_rgd->GetMeshVertexStates(pos, vel);
    for (int in = 0; in < num_vertices; in++)
        csv << pos[in] << endl;
    for (int in = 0; in < num_vertices; in++)
        csv << vel[in] << endl;
}

void ChVehicleCosimTireNodeRigid::WriteTireMeshInformation(utils::ChWriterCSV& csv) {
    // Print tire mesh connectivity
    int num_triangles = m_tire_rgd->GetContactMesh()->GetNumTriangles();
    csv << num_triangles << endl;

    const std::vector<ChVector3i>& triangles = m_tire_rgd->GetContactMesh()->GetIndicesVertexes();
    for (int ie = 0; ie < num_triangles; ie++) {
        csv << triangles[ie] << endl;
    }
}

void ChVehicleCosimTireNodeRigid::OutputVisualizationData(int frame) {
    auto filename = OutputFilename(m_node_out_dir + "/visualization", "vis", "dat", frame, 5);
    utils::WriteVisualizationAssets(m_system, filename, true);
}

}  // namespace vehicle
}  // namespace chrono
