// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
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
// Template for a simplified deformable tire.
//
// =============================================================================

//// TODO:
//// - implement CalculateForces (currently called from Setup())
//// - implement CalculateInertiaProperties
//// - extract force on wheel spindle
//// - add false coloring support

#include <algorithm>

#include "chrono_vehicle/wheeled_vehicle/tire/ChLUTTire.h"

namespace chrono {
namespace vehicle {

ChLUTTire::ChLUTTire(const std::string& name) : ChDeformableTire(name) {
    m_model = chrono_types::make_shared<LUTTireModel>();
    m_model->m_tire = this;
}

void ChLUTTire::SetTireGeometry(const std::vector<double>& ring_radii,
                                const std::vector<double>& ring_offsets,
                                int num_divs) {
    assert(ring_radii.size() > 1);
    assert(ring_radii.size() == ring_offsets.size());

    m_model->m_radii = ring_radii;
    m_model->m_offsets = ring_offsets;

    m_model->m_num_rings = (int)ring_radii.size();
    m_model->m_num_divs = num_divs;
}

void ChLUTTire::SetTireMass(double mass) {
    m_mass = mass;
}

void ChLUTTire::SetTireProperties(double eR, double dR, double eC, double dC, double eT, double dT) {
    m_model->m_eR = eR;
    m_model->m_dR = dR;
    m_model->m_eC = eC;
    m_model->m_dC = dC;
    m_model->m_eT = eT;
    m_model->m_dT = dT;
}

void ChLUTTire::SetTireContactMaterial(const ChContactMaterialData& mat_data) {
    m_contact_mat_data = mat_data;
}

double ChLUTTire::GetRadius() const {
    return *std::max_element(m_model->m_radii.begin(), m_model->m_radii.end());
}

double ChLUTTire::GetRimRadius() const {
    return 0;  // not needed
}

double ChLUTTire::GetWidth() const {
    return m_model->m_offsets.back() - m_model->m_offsets.front();
}

void ChLUTTire::CreateContactMaterial() {
    m_contact_mat =
        std::static_pointer_cast<ChMaterialSurfaceSMC>(m_contact_mat_data.CreateMaterial(ChContactMethod::SMC));
}

void ChLUTTire::Initialize(std::shared_ptr<ChWheel> wheel) {
    ChTire::Initialize(wheel);

    ChSystem* system = wheel->GetSpindle()->GetSystem();
    assert(system);

    if (IsPressureEnabled() && m_pressure <= 0)
        m_pressure = GetDefaultPressure();

    if (IsContactEnabled())
        CreateContactMaterial();

    m_model->m_wheel = wheel->GetSpindle().get();
    m_model->Construct(*(wheel->GetSpindle().get()));

    system->Add(m_model);
}

void ChLUTTire::Synchronize(double time, const ChTerrain& terrain) {
    //// TODO
}

void ChLUTTire::Advance(double step) {
    //// TODO
}

TerrainForce ChLUTTire::ReportTireForce(ChTerrain* terrain) const {
    TerrainForce terrain_force;
    //// TODO
    return terrain_force;
}

TerrainForce ChLUTTire::ReportTireForce(ChTerrain* terrain, ChCoordsys<>& tire_frame) const {
    std::cerr << "ChLUTTire::ReportTireForce for local frame not implemented." << std::endl;
    throw ChException("ChLUTTire::ReportTireForce for local frame not implemented.");
}

void ChLUTTire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::NONE)
        return;

    m_model->AddVisualShape(m_model->m_trimesh_shape);
}

void ChLUTTire::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAssets(m_model);
}

void ChLUTTire::InitializeInertiaProperties() {
    if (!IsInitialized())
        return;

    ChVector<> com;
    m_model->CalculateInertiaProperties(*(m_wheel->GetSpindle().get()), com, m_inertia);
    m_com = ChFrame<>(com, QUNIT);
}

void ChLUTTire::UpdateInertiaProperties() {
    if (!IsInitialized())
        return;

    ChVector<> com;
    m_model->CalculateInertiaProperties(*(m_wheel->GetSpindle().get()), com, m_inertia);
    m_com = ChFrame<>(com, QUNIT);
}

// =============================================================================

int LUTTireModel::NodeIndex(int ir, int id) {
    // If ring index out-of-bounds, return -1
    if (ir < 0 || ir >= m_num_rings)
        return -1;

    // Make sure to use a positive value of the division index
    while (id < 0)
        id += m_num_divs;

    // Wrap around circumference if needed
    return ir * m_num_divs + (id % m_num_divs);
}

void LUTTireModel::Construct(const ChFrameMoving<>& wheel_frame) {
    m_num_nodes = m_num_rings * m_num_divs;
    m_num_faces = 2 * (m_num_rings - 1) * m_num_divs;

    m_node_mass = m_tire->GetMass() / m_num_nodes;

    // Create the visualization shape and access the underling trimesh
    m_trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
    auto trimesh = m_trimesh_shape->GetMesh();
    auto& vertices = trimesh->getCoordsVertices();
    auto& normals = trimesh->getCoordsNormals();
    auto& idx_vertices = trimesh->getIndicesVertexes();
    auto& idx_normals = trimesh->getIndicesNormals();
    ////auto& uv_coords = trimesh->getCoordsUV();
    auto& colors = trimesh->getCoordsColors();

    // Create the FEA nodes (with positions expressed in the global frame) and load the mesh vertices.
    m_nodes.resize(m_num_nodes);
    vertices.resize(m_num_nodes);
    double dphi = CH_C_2PI / m_num_divs;
    int k = 0;
    for (int ir = 0; ir < m_num_rings; ir++) {
        double y = m_offsets[ir];
        double r = m_radii[ir];
        for (int id = 0; id < m_num_divs; id++) {
            double phi = id * dphi;
            double x = r * std::cos(phi);
            double z = r * std::sin(phi);
            vertices[k] = wheel_frame.TransformPointLocalToParent(ChVector<>(x, y, z));
            m_nodes[k] = chrono_types::make_shared<fea::ChNodeFEAxyz>(vertices[k]);
            m_nodes[k]->SetMass(m_node_mass);
            k++;
        }
    }

    // Auxiliary face information (for possible collision mesh)
    struct FaceAuxData {
        ChVector<int> nbr_node;    // neighbor (opposite) vertex/node for each face vertex
        ChVector<bool> owns_node;  // vertex/node owned by the face?
        ChVector<bool> owns_edge;  // edge owned by the face?
    };
    std::vector<FaceAuxData> auxdata(m_num_faces);

    // Create the mesh faces and define auxiliary data (for possible collision mesh)
    idx_vertices.resize(m_num_faces);
    idx_normals.resize(m_num_faces);
    k = 0;
    for (int ir = 0; ir < m_num_rings - 1; ir++) {
        bool last = ir == m_num_rings - 2;
        for (int id = 0; id < m_num_divs; id++) {
            int v1 = NodeIndex(ir, id);
            int v2 = NodeIndex(ir + 1, id);
            int v3 = NodeIndex(ir + 1, id + 1);
            int v4 = NodeIndex(ir, id + 1);
            idx_vertices[k] = {v1, v2, v3};
            idx_normals[k] = {v1, v2, v3};
            auxdata[k].nbr_node = {NodeIndex(ir + 2, id + 1), v4, NodeIndex(ir, id - 1)};
            auxdata[k].owns_node = {true, last, false};
            auxdata[k].owns_edge = {true, last, true};
            k++;
            idx_vertices[k] = {v1, v3, v4};
            idx_normals[k] = {v1, v3, v4};
            auxdata[k].nbr_node = {NodeIndex(ir + 1, id + 2), NodeIndex(ir - 1, id), v2};
            auxdata[k].owns_node = {false, false, false};
            auxdata[k].owns_edge = {false, false, true};
            k++;
        }
    }

    // Create the contact surface of the specified type and initialize it using the underlying model
    if (m_tire->IsContactEnabled()) {
        auto contact_mat = m_tire->GetContactMaterial();

        switch (m_tire->GetContactSurfaceType()) {
            case ChDeformableTire::ContactSurfaceType::NODE_CLOUD: {
                auto contact_surf = chrono_types::make_shared<fea::ChContactSurfaceNodeCloud>(contact_mat);
                contact_surf->SetPhysicsItem(this);
                for (const auto& node : m_nodes)
                    contact_surf->AddNode(node, m_tire->GetContactNodeRadius());
                m_contact_surf = contact_surf;
                break;
            }
            case ChDeformableTire::ContactSurfaceType::TRIANGLE_MESH: {
                auto contact_surf = chrono_types::make_shared<fea::ChContactSurfaceMesh>(contact_mat);
                contact_surf->SetPhysicsItem(this);
                for (k = 0; k < m_num_faces; k++) {
                    const auto& node1 = m_nodes[idx_vertices[k][0]];
                    const auto& node2 = m_nodes[idx_vertices[k][1]];
                    const auto& node3 = m_nodes[idx_vertices[k][2]];
                    const auto& edge_node1 = (auxdata[k].nbr_node[0] != -1 ? m_nodes[auxdata[k].nbr_node[0]] : nullptr);
                    const auto& edge_node2 = (auxdata[k].nbr_node[1] != -1 ? m_nodes[auxdata[k].nbr_node[1]] : nullptr);
                    const auto& edge_node3 = (auxdata[k].nbr_node[2] != -1 ? m_nodes[auxdata[k].nbr_node[2]] : nullptr);
                    contact_surf->AddFace(node1, node2, node3,                                                        //
                                          edge_node1, edge_node2, edge_node3,                                         //
                                          auxdata[k].owns_node[0], auxdata[k].owns_node[1], auxdata[k].owns_node[2],  //
                                          auxdata[k].owns_edge[0], auxdata[k].owns_edge[1], auxdata[k].owns_edge[2],  //
                                          m_tire->GetContactFaceThickness());
                }

                m_contact_surf = contact_surf;
                break;
            }
        }
    }

    // Initialize mesh colors and vertex normals
    colors.resize(m_num_nodes, ChColor(1, 1, 1));
    normals.resize(m_num_nodes, ChVector<>(0, 0, 0));

    // Calculate face normals, accumulate vertex normals, and count number of faces adjacent to each vertex
    std::vector<int> accumulators(m_num_nodes, 0);
    for (int it = 0; it < m_num_faces; it++) {
        // Calculate the triangle normal as a normalized cross product.
        ChVector<> nrm = Vcross(vertices[idx_vertices[it][1]] - vertices[idx_vertices[it][0]],
                                vertices[idx_vertices[it][2]] - vertices[idx_vertices[it][0]]);
        nrm.Normalize();
        // Increment the normals of all incident vertices by the face normal
        normals[idx_normals[it][0]] += nrm;
        normals[idx_normals[it][1]] += nrm;
        normals[idx_normals[it][2]] += nrm;
        // Increment the count of all incident vertices by 1
        accumulators[idx_normals[it][0]] += 1;
        accumulators[idx_normals[it][1]] += 1;
        accumulators[idx_normals[it][2]] += 1;
    }

    // Set vertex normals to avergave values over all adjacent faces
    for (int in = 0; in < m_num_nodes; in++) {
        normals[in] /= (double)accumulators[in];
    }
}

void LUTTireModel::CalculateInertiaProperties(const ChFrameMoving<>& wheel_frame,
                                              ChVector<>& com,
                                              ChMatrix33<>& inertia) {
    //// TODO
}

void LUTTireModel::CalculateForces(const ChFrameMoving<>& wheel_frame) {
    //// TODO: This is where the actual tire model is implemented
    ////       Calculate forces at each vertex, then use ChNodeFEAxyz::SetForce
}

// -----------------------------------------------------------------------------

void LUTTireModel::SyncCollisionModels() {
    if (m_contact_surf)
        m_contact_surf->SurfaceSyncCollisionModels();
}

void LUTTireModel::AddCollisionModelsToSystem() {
    assert(GetSystem());
    if (m_contact_surf) {
        m_contact_surf->SurfaceSyncCollisionModels();
        m_contact_surf->SurfaceAddCollisionModelsToSystem(GetSystem());
    }
}

void LUTTireModel::RemoveCollisionModelsFromSystem() {
    assert(GetSystem());
    if (m_contact_surf)
        m_contact_surf->SurfaceRemoveCollisionModelsFromSystem(GetSystem());
}

// -----------------------------------------------------------------------------

// Notes:
// The implementation of these functions is similar to those in ChMesh.
// It is assumed that none of the FEA nodes is fixed.

void LUTTireModel::SetupInitial() {
    // Calculate DOFs and state offsets
    m_dofs = 0;
    m_dofs_w = 0;
    for (auto& node : m_nodes) {
        node->SetupInitial(GetSystem());
        m_dofs += node->GetNdofX_active();
        m_dofs_w += node->GetNdofW_active();
    }
}

void LUTTireModel::Setup() {
    // Recompute DOFs and state offsets
    m_dofs = 0;
    m_dofs_w = 0;
    for (auto& node : m_nodes) {
        // Set node offsets in state vectors (based on the offsets of the container)
        node->NodeSetOffset_x(GetOffset_x() + m_dofs);
        node->NodeSetOffset_w(GetOffset_w() + m_dofs_w);

        // Count the actual degrees of freedom (consider only nodes that are not fixed)
        m_dofs += node->GetNdofX_active();
        m_dofs_w += node->GetNdofW_active();
    }

    // Calculate nodal forces
    CalculateForces(*m_wheel);

    // Update visualization mesh
    auto trimesh = m_trimesh_shape->GetMesh();
    auto& vertices = trimesh->getCoordsVertices();
    auto& normals = trimesh->getCoordsNormals();
    auto& colors = trimesh->getCoordsColors();

    for (int k = 0; k < m_num_nodes; k++) {
        vertices[k] = m_nodes[k]->GetPos();
    }

    //// TODO: update vertex normals and colors
}

void LUTTireModel::Update(double t, bool update_assets) {
    // Parent class update
    ChPhysicsItem::Update(t, update_assets);

    //// TODO: move call to CalculateForces() here?
}

// -----------------------------------------------------------------------------

void LUTTireModel::IntStateGather(const unsigned int off_x,
                                  ChState& x,
                                  const unsigned int off_v,
                                  ChStateDelta& v,
                                  double& T) {
    unsigned int local_off_x = 0;
    unsigned int local_off_v = 0;
    for (auto& node : m_nodes) {
        node->NodeIntStateGather(off_x + local_off_x, x, off_v + local_off_v, v, T);
        local_off_x += node->GetNdofX_active();
        local_off_v += node->GetNdofW_active();
    }

    T = GetChTime();
}

void LUTTireModel::IntStateScatter(const unsigned int off_x,
                                   const ChState& x,
                                   const unsigned int off_v,
                                   const ChStateDelta& v,
                                   const double T,
                                   bool full_update) {
    unsigned int local_off_x = 0;
    unsigned int local_off_v = 0;
    for (auto& node : m_nodes) {
        node->NodeIntStateScatter(off_x + local_off_x, x, off_v + local_off_v, v, T);
        local_off_x += node->GetNdofX_active();
        local_off_v += node->GetNdofW_active();
    }

    Update(T, full_update);
}

void LUTTireModel::IntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) {
    unsigned int local_off_a = 0;
    for (auto& node : m_nodes) {
        node->NodeIntStateGatherAcceleration(off_a + local_off_a, a);
        local_off_a += node->GetNdofW_active();
    }
}

void LUTTireModel::IntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) {
    unsigned int local_off_a = 0;
    for (auto& node : m_nodes) {
        node->NodeIntStateScatterAcceleration(off_a + local_off_a, a);
        local_off_a += node->GetNdofW_active();
    }
}

void LUTTireModel::IntStateIncrement(const unsigned int off_x,
                                     ChState& x_new,
                                     const ChState& x,
                                     const unsigned int off_v,
                                     const ChStateDelta& Dv) {
    unsigned int local_off_x = 0;
    unsigned int local_off_v = 0;
    for (auto& node : m_nodes) {
        node->NodeIntStateIncrement(off_x + local_off_x, x_new, x, off_v + local_off_v, Dv);
        local_off_x += node->GetNdofX_active();
        local_off_v += node->GetNdofW_active();
    }
}

void LUTTireModel::IntStateGetIncrement(const unsigned int off_x,
                                        const ChState& x_new,
                                        const ChState& x,
                                        const unsigned int off_v,
                                        ChStateDelta& Dv) {
    unsigned int local_off_x = 0;
    unsigned int local_off_v = 0;
    for (auto& node : m_nodes) {
        node->NodeIntStateGetIncrement(off_x + local_off_x, x_new, x, off_v + local_off_v, Dv);
        local_off_x += node->GetNdofX_active();
        local_off_v += node->GetNdofW_active();
    }
}

void LUTTireModel::IntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) {
    // Applied nodal forces (calculated in CalculateForces)
    unsigned int local_off_v = 0;
    for (auto& node : m_nodes) {
        node->NodeIntLoadResidual_F(off + local_off_v, R, c);
        local_off_v += node->GetNdofW_active();
    }

    // Gravitational nodal forces
    local_off_v = 0;
    if (system) {
        for (auto& node : m_nodes) {
            ChVector<> fg = c * node->GetMass() * system->Get_G_acc();
            R.segment(off + local_off_v, 3) += fg.eigen();
            local_off_v += node->GetNdofW_active();
        }
    }
}

void LUTTireModel::IntLoadResidual_Mv(const unsigned int off,
                                      ChVectorDynamic<>& R,
                                      const ChVectorDynamic<>& w,
                                      const double c) {
    unsigned int local_off_v = 0;
    for (auto& node : m_nodes) {
        node->NodeIntLoadResidual_Mv(off + local_off_v, R, w, c);
        local_off_v += node->GetNdofW_active();
    }
}

void LUTTireModel::IntToDescriptor(const unsigned int off_v,
                                   const ChStateDelta& v,
                                   const ChVectorDynamic<>& R,
                                   const unsigned int off_L,
                                   const ChVectorDynamic<>& L,
                                   const ChVectorDynamic<>& Qc) {
    unsigned int local_off_v = 0;
    for (auto& node : m_nodes) {
        node->NodeIntToDescriptor(off_v + local_off_v, v, R);
        local_off_v += node->GetNdofW_active();
    }
}

void LUTTireModel::IntFromDescriptor(const unsigned int off_v,
                                     ChStateDelta& v,
                                     const unsigned int off_L,
                                     ChVectorDynamic<>& L) {
    unsigned int local_off_v = 0;
    for (auto& node : m_nodes) {
        node->NodeIntFromDescriptor(off_v + local_off_v, v);
        local_off_v += node->GetNdofW_active();
    }
}

}  // end namespace vehicle
}  // end namespace chrono
