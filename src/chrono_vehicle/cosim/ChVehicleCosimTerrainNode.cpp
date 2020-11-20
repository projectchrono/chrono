// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2020 projectchrono.org
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
// Mechanism for testing tires over granular terrain.  The mechanism + tire
// system is co-simulated with a Chrono::Parallel system for the granular terrain.
//
// Implementation of the base class TERRAIN NODE.
//
// The global reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

//// RADU TODO:
////    mesh connectivity doesn't need to be communicated every time (modify Chrono?)

#include <algorithm>
#include <cmath>

#include <mpi.h>

#include "chrono_vehicle/cosim/ChVehicleCosimTerrainNode.h"

using std::cout;
using std::endl;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Construction of the terrain node:
// - create the (parallel) Chrono system and set solver parameters
// - create the OpenGL visualization window
// -----------------------------------------------------------------------------
ChVehicleCosimTerrainNode::ChVehicleCosimTerrainNode(Type type, ChContactMethod method, bool render)
    : ChVehicleCosimBaseNode("TERRAIN"), m_type(type), m_method(method), m_render(render), m_init_height(0) {
    // Default container dimensions
    m_hdimX = 1.0;
    m_hdimY = 0.25;
    m_hdimZ = 0.5;
    m_hthick = 0.1;

    // Default proxy body properties
    m_fixed_proxies = false;
    m_mass_p = 1;
    m_radius_p = 0.01;

    // Default terrain contact material
    switch (m_method) {
        case ChContactMethod::SMC:
            m_material_terrain = chrono_types::make_shared<ChMaterialSurfaceSMC>();
            break;
        case ChContactMethod::NSC:
            m_material_terrain = chrono_types::make_shared<ChMaterialSurfaceNSC>();
            break;
    }
}

// -----------------------------------------------------------------------------

void ChVehicleCosimTerrainNode::SetContainerDimensions(double length, double width, double height, double thickness) {
    m_hdimX = length / 2;
    m_hdimY = width / 2;
    m_hdimZ = height / 2;
    m_hthick = thickness / 2;
}

void ChVehicleCosimTerrainNode::SetProxyProperties(double mass, double radius, bool fixed) {
    m_mass_p = mass;
    m_radius_p = radius;
    m_fixed_proxies = fixed;
}

// -----------------------------------------------------------------------------
// Initialization of the terrain node:
// - if not already done, complete system construction
// - send terrain height
// - receive information on tire mesh topology (number vertices and triangles)
// - receive tire contact material properties and create the "tire" material
// - create the appropriate proxy bodies (state not set yet)
// -----------------------------------------------------------------------------
void ChVehicleCosimTerrainNode::Initialize() {
    Construct();

    // Reset system time
    GetSystem()->SetChTime(0);

    // ------------------------------------------
    // Send information for initial tire location
    // ------------------------------------------

    // This includes the terrain height and the container half-length.
    // Note: take into account dimension of proxy bodies
    double init_dim[2] = {m_init_height + m_radius_p, m_hdimX};
    MPI_Send(init_dim, 2, MPI_DOUBLE, RIG_NODE_RANK, 0, MPI_COMM_WORLD);

    cout << "[Terrain node] Sent initial terrain height = " << init_dim[0] << endl;
    cout << "[Terrain node] Sent container half-length = " << init_dim[1] << endl;

    // ------------------------------------------
    // Receive tire contact surface specification
    // ------------------------------------------

    unsigned int surf_props[3];
    MPI_Status status_p;
    MPI_Recv(surf_props, 3, MPI_UNSIGNED, RIG_NODE_RANK, 0, MPI_COMM_WORLD, &status_p);

    m_rigid_tire = (surf_props[0] == 1);
    m_mesh_data.nv = surf_props[1];
    m_mesh_data.nt = surf_props[2];

    m_mesh_data.vpos.resize(m_mesh_data.nv);
    m_mesh_data.tri.resize(m_mesh_data.nt);

    m_mesh_state.vpos.resize(m_mesh_data.nv);
    m_mesh_state.vvel.resize(m_mesh_data.nv);

    cout << "[Terrain node] Received vertices = " << surf_props[0] << " triangles = " << surf_props[1] << endl;

    // -----------------------------------------------
    // Receive tire mesh vertices and triangle indices
    // -----------------------------------------------

    MPI_Status status_v;
    double* vert_data = new double[3 * m_mesh_data.nv];
    int* tri_data = new int[3 * m_mesh_data.nt];
    MPI_Recv(vert_data, 3 * m_mesh_data.nv, MPI_DOUBLE, RIG_NODE_RANK, 0, MPI_COMM_WORLD, &status_v);
    MPI_Recv(tri_data, 3 * m_mesh_data.nt, MPI_INT, RIG_NODE_RANK, 0, MPI_COMM_WORLD, &status_v);

    for (unsigned int iv = 0; iv < m_mesh_data.nv; iv++) {
        m_mesh_data.vpos[iv].x() = vert_data[3 * iv + 0];
        m_mesh_data.vpos[iv].y() = vert_data[3 * iv + 1];
        m_mesh_data.vpos[iv].z() = vert_data[3 * iv + 2];
    }

    for (unsigned int it = 0; it < m_mesh_data.nt; it++) {
        m_mesh_data.tri[it].x() = tri_data[3 * it + 0];
        m_mesh_data.tri[it].y() = tri_data[3 * it + 1];
        m_mesh_data.tri[it].z() = tri_data[3 * it + 2];
    }

    delete[] vert_data;
    delete[] tri_data;

    // ----------------------------------------
    // Receive tire contact material properties
    // ----------------------------------------

    // Create the "tire" contact material, but defer using it until the proxy bodies are created.
    float mat_props[8];
    MPI_Status status_m;
    MPI_Recv(mat_props, 8, MPI_FLOAT, RIG_NODE_RANK, 0, MPI_COMM_WORLD, &status_m);

    switch (m_method) {
        case ChContactMethod::SMC: {
            // Properties for tire
            auto mat_tire = chrono_types::make_shared<ChMaterialSurfaceSMC>();
            mat_tire->SetFriction(mat_props[0]);
            mat_tire->SetRestitution(mat_props[1]);
            mat_tire->SetYoungModulus(mat_props[2]);
            mat_tire->SetPoissonRatio(mat_props[3]);
            mat_tire->SetKn(mat_props[4]);
            mat_tire->SetGn(mat_props[5]);
            mat_tire->SetKt(mat_props[6]);
            mat_tire->SetGt(mat_props[7]);

            m_material_tire = mat_tire;

            break;
        }
        case ChContactMethod::NSC: {
            auto mat_tire = chrono_types::make_shared<ChMaterialSurfaceNSC>();
            mat_tire->SetFriction(mat_props[0]);
            mat_tire->SetRestitution(mat_props[1]);

            m_material_tire = mat_tire;

            break;
        }
    }

    cout << "[Terrain node] received tire material:  friction = " << mat_props[0] << endl;

    // -------------------
    // Create proxy bodies
    // -------------------

    CreateProxies();
}

// -----------------------------------------------------------------------------
// Synchronization of the terrain node:
// - receive tire mesh vertex states and set states of proxy bodies
// - calculate current cumulative contact forces on all system bodies
// - extract and send forces at each vertex
// -----------------------------------------------------------------------------
void ChVehicleCosimTerrainNode::Synchronize(int step_number, double time) {
    // Receive tire mesh vertex locations and velocities.
    MPI_Status status;
    double* vert_data = new double[2 * 3 * m_mesh_data.nv];
    MPI_Recv(vert_data, 2 * 3 * m_mesh_data.nv, MPI_DOUBLE, RIG_NODE_RANK, step_number, MPI_COMM_WORLD, &status);

    for (unsigned int iv = 0; iv < m_mesh_data.nv; iv++) {
        unsigned int offset = 3 * iv;
        m_mesh_state.vpos[iv] = ChVector<>(vert_data[offset + 0], vert_data[offset + 1], vert_data[offset + 2]);
        offset += 3 * m_mesh_data.nv;
        m_mesh_state.vvel[iv] = ChVector<>(vert_data[offset + 0], vert_data[offset + 1], vert_data[offset + 2]);
    }

    delete[] vert_data;

    ////PrintMeshUpdateData();

    // Set position, rotation, and velocity of proxy bodies.
    UpdateProxies();
    PrintProxiesUpdateData();

    // Collect contact forces on subset of mesh vertices.
    // Note that no forces are collected at the first step.
    std::vector<double> vert_forces;
    std::vector<int> vert_indices;

    if (step_number > 0) {
        ForcesProxies(vert_forces, vert_indices);
    }

    // Send vertex indices and forces.
    int num_vert = (int)vert_indices.size();
    MPI_Send(vert_indices.data(), num_vert, MPI_INT, RIG_NODE_RANK, step_number, MPI_COMM_WORLD);
    MPI_Send(vert_forces.data(), 3 * num_vert, MPI_DOUBLE, RIG_NODE_RANK, step_number, MPI_COMM_WORLD);

    cout << "[Terrain node] step number: " << step_number << "  num contacts: " << GetSystem()->GetNcontacts()
         << "  vertices in contact: " << num_vert << endl;

    // Let derived classes perform optional operations
    OnSynchronize(step_number, time);
}

// -----------------------------------------------------------------------------
// Advance simulation of the terrain node by the specified duration
// -----------------------------------------------------------------------------
void ChVehicleCosimTerrainNode::Advance(double step_size) {
    m_timer.reset();
    m_timer.start();
    double t = 0;
    while (t < step_size) {
        double h = std::min<>(m_step_size, step_size - t);
        GetSystem()->DoStepDynamics(h);
        t += h;
    }
    m_timer.stop();
    m_cum_sim_time += m_timer();

    // Let derived classes perform optional operations (e.g. rendering)
    OnAdvance(step_size);

    PrintProxiesContactData();
}

// -----------------------------------------------------------------------------

void ChVehicleCosimTerrainNode::OutputData(int frame) {
    // Append to results output file
    if (m_outf.is_open()) {
        //// TODO
    }

    OutputTerrainData(frame);
}

// Print vertex and face connectivity data, as received from the rig node at synchronization.
void ChVehicleCosimTerrainNode::PrintMeshUpdateData() {
    cout << "[Terrain node] mesh vertices and faces" << endl;
    std::for_each(m_mesh_state.vpos.begin(), m_mesh_state.vpos.end(),
                  [](const ChVector<>& a) { cout << a.x() << "  " << a.y() << "  " << a.z() << endl; });
    std::for_each(m_mesh_data.tri.begin(), m_mesh_data.tri.end(),
                  [](const ChVector<int>& a) { cout << a.x() << "  " << a.y() << "  " << a.z() << endl; });
}

}  // end namespace vehicle
}  // end namespace chrono
