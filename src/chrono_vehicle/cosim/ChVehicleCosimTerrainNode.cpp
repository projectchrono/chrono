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
    // Default patch dimensions
    m_hdimX = 1.0;
    m_hdimY = 0.25;

    // Default proxy body properties
    m_fixed_proxies = false;

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

void ChVehicleCosimTerrainNode::SetPatchDimensions(double length, double width) {
    m_hdimX = length / 2;
    m_hdimY = width / 2;
}

void ChVehicleCosimTerrainNode::SetProxyFixed(bool fixed) {
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
    double init_dim[2] = {m_init_height + 0.05, m_hdimX};
    MPI_Send(init_dim, 2, MPI_DOUBLE, RIG_NODE_RANK, 0, MPI_COMM_WORLD);

    cout << "[Terrain node] Sent initial terrain height = " << init_dim[0] << endl;
    cout << "[Terrain node] Sent container half-length = " << init_dim[1] << endl;

    // ------------------------------------------
    // Receive tire contact surface specification
    // ------------------------------------------

    MPI_Status status;

    unsigned int surf_props[3];
    MPI_Recv(surf_props, 3, MPI_UNSIGNED, RIG_NODE_RANK, 0, MPI_COMM_WORLD, &status);

    m_flexible_tire = (surf_props[0] == 1);
    m_mesh_data.nv = surf_props[1];
    m_mesh_data.nt = surf_props[2];

    if (m_flexible_tire && !SupportsFlexibleTire()) {
        cout << "ERROR: terrain system does not support flexible tires!" << endl;
        MPI_Abort(MPI_COMM_WORLD, 1);
    }

    m_mesh_data.vpos.resize(m_mesh_data.nv);
    m_mesh_data.vnrm.resize(m_mesh_data.nv);
    m_mesh_data.tri.resize(m_mesh_data.nt);

    m_mesh_state.vpos.resize(m_mesh_data.nv);
    m_mesh_state.vvel.resize(m_mesh_data.nv);

    // -----------------------------------------------------------
    // Receive tire mesh vertices & normals and triangle indices
    // -----------------------------------------------------------

    double* vert_data = new double[2 * 3 * m_mesh_data.nv];
    int* tri_data = new int[3 * m_mesh_data.nt];
    MPI_Recv(vert_data, 2 * 3 * m_mesh_data.nv, MPI_DOUBLE, RIG_NODE_RANK, 0, MPI_COMM_WORLD, &status);
    MPI_Recv(tri_data, 3 * m_mesh_data.nt, MPI_INT, RIG_NODE_RANK, 0, MPI_COMM_WORLD, &status);

    for (unsigned int iv = 0; iv < m_mesh_data.nv; iv++) {
        m_mesh_data.vpos[iv].x() = vert_data[6 * iv + 0];
        m_mesh_data.vpos[iv].y() = vert_data[6 * iv + 1];
        m_mesh_data.vpos[iv].z() = vert_data[6 * iv + 2];
        m_mesh_data.vnrm[iv].x() = vert_data[6 * iv + 3];
        m_mesh_data.vnrm[iv].y() = vert_data[6 * iv + 4];
        m_mesh_data.vnrm[iv].z() = vert_data[6 * iv + 5];
    }

    for (unsigned int it = 0; it < m_mesh_data.nt; it++) {
        m_mesh_data.tri[it].x() = tri_data[3 * it + 0];
        m_mesh_data.tri[it].y() = tri_data[3 * it + 1];
        m_mesh_data.tri[it].z() = tri_data[3 * it + 2];
    }

    delete[] vert_data;
    delete[] tri_data;

    cout << "[Terrain node] Received " << surf_props[1] << " vertices and " << surf_props[2] << " triangles" << endl;

    // ----------------
    // Receive rig mass
    // ----------------

    MPI_Recv(&m_rig_mass, 1, MPI_DOUBLE, RIG_NODE_RANK, 0, MPI_COMM_WORLD, &status);

    cout << "[Terrain node] received rig mass = " << m_rig_mass << endl;

    // ----------------------------------------
    // Receive tire contact material properties
    // ----------------------------------------

    // Create the "tire" contact material, but defer using it until the proxy bodies are created.
    float mat_props[8];
    MPI_Recv(mat_props, 8, MPI_FLOAT, RIG_NODE_RANK, 0, MPI_COMM_WORLD, &status);

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

    if (m_flexible_tire)
        CreateMeshProxies();
    else
        CreateWheelProxy();
}

// -----------------------------------------------------------------------------
// Synchronization of the terrain node:
// - receive tire mesh vertex states and set states of proxy bodies
// - calculate current cumulative contact forces on all system bodies
// - extract and send forces at each vertex
// -----------------------------------------------------------------------------
void ChVehicleCosimTerrainNode::Synchronize(int step_number, double time) {
    if (m_flexible_tire)
        SynchronizeFlexibleTire(step_number, time);
    else
        SynchronizeRigidTire(step_number, time);

    // Let derived classes perform optional operations
    OnSynchronize(step_number, time);
}

void ChVehicleCosimTerrainNode::SynchronizeRigidTire(int step_number, double time) {
    // Receive wheel state data
    MPI_Status status;
    double state_data[14];
    MPI_Recv(state_data, 14, MPI_DOUBLE, RIG_NODE_RANK, step_number, MPI_COMM_WORLD, &status);

    m_wheel_state.pos = ChVector<>(state_data[0], state_data[1], state_data[2]);
    m_wheel_state.rot = ChQuaternion<>(state_data[3], state_data[4], state_data[5], state_data[6]);
    m_wheel_state.lin_vel = ChVector<>(state_data[7], state_data[8], state_data[9]);
    m_wheel_state.ang_vel = ChVector<>(state_data[10], state_data[11], state_data[12]);
    m_wheel_state.omega = state_data[13];

    // Set position, rotation, and velocities of wheel proxy body.
    UpdateWheelProxy();
    PrintWheelProxyUpdateData();

    // Collect contact force on wheel proxy and load in m_wheel_contact. 
    // It is assumed that this force is given at wheel center.
    // Note that no force is collected at the first step.
    if (step_number > 0) {
        GetForceWheelProxy();
    }

    // Send wheel contact force.
    double force_data[] = {m_wheel_contact.force.x(),  m_wheel_contact.force.y(),  m_wheel_contact.force.z(),
                           m_wheel_contact.moment.x(), m_wheel_contact.moment.y(), m_wheel_contact.moment.z()};
    MPI_Send(force_data, 6, MPI_DOUBLE, RIG_NODE_RANK, step_number, MPI_COMM_WORLD);

    cout << "[Terrain node] step number: " << step_number << "  num contacts: " << GetSystem()->GetNcontacts() << endl;
}

void ChVehicleCosimTerrainNode::SynchronizeFlexibleTire(int step_number, double time) {
    // Receive mesh state data
    MPI_Status status;
    double* vert_data = new double[2 * 3 * m_mesh_data.nv];
    MPI_Recv(vert_data, 2 * 3 * m_mesh_data.nv, MPI_DOUBLE, RIG_NODE_RANK, step_number, MPI_COMM_WORLD, &status);

    for (unsigned int iv = 0; iv < m_mesh_data.nv; iv++) {
        unsigned int offset = 3 * iv;
        m_mesh_state.vpos[iv] = ChVector<>(vert_data[offset + 0], vert_data[offset + 1], vert_data[offset + 2]);
        offset += 3 * m_mesh_data.nv;
        m_mesh_state.vvel[iv] = ChVector<>(vert_data[offset + 0], vert_data[offset + 1], vert_data[offset + 2]);
    }

    ////PrintMeshUpdateData();

    delete[] vert_data;

    // Set position, rotation, and velocity of proxy bodies.
    UpdateMeshProxies();
    PrintMeshProxiesUpdateData();

    // Collect contact forces on subset of mesh vertices and load in m_mesh_contact.
    // Note that no forces are collected at the first step.
    if (step_number == 0)
        m_mesh_contact.nv = 0;
    else
        GetForcesMeshProxies();

    // Send vertex indices and forces.
    double* force_data = new double[3 * m_mesh_contact.nv];
    for (int i = 0; i < m_mesh_contact.nv; i++) {
        force_data[3 * i + 0] = m_mesh_contact.vforce[i].x();
        force_data[3 * i + 1] = m_mesh_contact.vforce[i].y();
        force_data[3 * i + 2] = m_mesh_contact.vforce[i].z();
    }
    MPI_Send(m_mesh_contact.vidx.data(), m_mesh_contact.nv, MPI_INT, RIG_NODE_RANK, step_number, MPI_COMM_WORLD);
    MPI_Send(force_data, 3 * m_mesh_contact.nv, MPI_DOUBLE, RIG_NODE_RANK, step_number, MPI_COMM_WORLD);

    delete[] force_data;

    cout << "[Terrain node] step number: " << step_number << "  num contacts: " << GetSystem()->GetNcontacts()
         << "  vertices in contact: " << m_mesh_contact.nv << endl;
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

    if (m_flexible_tire)
        PrintMeshProxiesContactData();
    else
        PrintWheelProxyContactData();
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
