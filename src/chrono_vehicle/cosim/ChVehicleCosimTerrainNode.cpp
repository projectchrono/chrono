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
// Definition of the base vehicle co-simulation TERRAIN NODE class.
//
// Implementation of the base class TERRAIN NODE.
//
// The global reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include <fstream>
#include <algorithm>
#include <cmath>

#include "chrono_vehicle/cosim/ChVehicleCosimTerrainNode.h"

#include "chrono_thirdparty/rapidjson/filereadstream.h"
#include "chrono_thirdparty/rapidjson/istreamwrapper.h"

using std::cout;
using std::endl;

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Construction of the base terrain node.
// -----------------------------------------------------------------------------
ChVehicleCosimTerrainNode::ChVehicleCosimTerrainNode(double length, double width)
    : ChVehicleCosimBaseNode("TERRAIN"),
      m_hdimX(length / 2),
      m_hdimY(width / 2),
      m_load_mass(50),
      m_render(false),
      m_render_step(0.01),
      m_interface_type(InterfaceType::BODY) {}

// -----------------------------------------------------------------------------

void ChVehicleCosimTerrainNode::EnableRuntimeVisualization(bool render, double render_fps) {
    m_render = render;
    m_render_step = 1.0 / render_fps;
}

void ChVehicleCosimTerrainNode::SetDimensions(double length, double width) {
    m_hdimX = length / 2;
    m_hdimY = width / 2;
}

// -----------------------------------------------------------------------------
// Initialization of the terrain node(s):
// - send terrain height
// - receive information on tire mesh topology (number vertices and triangles)
// - receive tire contact material properties and create the "tire" material
// - create the appropriate proxy bodies (state not set yet)
// Note:
// Only the main terrain node participates in the co-simulation data exchange.
// -----------------------------------------------------------------------------
void ChVehicleCosimTerrainNode::Initialize() {
    // Invoke the base class method to figure out distribution of node types
    ChVehicleCosimBaseNode::Initialize();

    m_tire_radius.resize(m_num_tire_nodes);
    m_tire_width.resize(m_num_tire_nodes);
    m_load_mass.resize(m_num_tire_nodes);
    m_mat_props.resize(m_num_tire_nodes);
    m_mesh_data.resize(m_num_tire_nodes);
    m_mesh_state.resize(m_num_tire_nodes);
    m_mesh_contact.resize(m_num_tire_nodes);
    m_spindle_state.resize(m_num_tire_nodes);
    m_wheel_contact.resize(m_num_tire_nodes);

    if (m_rank == TERRAIN_NODE_RANK) {
        // -----------------------------------------
        // Send terrain patch dimensions to MBS node
        // -----------------------------------------

        // Note: take into account dimension of proxy bodies
        double init_dim[3] = {GetInitHeight() + 0.05, 2 * m_hdimX, 2 * m_hdimY};
        MPI_Send(init_dim, 3, MPI_DOUBLE, MBS_NODE_RANK, 0, MPI_COMM_WORLD);

        if (m_verbose) {
            cout << "[Terrain node] Sent initial terrain height = " << init_dim[0] << endl;
            cout << "[Terrain node] Sent terrain length = " << init_dim[1] << endl;
            cout << "[Terrain node] Sent terrain width = " << init_dim[2] << endl;
        }

        // -------------------------------------------------
        // Receive requested interface type from TIRE node 0
        // -------------------------------------------------

        MPI_Status status;
        char interface_type;
        MPI_Recv(&interface_type, 1, MPI_CHAR, TIRE_NODE_RANK(0), 0, MPI_COMM_WORLD, &status);
        m_interface_type = (interface_type == 0) ? InterfaceType::BODY : InterfaceType::MESH;

        // ------------------------
        // Receive tire information
        // ------------------------

        for (unsigned int i = 0; i < m_num_tire_nodes; i++) {
            // Tire radius
            double tire_info[3];
            MPI_Recv(tire_info, 3, MPI_DOUBLE, TIRE_NODE_RANK(i), 0, MPI_COMM_WORLD, &status);
            m_tire_radius[i] = tire_info[1];
            m_tire_width[i] = tire_info[2];

            // Tire contact surface specification

            unsigned int surf_props[3];
            MPI_Recv(surf_props, 3, MPI_UNSIGNED, TIRE_NODE_RANK(i), 0, MPI_COMM_WORLD, &status);

            m_mesh_data[i].nv = surf_props[0];
            m_mesh_data[i].nn = surf_props[1];
            m_mesh_data[i].nt = surf_props[2];

            if (m_interface_type == InterfaceType::MESH && !SupportsMeshInterface()) {
                cout << "ERROR: terrain system does not support the MESH interface type!" << endl;
                MPI_Abort(MPI_COMM_WORLD, 1);
            }

            m_mesh_data[i].verts.resize(m_mesh_data[i].nv);
            m_mesh_data[i].norms.resize(m_mesh_data[i].nn);
            m_mesh_data[i].idx_verts.resize(m_mesh_data[i].nt);
            m_mesh_data[i].idx_norms.resize(m_mesh_data[i].nt);

            m_mesh_state[i].vpos.resize(m_mesh_data[i].nv);
            m_mesh_state[i].vvel.resize(m_mesh_data[i].nv);

            // Tire mesh vertices & normals and triangle indices

            double* vert_data = new double[3 * m_mesh_data[i].nv + 3 * m_mesh_data[i].nn];
            int* tri_data = new int[3 * m_mesh_data[i].nt + 3 * m_mesh_data[i].nt];
            MPI_Recv(vert_data, 3 * m_mesh_data[i].nv + 3 * m_mesh_data[i].nn, MPI_DOUBLE, TIRE_NODE_RANK(i), 0,
                     MPI_COMM_WORLD, &status);
            MPI_Recv(tri_data, 3 * m_mesh_data[i].nt + 3 * m_mesh_data[i].nt, MPI_INT, TIRE_NODE_RANK(i), 0,
                     MPI_COMM_WORLD, &status);

            for (unsigned int iv = 0; iv < m_mesh_data[i].nv; iv++) {
                m_mesh_data[i].verts[iv].x() = vert_data[3 * iv + 0];
                m_mesh_data[i].verts[iv].y() = vert_data[3 * iv + 1];
                m_mesh_data[i].verts[iv].z() = vert_data[3 * iv + 2];
            }
            for (unsigned int in = 0; in < m_mesh_data[i].nn; in++) {
                m_mesh_data[i].norms[in].x() = vert_data[3 * m_mesh_data[i].nv + 3 * in + 0];
                m_mesh_data[i].norms[in].y() = vert_data[3 * m_mesh_data[i].nv + 3 * in + 1];
                m_mesh_data[i].norms[in].z() = vert_data[3 * m_mesh_data[i].nv + 3 * in + 2];
            }
            for (unsigned int it = 0; it < m_mesh_data[i].nt; it++) {
                m_mesh_data[i].idx_verts[it].x() = tri_data[6 * it + 0];
                m_mesh_data[i].idx_verts[it].y() = tri_data[6 * it + 1];
                m_mesh_data[i].idx_verts[it].z() = tri_data[6 * it + 2];
                m_mesh_data[i].idx_norms[it].x() = tri_data[6 * it + 3];
                m_mesh_data[i].idx_norms[it].y() = tri_data[6 * it + 4];
                m_mesh_data[i].idx_norms[it].z() = tri_data[6 * it + 5];
            }

            delete[] vert_data;
            delete[] tri_data;

            if (m_verbose)
                cout << "[Terrain node] Received " << surf_props[0] << " vertices and " << surf_props[2] << " triangles"
                     << endl;

            // Load mass
            MPI_Recv(&m_load_mass[i], 1, MPI_DOUBLE, TIRE_NODE_RANK(i), 0, MPI_COMM_WORLD, &status);

            if (m_verbose)
                cout << "[Terrain node] received load mass = " << m_load_mass[i] << endl;

            // Tire contact material properties

            float props[8];
            MPI_Recv(props, 8, MPI_FLOAT, TIRE_NODE_RANK(i), 0, MPI_COMM_WORLD, &status);

            m_mat_props[i].mu = props[0];
            m_mat_props[i].cr = props[1];
            m_mat_props[i].Y = props[2];
            m_mat_props[i].nu = props[3];
            m_mat_props[i].kn = props[4];
            m_mat_props[i].gn = props[5];
            m_mat_props[i].kt = props[6];
            m_mat_props[i].gt = props[7];

            if (m_verbose)
                cout << "[Terrain node] received tire material:  friction = " << props[0] << endl;
        }
    }

    // ----------------------------------------------------
    // Let derived classes perform their own initialization
    // ----------------------------------------------------
    OnInitialize(m_num_tire_nodes);
}

// -----------------------------------------------------------------------------
// Synchronization of the terrain node:
// - receive tire mesh vertex states and set states of proxy bodies
// - calculate current cumulative contact forces on all system bodies
// - extract and send forces at each vertex
// Note:
// Only the main terrain node participates in the co-simulation data exchange.
// -----------------------------------------------------------------------------
void ChVehicleCosimTerrainNode::Synchronize(int step_number, double time) {
    switch (m_interface_type) {
        case InterfaceType::BODY:
            SynchronizeBody(step_number, time);
            break;
        case InterfaceType::MESH:
            SynchronizeMesh(step_number, time);
            break;
    }

    // Let derived classes perform optional operations
    OnSynchronize(step_number, time);
}

void ChVehicleCosimTerrainNode::SynchronizeBody(int step_number, double time) {
    for (unsigned int i = 0; i < m_num_tire_nodes; i++) {
        if (m_rank == TERRAIN_NODE_RANK) {
            // Receive spindle state data
            MPI_Status status;
            double state_data[13];
            MPI_Recv(state_data, 13, MPI_DOUBLE, TIRE_NODE_RANK(i), step_number, MPI_COMM_WORLD, &status);

            m_spindle_state[i].pos = ChVector<>(state_data[0], state_data[1], state_data[2]);
            m_spindle_state[i].rot = ChQuaternion<>(state_data[3], state_data[4], state_data[5], state_data[6]);
            m_spindle_state[i].lin_vel = ChVector<>(state_data[7], state_data[8], state_data[9]);
            m_spindle_state[i].ang_vel = ChVector<>(state_data[10], state_data[11], state_data[12]);
        }

        // Set position, rotation, and velocities of wheel proxy body.
        UpdateWheelProxy(i, m_spindle_state[i]);

        // Collect contact force on wheel proxy and load in m_wheel_contact.
        // It is assumed that this force is given at spindle center.
        // Note that no force is collected at the first step.
        if (step_number > 0) {
            GetForceWheelProxy(i, m_wheel_contact[i]);
        }

        if (m_rank == TERRAIN_NODE_RANK) {
            // Send wheel contact force.
            double force_data[] = {m_wheel_contact[i].force.x(),  m_wheel_contact[i].force.y(),
                                   m_wheel_contact[i].force.z(),  m_wheel_contact[i].moment.x(),
                                   m_wheel_contact[i].moment.y(), m_wheel_contact[i].moment.z()};
            MPI_Send(force_data, 6, MPI_DOUBLE, TIRE_NODE_RANK(i), step_number, MPI_COMM_WORLD);

            if (m_verbose)
                cout << "[Terrain node] step number: " << step_number << "  num contacts: " << GetNumContacts() << endl;
        }
    }
}

void ChVehicleCosimTerrainNode::SynchronizeMesh(int step_number, double time) {
    for (unsigned int i = 0; i < m_num_tire_nodes; i++) {
        if (m_rank == TERRAIN_NODE_RANK) {
            // Receive mesh state data
            MPI_Status status;
            double* vert_data = new double[2 * 3 * m_mesh_data[i].nv];
            MPI_Recv(vert_data, 2 * 3 * m_mesh_data[i].nv, MPI_DOUBLE, TIRE_NODE_RANK(i), step_number, MPI_COMM_WORLD,
                     &status);

            for (unsigned int iv = 0; iv < m_mesh_data[i].nv; iv++) {
                unsigned int offset = 3 * iv;
                m_mesh_state[i].vpos[iv] =
                    ChVector<>(vert_data[offset + 0], vert_data[offset + 1], vert_data[offset + 2]);
                offset += 3 * m_mesh_data[i].nv;
                m_mesh_state[i].vvel[iv] =
                    ChVector<>(vert_data[offset + 0], vert_data[offset + 1], vert_data[offset + 2]);
            }

            ////if (m_verbose)
            ////    PrintMeshUpdateData(i);

            delete[] vert_data;
        }

        // Set position, rotation, and velocity of proxy bodies.
        UpdateMeshProxies(i, m_mesh_state[i]);

        // Collect contact forces on subset of mesh vertices and load in m_mesh_contact.
        // Note that no forces are collected at the first step.
        if (step_number == 0)
            m_mesh_contact[i].nv = 0;
        else
            GetForcesMeshProxies(i, m_mesh_contact[i]);

        if (m_rank == TERRAIN_NODE_RANK) {
            // Send vertex indices and forces.
            double* force_data = new double[3 * m_mesh_contact[i].nv];
            for (int iv = 0; iv < m_mesh_contact[i].nv; i++) {
                force_data[3 * iv + 0] = m_mesh_contact[i].vforce[iv].x();
                force_data[3 * iv + 1] = m_mesh_contact[i].vforce[iv].y();
                force_data[3 * iv + 2] = m_mesh_contact[i].vforce[iv].z();
            }
            MPI_Send(m_mesh_contact[i].vidx.data(), m_mesh_contact[i].nv, MPI_INT, TIRE_NODE_RANK(i), step_number,
                     MPI_COMM_WORLD);
            MPI_Send(force_data, 3 * m_mesh_contact[i].nv, MPI_DOUBLE, TIRE_NODE_RANK(i), step_number, MPI_COMM_WORLD);

            delete[] force_data;

            if (m_verbose)
                cout << "[Terrain node] step number: " << step_number << "  num contacts: " << GetNumContacts()
                     << "  vertices in contact: " << m_mesh_contact[i].nv << endl;
        }
    }
}

// -----------------------------------------------------------------------------
// Advance simulation of the terrain node by the specified duration
// -----------------------------------------------------------------------------
void ChVehicleCosimTerrainNode::Advance(double step_size) {
    static double sim_time = 0;
    static double render_time = 0;

    // Let derived classes advance the terrain state
    m_timer.reset();
    m_timer.start();
    OnAdvance(step_size);
    sim_time += step_size;
    m_timer.stop();
    m_cum_sim_time += m_timer();

    // Request the derived class to render simulation
    if (m_render && sim_time >= render_time) {
        Render(sim_time);
        render_time += std::max(m_render_step, step_size);
    }
}

// -----------------------------------------------------------------------------

void ChVehicleCosimTerrainNode::OutputData(int frame) {
    // Append to results output file
    if (m_outf.is_open()) {
        //// TODO
    }

    OnOutputData(frame);
}

// Print mesh vertex data, as received at synchronization.
void ChVehicleCosimTerrainNode::PrintMeshUpdateData(unsigned int i) {
    cout << "[Terrain node] mesh vertices and faces" << endl;
    std::for_each(m_mesh_state[i].vpos.begin(), m_mesh_state[i].vpos.end(),
                  [](const ChVector<>& a) { cout << a.x() << "  " << a.y() << "  " << a.z() << endl; });
}

}  // end namespace vehicle
}  // end namespace chrono
