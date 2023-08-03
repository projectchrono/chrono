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
      m_dimX(length / 2),
      m_dimY(width / 2),
      m_load_mass(50),
      m_interface_type(InterfaceType::BODY) {}

// -----------------------------------------------------------------------------

void ChVehicleCosimTerrainNode::SetDimensions(double length, double width) {
    m_dimX = length;
    m_dimY = width;
}

void ChVehicleCosimTerrainNode::GetDimensions(double& length, double& width) {
    length = m_dimX;
    width = m_dimY;
}

// -----------------------------------------------------------------------------
// Initialization of the terrain node(s):
// - send terrain height
// - receive information on object representation (primitive or mesh)
// - receive contact material properties
// - create the appropriate proxy bodies (state not set yet)
// Note:
// Only the main terrain node participates in the co-simulation data exchange.
// -----------------------------------------------------------------------------
void ChVehicleCosimTerrainNode::Initialize() {
    // Invoke the base class method to figure out distribution of node types
    ChVehicleCosimBaseNode::Initialize();

    // Decide if communicating with TIRE nodes or MBS node
    m_wheeled = (m_num_wheeled_mbs_nodes == 1) ? true : false;

    if (m_rank == TERRAIN_NODE_RANK) {
        // 1. Send terrain patch dimensions to MBS node

        // Note: take into account dimension of proxy bodies
        double init_dim[3] = {GetInitHeight() + 0.05, m_dimX, m_dimY};
        MPI_Send(init_dim, 3, MPI_DOUBLE, MBS_NODE_RANK, 0, MPI_COMM_WORLD);

        if (m_verbose) {
            cout << "[Terrain node] Send: initial terrain height = " << init_dim[0] << endl;
            cout << "[Terrain node] Send: terrain length = " << init_dim[1] << endl;
            cout << "[Terrain node] Send: terrain width = " << init_dim[2] << endl;
        }

        // 2. Receive number of interacting object from MBS node

        MPI_Status status;
        MPI_Recv(&m_num_objects, 1, MPI_INT, MBS_NODE_RANK, 0, MPI_COMM_WORLD, &status);

        // 3. Receive expected communication interface type

        char comm_type;
        if (m_wheeled) {
            // Receive from 1st TIRE node
            MPI_Recv(&comm_type, 1, MPI_CHAR, TIRE_NODE_RANK(0), 0, MPI_COMM_WORLD, &status);
        } else {
            // Receive from the tracked MBS node
            MPI_Recv(&comm_type, 1, MPI_CHAR, MBS_NODE_RANK, 0, MPI_COMM_WORLD, &status);
        }
        m_interface_type = (comm_type == 0) ? InterfaceType::BODY : InterfaceType::MESH;

        if (m_interface_type == InterfaceType::MESH && !SupportsMeshInterface()) {
            cout << "ERROR: terrain system does not support the MESH interface type!" << endl;
            MPI_Abort(MPI_COMM_WORLD, 1);
        }

        // 4. Resize arrays for co-simulation data exchange (one per interacting object)

        m_obj_map.resize(m_num_objects);

        if (m_interface_type == InterfaceType::MESH) {
            m_mesh_state.resize(m_num_objects);
            m_mesh_contact.resize(m_num_objects);
        }
        m_rigid_state.resize(m_num_objects);
        m_rigid_contact.resize(m_num_objects);

        // 5. Receive object information (geometry, contact materials, load mass)

        if (m_wheeled) {
            // Get tire geometry data from TIRE nodes
            InitializeTireData();
        } else {
            // Get track geometry data from tracked MBS node
            InitializeTrackData();
        }
    }

    // Let derived classes perform their own initialization
    OnInitialize(m_num_objects);
}

void ChVehicleCosimTerrainNode::InitializeTireData() {
    MPI_Status status;

    // Resize arrays with geometric object information (one per tire)
    m_aabb.resize(m_num_objects);
    m_geometry.resize(m_num_objects);
    m_load_mass.resize(m_num_objects);

    // Set mapping from objects to shapes (each tire has its own geometry)
    for (int i = 0; i < m_num_objects; i++)
        m_obj_map[i] = i;

    // Exchange data with each TIRE node
    for (int i = 0; i < m_num_objects; i++) {
        // Receive tire geometry
        RecvGeometry(m_geometry[i], TIRE_NODE_RANK(i));

        // If using MESH interface, there must be one and exactly one mesh
        if (m_interface_type == InterfaceType::MESH && m_geometry[i].m_coll_meshes.size() != 1) {
            cout << "ERROR: using MESH interface, but tire geometry does not include a mesh!" << endl;
            MPI_Abort(MPI_COMM_WORLD, 1);       
        }

        // Set size of collision model for this tire
        m_aabb[i] = m_geometry[0].CalculateAABB();

        // Resize mesh state vectors (if used)
        if (m_interface_type == InterfaceType::MESH) {
            int nv = m_geometry[i].m_coll_meshes[0].m_trimesh->getNumVertices();
            m_mesh_state[i].vpos.resize(nv);
            m_mesh_state[i].vvel.resize(nv);
        }

        // Receive load mass
        MPI_Recv(&m_load_mass[i], 1, MPI_DOUBLE, TIRE_NODE_RANK(i), 0, MPI_COMM_WORLD, &status);
        if (m_verbose)
            cout << "[Terrain node] Recv:  load_mass = " << m_load_mass[i] << endl;
    }
}

void ChVehicleCosimTerrainNode::InitializeTrackData() {
    MPI_Status status;

    // Resize arrays with geometric object information (same for all track shoes)
    m_aabb.resize(1);
    m_geometry.resize(1);
    m_load_mass.resize(1);

    // Set mapping from objects to shapes (all track shoes use the same geometry)
    for (int i = 0; i < m_num_objects; i++)
        m_obj_map[i] = 0;

    // Receive track shoe geometry from the tracked MBS node
    RecvGeometry(m_geometry[0], MBS_NODE_RANK);

    // If using MESH interface, there must be one and exactly one mesh
    if (m_interface_type == InterfaceType::MESH && m_geometry[0].m_coll_meshes.size() != 1) {
        cout << "ERROR: using MESH interface, but shoe geometry does not include a mesh!" << endl;
        MPI_Abort(MPI_COMM_WORLD, 1);
    }

    // Set size of collision model for each track shoe    
    m_aabb[0] = m_geometry[0].CalculateAABB();

    // Resize mesh state vectors (if used)
    if (m_interface_type == InterfaceType::MESH) {
        int nv = m_geometry[0].m_coll_meshes[0].m_trimesh->getNumVertices();
        m_mesh_state[0].vpos.resize(nv);
        m_mesh_state[0].vvel.resize(nv);
    }

    // Receive mass of a track shoe
    MPI_Recv(&m_load_mass[0], 1, MPI_DOUBLE, MBS_NODE_RANK, 0, MPI_COMM_WORLD, &status);
    if (m_verbose)
        cout << "[Terrain node] Recv:  load_mass = " << m_load_mass[0] << endl;
}

// -----------------------------------------------------------------------------
// Synchronization of the terrain node:
// - receive mesh vertex states and set states of proxy bodies
// - calculate current cumulative contact forces on all system bodies
// - extract and send forces at each vertex
// Note:
// Only the main terrain node participates in the co-simulation data exchange.
// -----------------------------------------------------------------------------
void ChVehicleCosimTerrainNode::Synchronize(int step_number, double time) {
    switch (m_interface_type) {
        case InterfaceType::BODY:
            if (m_wheeled)
                SynchronizeWheeledBody(step_number, time);
            else
                SynchronizeTrackedBody(step_number, time);
            break;
        case InterfaceType::MESH:
            if (m_wheeled)
                SynchronizeWheeledMesh(step_number, time);
            else
                SynchronizeTrackedMesh(step_number, time);
            break;
    }

    // Let derived classes perform optional operations
    OnSynchronize(step_number, time);
}

void ChVehicleCosimTerrainNode::SynchronizeWheeledBody(int step_number, double time) {
    for (int i = 0; i < m_num_objects; i++) {
        if (m_rank == TERRAIN_NODE_RANK) {
            // Receive rigid body state data for this tire
            MPI_Status status;
            double state_data[13];
            MPI_Recv(state_data, 13, MPI_DOUBLE, TIRE_NODE_RANK(i), step_number, MPI_COMM_WORLD, &status);

            m_rigid_state[i].pos = ChVector<>(state_data[0], state_data[1], state_data[2]);
            m_rigid_state[i].rot = ChQuaternion<>(state_data[3], state_data[4], state_data[5], state_data[6]);
            m_rigid_state[i].lin_vel = ChVector<>(state_data[7], state_data[8], state_data[9]);
            m_rigid_state[i].ang_vel = ChVector<>(state_data[10], state_data[11], state_data[12]);

            if (m_verbose)
                cout << "[Terrain node] Recv: spindle position (" << i << ") = " << m_rigid_state[i].pos << endl;
        }

        // Set position, rotation, and velocities of proxy rigid body
        UpdateRigidProxy(i, m_rigid_state[i]);

        // Collect contact force on rigid proxy and load in m_rigid_contact.
        // It is assumed that this force is given at body center.
        // Note that no force is collected at the first step.
        if (step_number > 0) {
            GetForceRigidProxy(i, m_rigid_contact[i]);
        }

        if (m_rank == TERRAIN_NODE_RANK) {
            // Send wheel contact force
            double force_data[] = {m_rigid_contact[i].force.x(),  m_rigid_contact[i].force.y(),
                                   m_rigid_contact[i].force.z(),  m_rigid_contact[i].moment.x(),
                                   m_rigid_contact[i].moment.y(), m_rigid_contact[i].moment.z()};
            MPI_Send(force_data, 6, MPI_DOUBLE, TIRE_NODE_RANK(i), step_number, MPI_COMM_WORLD);

            if (m_verbose)
                cout << "[Terrain node] Send: spindle force (" << i << ") = " << m_rigid_contact[i].force << endl;
        }
    }

    if (m_rank == TERRAIN_NODE_RANK && m_verbose) {
        cout << "[Terrain node] step number: " << step_number << "  num contacts: " << GetNumContacts() << endl;
    }
}

void ChVehicleCosimTerrainNode::SynchronizeTrackedBody(int step_number, double time) {
    std::vector<double> all_states(13 * m_num_objects);
    std::vector<double> all_forces(6 * m_num_objects);
    int start_idx;

    // Receive rigid body data for all track shoes
    if (m_rank == TERRAIN_NODE_RANK) {
        MPI_Status status;
        MPI_Recv(all_states.data(), 13 * m_num_objects, MPI_DOUBLE, MBS_NODE_RANK, step_number, MPI_COMM_WORLD, &status);

        // Unpack rigid body data
        start_idx = 0;
        for (int i = 0; i < m_num_objects; i++) {
            m_rigid_state[i].pos =
                ChVector<>(all_states[start_idx + 0], all_states[start_idx + 1], all_states[start_idx + 2]);
            m_rigid_state[i].rot = ChQuaternion<>(all_states[start_idx + 3], all_states[start_idx + 4],
                                                  all_states[start_idx + 5], all_states[start_idx + 6]);
            m_rigid_state[i].lin_vel =
                ChVector<>(all_states[start_idx + 7], all_states[start_idx + 8], all_states[start_idx + 9]);
            m_rigid_state[i].ang_vel =
                ChVector<>(all_states[start_idx + 10], all_states[start_idx + 11], all_states[start_idx + 12]);
            start_idx += 13;
        }
    }

    // Set position, rotation, and velocities of proxy rigid body.
    // Collect contact force on rigid proxy and load in m_rigid_contact.
    // It is assumed that this force is given at body center.
    // Note that no force is collected at the first step.
    for (int i = 0; i < m_num_objects; i++) {
        UpdateRigidProxy(i, m_rigid_state[i]);
        if (step_number > 0) {
            GetForceRigidProxy(i, m_rigid_contact[i]);
        }
    }

    // Send contact forces for all track shoes
    if (m_rank == TERRAIN_NODE_RANK) {
        // Pack contact forces
        start_idx = 0;
        for (int i = 0; i < m_num_objects; i++) {
            all_forces[start_idx + 0] = m_rigid_contact[i].force.x();
            all_forces[start_idx + 1] = m_rigid_contact[i].force.y();
            all_forces[start_idx + 2] = m_rigid_contact[i].force.z();
            all_forces[start_idx + 3] = m_rigid_contact[i].moment.x();
            all_forces[start_idx + 4] = m_rigid_contact[i].moment.y();
            all_forces[start_idx + 5] = m_rigid_contact[i].moment.z();
            start_idx += 6;
        }

        MPI_Send(all_forces.data(), 6 * m_num_objects, MPI_DOUBLE, MBS_NODE_RANK, step_number, MPI_COMM_WORLD);

        if (m_verbose)
            cout << "[Terrain node] step number: " << step_number << "  num contacts: " << GetNumContacts() << endl;
    }
}

void ChVehicleCosimTerrainNode::SynchronizeWheeledMesh(int step_number, double time) {
    for (int i = 0; i < m_num_objects; i++) {
        if (m_rank == TERRAIN_NODE_RANK) {
            auto nv = m_geometry[i].m_coll_meshes[0].m_trimesh->getNumVertices();

            // Receive mesh state data
            MPI_Status status;
            double* vert_data = new double[2 * 3 * nv];
            MPI_Recv(vert_data, 2 * 3 * nv, MPI_DOUBLE, TIRE_NODE_RANK(i), step_number, MPI_COMM_WORLD,
                     &status);

            for (int iv = 0; iv < nv; iv++) {
                int offset = 3 * iv;
                m_mesh_state[i].vpos[iv] =
                    ChVector<>(vert_data[offset + 0], vert_data[offset + 1], vert_data[offset + 2]);
                offset += 3 * nv;
                m_mesh_state[i].vvel[iv] =
                    ChVector<>(vert_data[offset + 0], vert_data[offset + 1], vert_data[offset + 2]);
            }

            ////if (m_verbose)
            ////    PrintMeshUpdateData(i);

            delete[] vert_data;
        }

        // Set position, rotation, and velocity of proxy bodies.
        UpdateMeshProxy(i, m_mesh_state[i]);

        // Collect contact forces on subset of mesh vertices and load in m_mesh_contact.
        // Note that no forces are collected at the first step.
        m_mesh_contact[i].vidx.clear();
        m_mesh_contact[i].vforce.clear();

        if (step_number == 0)
            m_mesh_contact[i].nv = 0;
        else
            GetForceMeshProxy(i, m_mesh_contact[i]);

        if (m_rank == TERRAIN_NODE_RANK) {
            // Send vertex indices and forces.
            MPI_Send(m_mesh_contact[i].vidx.data(), m_mesh_contact[i].nv, MPI_INT, TIRE_NODE_RANK(i), step_number,
                     MPI_COMM_WORLD);

            double* force_data = new double[3 * m_mesh_contact[i].nv];
            for (int iv = 0; iv < m_mesh_contact[i].nv; iv++) {
                force_data[3 * iv + 0] = m_mesh_contact[i].vforce[iv].x();
                force_data[3 * iv + 1] = m_mesh_contact[i].vforce[iv].y();
                force_data[3 * iv + 2] = m_mesh_contact[i].vforce[iv].z();
            }
            MPI_Send(force_data, 3 * m_mesh_contact[i].nv, MPI_DOUBLE, TIRE_NODE_RANK(i), step_number, MPI_COMM_WORLD);
            delete[] force_data;

            if (m_verbose)
                cout << "[Terrain node] step number: " << step_number << "  num contacts: " << GetNumContacts()
                     << "  vertices in contact: " << m_mesh_contact[i].nv << endl;
        }
    }
}

void ChVehicleCosimTerrainNode::SynchronizeTrackedMesh(int step_number, double time) {
    //// RADU TODO
}

// -----------------------------------------------------------------------------
// Advance simulation of the terrain node by the specified duration
// -----------------------------------------------------------------------------
void ChVehicleCosimTerrainNode::Advance(double step_size) {
    // Let derived classes advance the terrain state
    m_timer.reset();
    m_timer.start();
    OnAdvance(step_size);
    m_timer.stop();
    m_cum_sim_time += m_timer();

    // Possible rendering
    Render(step_size);
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
void ChVehicleCosimTerrainNode::PrintMeshUpdateData(int i) {
    cout << "[Terrain node] mesh vertices and faces" << endl;
    std::for_each(m_mesh_state[i].vpos.begin(), m_mesh_state[i].vpos.end(),
                  [](const ChVector<>& a) { cout << a.x() << "  " << a.y() << "  " << a.z() << endl; });
}

}  // end namespace vehicle
}  // end namespace chrono
