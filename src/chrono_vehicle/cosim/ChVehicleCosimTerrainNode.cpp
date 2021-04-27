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
// system is co-simulated with a terrain subsystem.
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

#include <mpi.h>

#include "chrono_vehicle/cosim/ChVehicleCosimTerrainNode.h"

#include "chrono_thirdparty/rapidjson/filereadstream.h"
#include "chrono_thirdparty/rapidjson/istreamwrapper.h"

using std::cout;
using std::endl;

using namespace rapidjson;

namespace chrono {
namespace vehicle {

std::string ChVehicleCosimTerrainNode::GetTypeAsString(ChVehicleCosimTerrainNode::Type type) {
    switch (type) {
        case Type::RIGID:
            return "Rigid";
        case Type::SCM:
            return "SCM";
        case Type::GRANULAR_OMP:
            return "GranularOMP";
        case Type::GRANULAR_GPU:
            return "GranularGPU";
        case Type::GRANULAR_MPI:
            return "GranularMPI";
        case Type::GRANULAR_SPH:
            return "GranularSPH";
        default:
            return "Unknown";
    }
}

ChVehicleCosimTerrainNode::Type ChVehicleCosimTerrainNode::GetTypeFromString(const std::string& type) {
    if (type == "RIGID")
        return Type::RIGID;
    if (type == "SCM")
        return Type::SCM;
    if (type == "GRANULAR_OMP")
        return Type::GRANULAR_OMP;
    if (type == "GRANULAR_GPU")
        return Type::GRANULAR_GPU;
    if (type == "GRANULAR_MPI")
        return Type::GRANULAR_MPI;
    if (type == "GRANULAR_SPH")
        return Type::GRANULAR_SPH;

    return Type::UNKNOWN;
}

bool ChVehicleCosimTerrainNode::ReadSpecfile(const std::string& specfile, Document& d) {
    std::ifstream ifs(specfile);
    if (!ifs.good()) {
        cout << "ERROR: Could not open JSON file: " << specfile << "\n" << endl;
        return false;
    }

    IStreamWrapper isw(ifs);
    d.ParseStream<ParseFlag::kParseCommentsFlag>(isw);
    if (d.IsNull()) {
        cout << "ERROR: Invalid JSON file: " << specfile << "\n" << endl;
        return false;
    }

    return true;
}

ChVehicleCosimTerrainNode::Type ChVehicleCosimTerrainNode::GetTypeFromSpecfile(const std::string& specfile) {
    Document d;
    if (!ReadSpecfile(specfile, d)) {
        return Type::UNKNOWN;
    }

    if (!d.HasMember("Type")) {
        cout << "ERROR: JSON file " << specfile << " does not specify terrain type!\n" << endl;
        return Type::UNKNOWN;
    }

    return GetTypeFromString(d["Type"].GetString());
}

// -----------------------------------------------------------------------------
// Construction of the base terrain node.
// -----------------------------------------------------------------------------
ChVehicleCosimTerrainNode::ChVehicleCosimTerrainNode(Type type, ChContactMethod method)
    : ChVehicleCosimBaseNode("TERRAIN"),
      m_type(type),
      m_method(method),
      m_render(false),
      m_render_step(0.01),
      m_init_height(0) {
    // Default patch dimensions
    m_hdimX = 1.0;
    m_hdimY = 0.25;

    // Default proxy body properties
    m_rig_mass = 50;
    m_flexible_tire = false;
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

void ChVehicleCosimTerrainNode::EnableRuntimeVisualization(bool render, double render_fps) {
    m_render = render;
    m_render_step = 1.0 / render_fps;
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

    // Create subdirectory for output from simulation
    if (!filesystem::create_directory(filesystem::path(m_node_out_dir + "/simulation"))) {
        std::cout << "Error creating directory " << m_node_out_dir + "/simulation" << std::endl;
        return;
    }

    // Reset system time
    GetSystem()->SetChTime(0);

    // ------------------------------------------
    // Send information for initial tire location
    // ------------------------------------------

    // This includes the terrain height and the container half-length.
    // Note: take into account dimension of proxy bodies
    double init_dim[2] = {m_init_height + 0.05, m_hdimX};
    MPI_Send(init_dim, 2, MPI_DOUBLE, RIG_NODE_RANK, 0, MPI_COMM_WORLD);

    if (m_verbose) {
        cout << "[Terrain node] Sent initial terrain height = " << init_dim[0] << endl;
        cout << "[Terrain node] Sent container half-length = " << init_dim[1] << endl;
    }

    // ------------------------------------------
    // Receive tire contact surface specification
    // ------------------------------------------

    MPI_Status status;

    unsigned int surf_props[4];
    MPI_Recv(surf_props, 4, MPI_UNSIGNED, RIG_NODE_RANK, 0, MPI_COMM_WORLD, &status);

    m_flexible_tire = (surf_props[0] == 1);
    m_mesh_data.nv = surf_props[1];
    m_mesh_data.nn = surf_props[2];
    m_mesh_data.nt = surf_props[3];

    if (m_flexible_tire && !SupportsFlexibleTire()) {
        cout << "ERROR: terrain system does not support flexible tires!" << endl;
        MPI_Abort(MPI_COMM_WORLD, 1);
    }

    m_mesh_data.verts.resize(m_mesh_data.nv);
    m_mesh_data.norms.resize(m_mesh_data.nn);
    m_mesh_data.idx_verts.resize(m_mesh_data.nt);
    m_mesh_data.idx_norms.resize(m_mesh_data.nt);

    m_mesh_state.vpos.resize(m_mesh_data.nv);
    m_mesh_state.vvel.resize(m_mesh_data.nv);

    // -----------------------------------------------------------
    // Receive tire mesh vertices & normals and triangle indices
    // -----------------------------------------------------------

    double* vert_data = new double[3 * m_mesh_data.nv + 3 * m_mesh_data.nn];
    int* tri_data = new int[3 * m_mesh_data.nt + 3 * m_mesh_data.nt];
    MPI_Recv(vert_data, 3 * m_mesh_data.nv + 3 * m_mesh_data.nn, MPI_DOUBLE, RIG_NODE_RANK, 0, MPI_COMM_WORLD, &status);
    MPI_Recv(tri_data, 3 * m_mesh_data.nt + 3 * m_mesh_data.nt, MPI_INT, RIG_NODE_RANK, 0, MPI_COMM_WORLD, &status);

    for (unsigned int iv = 0; iv < m_mesh_data.nv; iv++) {
        m_mesh_data.verts[iv].x() = vert_data[3 * iv + 0];
        m_mesh_data.verts[iv].y() = vert_data[3 * iv + 1];
        m_mesh_data.verts[iv].z() = vert_data[3 * iv + 2];
    }
    for (unsigned int in = 0; in < m_mesh_data.nn; in++) {
        m_mesh_data.norms[in].x() = vert_data[3 * m_mesh_data.nv + 3 * in + 0];
        m_mesh_data.norms[in].y() = vert_data[3 * m_mesh_data.nv + 3 * in + 1];
        m_mesh_data.norms[in].z() = vert_data[3 * m_mesh_data.nv + 3 * in + 2];
    }
    for (unsigned int it = 0; it < m_mesh_data.nt; it++) {
        m_mesh_data.idx_verts[it].x() = tri_data[6 * it + 0];
        m_mesh_data.idx_verts[it].y() = tri_data[6 * it + 1];
        m_mesh_data.idx_verts[it].z() = tri_data[6 * it + 2];
        m_mesh_data.idx_norms[it].x() = tri_data[6 * it + 3];
        m_mesh_data.idx_norms[it].y() = tri_data[6 * it + 4];
        m_mesh_data.idx_norms[it].z() = tri_data[6 * it + 5];
    }

    delete[] vert_data;
    delete[] tri_data;

    if (m_verbose)
        cout << "[Terrain node] Received " << surf_props[1] << " vertices and " << surf_props[3] << " triangles"
             << endl;

    // ----------------
    // Receive rig mass
    // ----------------

    MPI_Recv(&m_rig_mass, 1, MPI_DOUBLE, RIG_NODE_RANK, 0, MPI_COMM_WORLD, &status);

    if (m_verbose)
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

    if (m_verbose)
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

    if (m_verbose)
        cout << "[Terrain node] step number: " << step_number << "  num contacts: " << GetNumContacts() << endl;
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

    ////if (m_verbose)
    ////    PrintMeshUpdateData();

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

    if (m_verbose)
        cout << "[Terrain node] step number: " << step_number << "  num contacts: " << GetNumContacts()
             << "  vertices in contact: " << m_mesh_contact.nv << endl;
}

// -----------------------------------------------------------------------------
// Advance simulation of the terrain node by the specified duration
// -----------------------------------------------------------------------------
void ChVehicleCosimTerrainNode::Advance(double step_size) {
    static double render_time = 0;

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

    // Request the derived class to render simulation
    if (m_render && GetSystem()->GetChTime() > render_time) {
        OnRender(GetSystem()->GetChTime());
        render_time += std::max(m_render_step, step_size);
    }

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

    OnOutputData(frame);
}

// Print mesh vertex data, as received from the rig node at synchronization.
void ChVehicleCosimTerrainNode::PrintMeshUpdateData() {
    cout << "[Terrain node] mesh vertices and faces" << endl;
    std::for_each(m_mesh_state.vpos.begin(), m_mesh_state.vpos.end(),
                  [](const ChVector<>& a) { cout << a.x() << "  " << a.y() << "  " << a.z() << endl; });
}

}  // end namespace vehicle
}  // end namespace chrono
