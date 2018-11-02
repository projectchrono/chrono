// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2015 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Nic Olsen, Radu Serban
// =============================================================================
//
// Definition of the TERRAIN NODE.
//
// The global reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include <algorithm>
#include <cmath>
#include <set>
#include <vector>

#include "chrono/ChConfig.h"
#include "chrono/assets/ChLineShape.h"
#include "chrono/core/ChFileutils.h"
#include "chrono/geometry/ChLineBezier.h"

#include "TerrainNodeGran.h"

#define NORMAL_STIFFNESS_S2W 1e7f
#define NORMAL_STIFFNESS_S2S 1e7f
#define COHESION_RATIO 0.f

using std::cout;
using std::endl;

using namespace chrono;

const std::string TerrainNodeGran::m_checkpoint_filename = "checkpoint.dat";

utils::SamplingType sampling_type = utils::POISSON_DISK;
////utils::SamplingType sampling_type = utils::REGULAR_GRID;
////utils::SamplingType sampling_type = utils::HCP_PACK;

// -----------------------------------------------------------------------------
// Construction of the terrain node:
// - create the Chrono Granular system
// -----------------------------------------------------------------------------
TerrainNodeGran::TerrainNodeGran(int num_tires)
    : BaseNode("TERRAIN"),
      m_num_tires(num_tires),
      m_constructed(false),
      m_settling_output(false),
      m_initial_output(false),
      m_num_particles(0),
      m_particles_start_index(0),
      m_initial_height(0) {
    // ------------------------
    // Default model parameters
    // ------------------------

    // Default platform and container dimensions
    m_hdimX = 1.0;
    m_hdimY = 0.25;
    m_hdimZ = 0.5;

    // Default granular material properties
    m_radius_g = 0.01;
    m_rho_g = 2000;
    m_num_layers = 5;
    m_time_settling = 0.4;
    m_settling_output_fps = 100;

    // Default proxy body properties
    m_fixed_proxies = false;
    m_mass_pF = 1;

    // ------------------------------------
    // Create the Chrono Granular system
    // ------------------------------------

    // Create system and set default method-specific solver settings
    m_system = new chrono::granular::ChSystemGranular_MonodisperseSMC(m_radius_g, m_rho_g);
    m_system->set_gravitational_acceleration(0.f, 0.f, m_gacc.z());
    m_system->setVerbose(true);

    // Reserve space for tire information
    m_tire_data.resize(m_num_tires);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
TerrainNodeGran::~TerrainNodeGran() {
    delete m_system;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void TerrainNodeGran::SetOutDir(const std::string& dir_name, const std::string& suffix) {
    m_out_dir = dir_name;
    m_node_out_dir = dir_name + "/" + m_name + suffix;
    m_rank_out_dir = m_node_out_dir + "/results";

    if (ChFileutils::MakeDirectory(m_node_out_dir.c_str()) < 0) {
        std::cout << "Error creating directory " << m_node_out_dir << std::endl;
        MPI_Abort(MPI_COMM_WORLD, 1);
    }

    // Create results output file
    m_outf.open(m_node_out_dir + "/results.dat", std::ios::out);
    m_outf.precision(7);
    m_outf << std::scientific;

    // Create frame output directory
    if (ChFileutils::MakeDirectory(m_rank_out_dir.c_str()) < 0) {
        std::cout << "Error creating directory " << m_rank_out_dir << std::endl;
        MPI_Abort(MPI_COMM_WORLD, 1);
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void TerrainNodeGran::SetContainerDimensions(double length, double width, double height) {
    m_hdimX = length / 2;
    m_hdimY = width / 2;
    m_hdimZ = height / 2;
}

void TerrainNodeGran::SetPath(std::shared_ptr<ChBezierCurve> path) {
    m_path = path;
}

void TerrainNodeGran::SetGranularMaterial(double radius, double density, int num_layers) {
    m_radius_g = radius;
    m_rho_g = density;
    m_num_layers = num_layers;
}

void TerrainNodeGran::SetMaterialSurface(const std::shared_ptr<ChMaterialSurfaceSMC>& mat) {
    m_material_terrain = mat;
}

void TerrainNodeGran::SetProxyProperties(double mass, bool fixed) {
    m_mass_pF = mass;
    m_fixed_proxies = fixed;
}

// -----------------------------------------------------------------------------
// Complete construction of the mechanical system.
// This function is invoked automatically from Settle and Initialize.
// - adjust system settings
// - create the container body
// - if specified, create the granular material
// -----------------------------------------------------------------------------
void TerrainNodeGran::Construct() {
    if (m_constructed)
        return;

    // Inflated particle radius
    double r = 1.01 * m_radius_g;

    // Set domain
    double height = std::max((1 + m_num_layers) * 2 * r, 2 * m_hdimZ);
    m_system->setBOXdims(2 * m_hdimX, 2 * m_hdimY, height);

    // ----------------------------------------------------
    // Create the container body and the collision boundary
    // ----------------------------------------------------

    m_system->set_BD_Fixed(true);
    m_system->set_YoungModulus_SPH2WALL(NORMAL_STIFFNESS_S2W);

    // --------------------------
    // Generate granular material
    // --------------------------
    m_system->set_YoungModulus_SPH2SPH(NORMAL_STIFFNESS_S2S);
    m_system->set_Cohesion_ratio(COHESION_RATIO);
    double zlo = -1;  // TODO calculate this based on layers
    double zhi = 1;
    m_system->setFillBounds(-1.f, 1.f, -1.f, 1.f, zlo, zhi);

    m_num_particles = -1;  // TODO get number of particles created

    // Heighest particle Z coordinate
    m_initial_height = (2 * r) * m_num_layers;  // TODO this is an estimate

    cout << m_prefix << "Highest particle at:  " << m_initial_height << endl;
    cout << m_prefix << "Num. particles: " << m_num_particles << endl;

    // ------------------------------
    // Write initial body information
    // ------------------------------
    // TODO adapt and uncomment
    // if (m_initial_output) {
    //     std::ofstream outf(m_node_out_dir + "/init_particles.dat", std::ios::out);
    //     outf.precision(7);
    //     outf << std::scientific;
    //
    //     int i = -1;
    //     for (auto body : m_system->Get_bodylist()) {
    //         i++;
    //         auto status = m_system->ddm->comm_status[i];
    //         auto identifier = body->GetIdentifier();
    //         auto local_id = body->GetId();
    //         auto global_id = body->GetGid();
    //         auto pos = body->GetPos();
    //         outf << global_id << " " << local_id << " " << identifier << "   " << status << "   ";
    //         outf << pos.x() << " " << pos.y() << " " << pos.z();
    //         outf << std::endl;
    //     }
    //     outf.close();
    // }

    // --------------------------------------
    // Write file with terrain node settings
    // --------------------------------------

    // TODO adapt and uncomment
    // std::ofstream outf;
    // outf.open(m_node_out_dir + "/settings.dat", std::ios::out);
    // outf << "System settings" << endl;
    // outf << "   Integration step size = " << m_step_size << endl;
    // outf << "   Contact method = SMC" << endl;
    // outf << "   Mat. props? " << (m_system->GetSettings()->solver.use_material_properties ? "YES" : "NO") << endl;
    // outf << "   Collision envelope = " << m_system->GetSettings()->collision.collision_envelope << endl;
    // outf << "Container dimensions" << endl;
    // outf << "   X = " << 2 * m_hdimX << "  Y = " << 2 * m_hdimY << "  Z = " << 2 * m_hdimZ << endl;
    // outf << "Terrain material properties" << endl;
    // auto mat = std::static_pointer_cast<ChMaterialSurfaceSMC>(m_material_terrain);
    // outf << "   Coefficient of friction    = " << mat->GetKfriction() << endl;
    // outf << "   Coefficient of restitution = " << mat->GetRestitution() << endl;
    // outf << "   Young modulus              = " << mat->GetYoungModulus() << endl;
    // outf << "   Poisson ratio              = " << mat->GetPoissonRatio() << endl;
    // outf << "   Adhesion force             = " << mat->GetAdhesion() << endl;
    // outf << "   Kn = " << mat->GetKn() << endl;
    // outf << "   Gn = " << mat->GetGn() << endl;
    // outf << "   Kt = " << mat->GetKt() << endl;
    // outf << "   Gt = " << mat->GetGt() << endl;
    // outf << "Granular material properties" << endl;
    // outf << "   particle radius  = " << m_radius_g << endl;
    // outf << "   particle density = " << m_rho_g << endl;
    // outf << "   number layers    = " << m_num_layers << endl;
    // outf << "   number particles = " << m_num_particles << endl;
    // outf << "Settling" << endl;
    // outf << "   settling time = " << m_time_settling << endl;
    // outf << "   output? " << (m_settling_output ? "YES" : "NO") << endl;
    // outf << "   output frequency (FPS) = " << m_settling_output_fps << endl;
    // outf << "Proxy body properties" << endl;
    // outf << "   proxies fixed? " << (m_fixed_proxies ? "YES" : "NO") << endl;
    // outf << "   proxy mass = " << m_mass_pF << endl;

    // Mark system as constructed.
    m_constructed = true;
}

// -----------------------------------------------------------------------------
// Settling phase for the terrain node
// - if not already done, complete system construction
// - record height of terrain
// -----------------------------------------------------------------------------
void TerrainNodeGran::Settle() {
    Construct();

    // -------------------------------------
    // Simulate settling of granular terrain
    // -------------------------------------

    // Return now if time_settling = 0
    if (m_time_settling == 0)
        return;

    cout << m_prefix << " start settling" << endl;
    m_system->run_simulation(m_time_settling);

    // TODO adapt find height of granular material
    m_initial_height = 0.2;

    cout << m_prefix << " settling time = " << m_cum_sim_time << endl;
    m_cum_sim_time = 0;
}

// -----------------------------------------------------------------------------
// Initialization of the terrain node:
// - if not already done, complete system construction
// - send information on terrain height
// - receive information on tire mesh topology (number vertices and triangles)
// - receive tire contact material properties and create the "tire" material
// - create the appropriate proxy bodies (state not set yet)
// -----------------------------------------------------------------------------
void TerrainNodeGran::Initialize() {
    Construct();

    // ---------------------------------------------
    // Send information for initial vehicle position
    // ---------------------------------------------

    double init_height = m_initial_height + m_radius_g;
    double init_dim[2] = {init_height, m_hdimX};
    MPI_Send(init_dim, 2, MPI_DOUBLE, VEHICLE_NODE_RANK, 0, MPI_COMM_WORLD);

    cout << m_prefix << " Sent initial terrain height = " << init_dim[0] << endl;
    cout << m_prefix << " Sent container half-length = " << init_dim[1] << endl;

    // --------------------------------------------------------
    // Loop over all tires, receive information, create proxies
    // --------------------------------------------------------

    unsigned int start_tri_index = 0;

    for (int which = 0; which < m_num_tires; which++) {
        // Receive tire contact surface specification.
        unsigned int surf_props[2];

        MPI_Status status_p;
        MPI_Recv(surf_props, 2, MPI_UNSIGNED, TIRE_NODE_RANK(which), 0, MPI_COMM_WORLD, &status_p);
        cout << m_prefix << " Received vertices = " << surf_props[0] << " triangles = " << surf_props[1] << endl;

        m_tire_data[which].m_num_vert = surf_props[0];
        m_tire_data[which].m_num_tri = surf_props[1];

        m_tire_data[which].m_vertex_pos.resize(surf_props[0]);
        m_tire_data[which].m_vertex_vel.resize(surf_props[0]);
        m_tire_data[which].m_triangles.resize(surf_props[1]);
        m_tire_data[which].m_gids.resize(surf_props[1]);

        m_tire_data[which].m_start_tri = start_tri_index;
        start_tri_index += surf_props[1];

        // Receive tire mesh connectivity.
        unsigned int num_tri = m_tire_data[which].m_num_tri;
        int* tri_data = new int[3 * num_tri];

        MPI_Status status_c;
        MPI_Recv(tri_data, 3 * num_tri, MPI_INT, TIRE_NODE_RANK(which), 0, MPI_COMM_WORLD, &status_c);

        for (unsigned int it = 0; it < num_tri; it++) {
            m_tire_data[which].m_triangles[it][0] = tri_data[3 * it + 0];
            m_tire_data[which].m_triangles[it][1] = tri_data[3 * it + 1];
            m_tire_data[which].m_triangles[it][2] = tri_data[3 * it + 2];
        }

        delete[] tri_data;

        // Receive vertex locations.
        unsigned int num_vert = m_tire_data[which].m_num_vert;
        double* vert_data = new double[3 * num_vert];

        MPI_Status status_v;
        MPI_Recv(vert_data, 3 * num_vert, MPI_DOUBLE, TIRE_NODE_RANK(which), 0, MPI_COMM_WORLD, &status_v);

        for (unsigned int iv = 0; iv < num_vert; iv++) {
            m_tire_data[which].m_vertex_pos[iv] =
                ChVector<>(vert_data[3 * iv + 0], vert_data[3 * iv + 1], vert_data[3 * iv + 2]);
        }

        delete[] vert_data;

        // Receive tire contact material properties.
        float mat_props[8];

        MPI_Status status_m;
        MPI_Recv(mat_props, 8, MPI_FLOAT, TIRE_NODE_RANK(which), 0, MPI_COMM_WORLD, &status_m);
        cout << m_prefix << " received tire material:  friction = " << mat_props[0] << endl;

        auto mat_tire = std::make_shared<ChMaterialSurfaceSMC>();
        mat_tire->SetFriction(mat_props[0]);
        mat_tire->SetRestitution(mat_props[1]);
        mat_tire->SetYoungModulus(mat_props[2]);
        mat_tire->SetPoissonRatio(mat_props[3]);
        mat_tire->SetKn(mat_props[4]);
        mat_tire->SetGn(mat_props[5]);
        mat_tire->SetKt(mat_props[6]);
        mat_tire->SetGt(mat_props[7]);

        // Create proxy bodies. Represent the tire as triangles associated with mesh faces.
        CreateFaceProxies(which, mat_tire);
    }

    // ------------------------------
    // Write initial body information
    // ------------------------------

    // TODO adapt and uncomment
    // if (m_initial_output) {
    //     std::ofstream outf(m_node_out_dir + "/init_bodies.dat", std::ios::out);
    //     outf.precision(7);
    //     outf << std::scientific;
    //
    //     int i = -1;
    //     for (auto body : m_system->Get_bodylist()) {
    //         i++;
    //         auto status = m_system->ddm->comm_status[i];
    //         auto identifier = body->GetIdentifier();
    //         auto local_id = body->GetId();
    //         auto global_id = body->GetGid();
    //         auto pos = body->GetPos();
    //         outf << global_id << " " << local_id << " " << identifier << "   " << status << "   ";
    //         outf << pos.x() << " " << pos.y() << " " << pos.z();
    //         outf << std::endl;
    //     }
    //     outf.close();
    // }
}

// TODO adapt
// Create bodies with triangular contact geometry as proxies for the tire mesh faces.
// Assign to each body an identifier equal to the index of its corresponding mesh face.
// Maintain a list of all bodies associated with the tire.
// Add all proxy bodies to the same collision family and disable collision between any
// two members of this family.
void TerrainNodeGran::CreateFaceProxies(int which, std::shared_ptr<ChMaterialSurfaceSMC> material) {
    TireData& tire_data = m_tire_data[which];

    //// TODO:  better approximation of mass / inertia?
    ChVector<> inertia_pF = 1e-3 * m_mass_pF * ChVector<>(0.1, 0.1, 0.1);

    // TODO adapt and uncomment: add each triangle
    //     for (unsigned int it = 0; it < tire_data.m_num_tri; it++) {
    //         auto body = std::shared_ptr<ChBody>(m_system->NewBody());
    //         body->SetIdentifier(tire_data.m_start_tri + it);
    //         body->SetMass(m_mass_pF);
    //         body->SetInertiaXX(inertia_pF);
    //         body->SetBodyFixed(m_fixed_proxies);
    //         body->SetMaterialSurface(material);  // TODO adapt
    //
    //         // Determine initial position and contact shape
    //         const ChVector<int>& tri = tire_data.m_triangles[it];
    //         const ChVector<>& pA = tire_data.m_vertex_pos[tri[0]];
    //         const ChVector<>& pB = tire_data.m_vertex_pos[tri[1]];
    //         const ChVector<>& pC = tire_data.m_vertex_pos[tri[2]];
    //         ChVector<> pos = (pA + pB + pC) / 3;
    //         body->SetPos(pos);
    //
    //         // TODO adding triangles
    //         // TODO family to prevent contact with other triangles in the tire
    //         // Note that the vertex locations will be updated at every synchronization time.
    //         utils::AddTriangle(body.get(), pA - pos, pB - pos, pC - pos, name);
    //         m_system->AddBody(body);
    //
    //         // Update map global ID -> triangle index
    //         tire_data.m_gids[it] = body->GetGid();  // TODO get identifier
    //         tire_data.m_map[body->GetGid()] = it;   // TODO get identifier
    //     }
}

// -----------------------------------------------------------------------------
// Synchronization of the terrain node:
// - receive tire mesh vertex states and set states of proxy bodies
// - calculate current cumulative contact forces on all system bodies
// - extract and send forces at each vertex
// -----------------------------------------------------------------------------
void TerrainNodeGran::Synchronize(int step_number, double time) {
    // --------------------------------------------------------------
    // Loop over all tires, receive mesh vertex state, update proxies
    // --------------------------------------------------------------
    for (int which = 0; which < m_num_tires; which++) {
        // Receive tire mesh vertex locations and velocities.
        MPI_Status status;
        unsigned int num_vert = m_tire_data[which].m_num_vert;
        double* vert_data = new double[2 * 3 * num_vert];

        MPI_Recv(vert_data, 2 * 3 * num_vert, MPI_DOUBLE, TIRE_NODE_RANK(which), step_number, MPI_COMM_WORLD, &status);

        for (unsigned int iv = 0; iv < num_vert; iv++) {
            unsigned int offset = 3 * iv;
            m_tire_data[which].m_vertex_pos[iv] =
                ChVector<>(vert_data[offset + 0], vert_data[offset + 1], vert_data[offset + 2]);
            offset += 3 * num_vert;
            m_tire_data[which].m_vertex_vel[iv] =
                ChVector<>(vert_data[offset + 0], vert_data[offset + 1], vert_data[offset + 2]);
        }

        delete[] vert_data;

        // Set position, rotation, and velocity of proxy bodies.
        UpdateFaceProxies(which);
    }

    // ------------------------------------------------------------
    // Calculate cumulative contact forces for all bodies in system
    // ------------------------------------------------------------

    std::string msg = " step number: " + std::to_string(step_number);

    // -----------------------------------------------------------------
    // Loop over all tires, calculate vertex contact forces, send forces
    // -----------------------------------------------------------------

    msg += "  [  ";

    for (int which = 0; which < m_num_tires; which++) {
        // Collect contact forces on subset of mesh vertices.
        // Note that no forces are collected at the first step.
        std::vector<double> vert_forces;
        std::vector<int> vert_indices;

        if (step_number > 0) {
            // Gathers forces
            ForcesFaceProxies(which, vert_forces, vert_indices);
        }

        // Send vertex indices and forces.
        int num_vert = (int)vert_indices.size();
        MPI_Send(vert_indices.data(), num_vert, MPI_INT, TIRE_NODE_RANK(which), step_number, MPI_COMM_WORLD);
        MPI_Send(vert_forces.data(), 3 * num_vert, MPI_DOUBLE, TIRE_NODE_RANK(which), step_number, MPI_COMM_WORLD);

        msg += std::to_string(num_vert) + "  ";
    }

    msg += "]";

    if (m_verbose)
        cout << m_prefix << msg << endl;
}

// Set position, orientation, and velocity of proxy bodies based on tire mesh faces.
// The proxy body is effectively reconstructed at each synchronization time:
//    - position at the center of mass of the three vertices
//    - orientation: identity
//    - linear and angular velocity: consistent with vertex velocities
//    - contact shape: redefined to match vertex locations
void TerrainNodeGran::UpdateFaceProxies(int which) {
    // TODO adapt and uncomment: decide how we represent a triangle and how to update it
    // Traverse the information for the current tire and collect updated information.
    // const TireData& tire_data = m_tire_data[which];
    // std::vector<ChSystemDistributed::BodyState> states(tire_data.m_num_tri);
    // std::vector<ChSystemDistributed::TriData> shapes(tire_data.m_num_tri);
    // std::vector<int> shape_idx(tire_data.m_num_tri, 0);
    //
    // for (unsigned int it = 0; it < tire_data.m_num_tri; it++) {
    //     const ChVector<int>& tri = tire_data.m_triangles[it];
    //
    //     // Vertex locations and velocities (expressed in global frame)
    //     const ChVector<>& pA = tire_data.m_vertex_pos[tri[0]];
    //     const ChVector<>& pB = tire_data.m_vertex_pos[tri[1]];
    //     const ChVector<>& pC = tire_data.m_vertex_pos[tri[2]];
    //
    //     const ChVector<>& vA = tire_data.m_vertex_vel[tri[0]];
    //     const ChVector<>& vB = tire_data.m_vertex_vel[tri[1]];
    //     const ChVector<>& vC = tire_data.m_vertex_vel[tri[2]];
    //
    //     // Position and orientation of proxy body (at triangle barycenter)
    //     ChVector<> pos = (pA + pB + pC) / 3;
    //     states[it].pos = pos;
    //     states[it].rot = QUNIT;
    //
    //     // Linear velocity (absolute) and angular velocity (local)
    //     // These are the solution of an over-determined 9x6 linear system. However, for a centroidal
    //     // body reference frame, the linear velocity is the average of the 3 vertex velocities.
    //     // This leaves a 9x3 linear system for the angular velocity which should be solved in a
    //     // least-square sense:   Ax = b   =>  (A'A)x = A'b
    //     states[it].pos_dt = (vA + vB + vC) / 3;
    //     states[it].rot_dt = ChQuaternion<>(0, 0, 0, 0);  //// TODO: angular velocity
    //
    //     // Triangle contact shape (expressed in local frame).
    //     shapes[it].v1 = pA - pos;
    //     shapes[it].v2 = pB - pos;
    //     shapes[it].v3 = pC - pos;
    // }
    //
    // // TODO adapt
    // // Update body states
    // // Update collision shapes (one triangle per collision model)
    // m_system->SetBodyStates(tire_data.m_gids, states);
    // m_system->SetTriangleShapes(tire_data.m_gids, shape_idx, shapes);
}

// Calculate barycentric coordinates (a1, a2, a3) for a given point P
// with respect to the triangle with vertices {v1, v2, v3}
ChVector<> TerrainNodeGran::CalcBarycentricCoords(const ChVector<>& v1,
                                                  const ChVector<>& v2,
                                                  const ChVector<>& v3,
                                                  const ChVector<>& vP) {
    ChVector<> v12 = v2 - v1;
    ChVector<> v13 = v3 - v1;
    ChVector<> v1P = vP - v1;

    double d_12_12 = Vdot(v12, v12);
    double d_12_13 = Vdot(v12, v13);
    double d_13_13 = Vdot(v13, v13);
    double d_1P_12 = Vdot(v1P, v12);
    double d_1P_13 = Vdot(v1P, v13);

    double denom = d_12_12 * d_13_13 - d_12_13 * d_12_13;

    double a2 = (d_13_13 * d_1P_12 - d_12_13 * d_1P_13) / denom;
    double a3 = (d_12_12 * d_1P_13 - d_12_13 * d_1P_12) / denom;
    double a1 = 1 - a2 - a3;

    return ChVector<>(a1, a2, a3);
}

// Collect contact forces on the (face) proxy bodies that are in contact.
// Load mesh vertex forces and corresponding indices.
void TerrainNodeGran::ForcesFaceProxies(int which, std::vector<double>& vert_forces, std::vector<int>& vert_indices) {
    // Gather contact forces on proxy bodies on the terrain master rank.
    // TODO adapt and uncomment: how do we get contact forces on triangles from identifiers
    // auto force_pairs = m_system->GetBodyContactForces(m_tire_data[which].m_gids);
    //
    // // Maintain an unordered map of vertex indices and associated contact forces.
    // std::unordered_map<int, ChVector<>> my_map;
    //
    // // Loop over all triangles that experienced contact and accumulate forces on adjacent vertices.
    // for (auto& force_pair : force_pairs) {
    //     auto gid = force_pair.first;                             // global ID of the proxy body
    //     auto it = m_tire_data[which].m_map[gid];                 // index of corresponding triangle
    //     ChVector<int> tri = m_tire_data[which].m_triangles[it];  // adjacent vertex indices
    //
    //     // Centroid has barycentric coordinates {1/3, 1/3, 1/3}, so force is
    //     // distributed equally to the three vertices.
    //     ChVector<> force = force_pair.second / 3;
    //
    //     // For each vertex of the triangle, if it appears in the map, increment
    //     // the total contact force. Otherwise, insert a new entry in the map.
    //     auto v1 = my_map.find(tri[0]);
    //     if (v1 != my_map.end()) {
    //         v1->second += force;
    //     } else {
    //         my_map[tri[0]] = force;
    //     }
    //
    //     auto v2 = my_map.find(tri[1]);
    //     if (v2 != my_map.end()) {
    //         v2->second += force;
    //     } else {
    //         my_map[tri[1]] = force;
    //     }
    //
    //     auto v3 = my_map.find(tri[2]);
    //     if (v3 != my_map.end()) {
    //         v3->second += force;
    //     } else {
    //         my_map[tri[2]] = force;
    //     }
    // }
    //
    // // Extract map keys (indices of vertices in contact) and map values
    // // (corresponding contact forces) and load output vectors.
    // // Note: could improve efficiency by reserving space for vectors.
    // for (auto kv : my_map) {
    //     vert_indices.push_back(kv.first);
    //     vert_forces.push_back(kv.second.x());
    //     vert_forces.push_back(kv.second.y());
    //     vert_forces.push_back(kv.second.z());
    // }
}

// -----------------------------------------------------------------------------
// Advance simulation of the terrain node by the specified duration
// -----------------------------------------------------------------------------
void TerrainNodeGran::Advance(double step_size) {
    m_timer.reset();
    m_timer.start();
    // TODO adapt and uncomment: advance simulation by step_size
    // m_system->advance_fxn_that_doesnt_exist(step_size);
    m_timer.stop();
    m_cum_sim_time += m_timer();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void TerrainNodeGran::OutputData(int frame) {
    // Append to results output file
    if (m_outf.is_open()) {
        std::string del("  ");

        // TODO adapt and uncomment: how to get current time
        // m_outf << m_system->GetChTime() << del << GetSimTime() << del << GetTotalSimTime() << del;
        // m_outf << endl;
    }

    // Create and write frame output file.
    char filename[100];
    sprintf(filename, "%s/data_%04d.dat", m_rank_out_dir.c_str(), frame + 1);

    utils::CSV_writer csv(" ");
    WriteParticleInformation(csv);
    csv.write_to_file(filename);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void TerrainNodeGran::WriteParticleInformation(utils::CSV_writer& csv) {
    // Write current time, number of granular particles and their radius
    // TODO adapt and uncomment: how to get current time
    // csv << m_system->GetChTime() << endl;
    csv << m_num_particles << m_radius_g << endl;

    // Write particle positions and linear velocities.
    // Skip bodies that are not owned by this rank or are not terrain particles
    // TODO adapt and uncomment: How to output info
    // int i = -1;
    // for (auto body : *m_system->data_manager->body_list) {
    //     i++;
    //     if (body->GetIdentifier() < m_Id_g)
    //         continue;
    //     csv << body->GetGid() << body->GetPos() << body->GetPos_dt() << endl;
    // }
}