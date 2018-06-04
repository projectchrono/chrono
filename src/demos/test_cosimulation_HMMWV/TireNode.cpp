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
// Authors: Radu Serban, Antonio Recuero
// =============================================================================
//
// Definition of a TIRE NODE.
//
// The global reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include <omp.h>
#include <algorithm>
#include <set>
#include <vector>

#include "chrono/ChConfig.h"
#include "chrono/physics/ChLinkLock.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/timestepper/ChState.h"
#include "chrono/utils/ChUtilsCreators.h"

#include "chrono_fea/ChLoadContactSurfaceMesh.h"
#include "chrono_fea/ChElementShellANCF.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ANCFTire.h"

#include "chrono_thirdparty/rapidjson/document.h"
#include "chrono_thirdparty/rapidjson/filereadstream.h"

#ifdef CHRONO_MKL
#include "chrono_mkl/ChSolverMKL.h"
#endif

#include "TireNode.h"

using std::cout;
using std::endl;

using namespace chrono;
using namespace chrono::vehicle;

using namespace rapidjson;

// =============================================================================

// -----------------------------------------------------------------------------
// Construction of the tire node:
// - create the (sequential) Chrono system and set solver parameters
// -----------------------------------------------------------------------------
TireNode::TireNode(const std::string& json_filename, WheelID wheel_id, int num_threads)
    : BaseNode(""),
      m_tire_json(json_filename),
      m_wheel_id(wheel_id),
      m_tire_wrapper(nullptr),
      m_verbose_forces(false),
      m_verbose_states(false),
      m_verbose_solver(false) {
    m_name = "TIRE_" + std::to_string(m_wheel_id.id());
    m_prefix = "[Tire node " + std::to_string(m_wheel_id.id()) + " ]";

    // -------------------------------------
    // Peek in JSON file and infer tire type
    // -------------------------------------

    FILE* fp = fopen(json_filename.c_str(), "r");

    char readBuffer[65536];
    FileReadStream is(fp, readBuffer, sizeof(readBuffer));

    fclose(fp);

    Document d;
    d.ParseStream<ParseFlag::kParseCommentsFlag>(is);

    assert(d.HasMember("Type"));
    assert(d.HasMember("Template"));
    std::string template_type = d["Type"].GetString();
    std::string template_subtype = d["Template"].GetString();
    assert(template_type.compare("Tire") == 0);

    if (template_subtype.compare("ANCFTire") == 0) {
        m_type = ANCF;
    } else if (template_subtype.compare("FEATire") == 0) {
        m_type = FEA;
    } else if (template_subtype.compare("RigidTire") == 0) {
        m_type = RIGID;
    }

    cout << m_prefix << " axle = " << m_wheel_id.axle() << " side = " << m_wheel_id.side() << " tire type = " << m_type
         << " num_threads = " << num_threads << endl;

    // ------------------------
    // Default model parameters
    // ------------------------

    m_rim_mass = 15;
    m_rim_inertia = ChVector<>(1, 1, 1);
    m_rim_fixed = false;

    m_tire_pressure = true;

    // ----------------------------------
    // Create the (sequential) SMC system
    // ----------------------------------

    m_system = new ChSystemSMC;
    m_system->Set_G_acc(m_gacc);

    // Set number threads
    m_system->SetParallelThreadNumber(num_threads);
    CHOMPfunctions::SetNumThreads(num_threads);

#ifdef CHRONO_MKL
    // Solver settings
    auto mkl_solver = std::make_shared<ChSolverMKL<>>();
    mkl_solver->SetSparsityPatternLock(true);
    m_system->SetSolver(mkl_solver);
#else
    // Solver settings
    m_system->SetMaxItersSolverSpeed(100);
    m_system->SetMaxItersSolverStab(100);
    m_system->SetSolverType(ChSolver::Type::SOR);
    m_system->SetTol(1e-10);
    m_system->SetTolForce(1e-8);
#endif

    // Integrator settings
    m_system->SetTimestepperType(ChTimestepper::Type::HHT);
    m_integrator = std::static_pointer_cast<ChTimestepperHHT>(m_system->GetTimestepper());
    m_integrator->SetAlpha(-0.2);
    m_integrator->SetMaxiters(50);
    m_integrator->SetAbsTolerances(5e-05, 1.8e00);
    m_integrator->SetMode(ChTimestepperHHT::POSITION);
    m_integrator->SetScaling(true);
    m_integrator->SetVerbose(m_verbose_solver);
    m_integrator->SetMaxItersSuccess(5);
}

// -----------------------------------------------------------------------------
// Destructor - free memory for underlying system and tire wrapper object
// -----------------------------------------------------------------------------
TireNode::~TireNode() {
    delete m_system;
    delete m_tire_wrapper;
}

// -----------------------------------------------------------------------------
// Set the inertia properties for the rim body proxy
// -----------------------------------------------------------------------------
void TireNode::SetProxyProperties(double mass, const ChVector<>& inertia, bool fixed) {
    m_rim_mass = mass;
    m_rim_inertia = inertia;
    m_rim_fixed = fixed;
}

// -----------------------------------------------------------------------------
// Initialization of the tire node:
// - receive (from vehicle node) the initial wheel state
// - create and initialize the rim body
// - create and initialize the tire
// - extract contact surface
// - send to terrain node information on tire mesh topology (number verices and triangles)
// - send to terrain node information on tire contact material
// -----------------------------------------------------------------------------
void TireNode::Initialize() {
    // ---------------------------
    // Receive initial wheel state
    // ---------------------------

    double init_state[7];
    MPI_Status status;
    MPI_Recv(init_state, 7, MPI_DOUBLE, VEHICLE_NODE_RANK, m_wheel_id.id(), MPI_COMM_WORLD, &status);

    cout << m_prefix << " Init. loc. = " << init_state[0] << " " << init_state[1] << " " << init_state[2] << endl;
    cout << m_prefix << " Init. rot. = " << init_state[3] << " " << init_state[4] << " " << init_state[5] << " "
         << init_state[6] << endl;

    ChVector<> loc(init_state[0], init_state[1], init_state[2]);
    ChQuaternion<> rot(init_state[3], init_state[4], init_state[5], init_state[6]);

    // ----------------------------------
    // Create and initialize rim and tire
    // ----------------------------------

    // Create the rim body
    m_rim = std::shared_ptr<ChBody>(m_system->NewBody());
    m_rim->SetMass(m_rim_mass);
    m_rim->SetInertiaXX(m_rim_inertia);
    m_rim->SetBodyFixed(m_rim_fixed);
    m_system->AddBody(m_rim);

    // Initialize rim body (always zero linear and angular velocities)
    m_rim->SetPos(loc);
    m_rim->SetRot(rot);
    m_rim->SetPos_dt(ChVector<>(0, 0, 0));
    m_rim->SetWvel_loc(ChVector<>(0, 0, 0));

    // Create the tire wrapper
    switch (m_type) {
        case ANCF:
            m_tire_wrapper = new TireANCF(m_tire_json, m_tire_pressure);
            break;
        ////case FEA:
        ////    m_tire_wrapper = new TireFEA(m_tire_json, m_tire_pressure);
        ////    break;
        case RIGID:
            m_tire_wrapper = new TireRigid(m_tire_json);
            break;
    }

    // Initialize the tire and obtain contact surface properties.
    std::array<int, 2> surf_props;
    std::array<float, 8> mat_props;
    m_tire_wrapper->Initialize(m_rim, m_wheel_id.side(), surf_props, mat_props);

    // Mark completion of system construction
    m_system->SetupInitial();

    // -------------------------------------------------
    // Send tire contact surface and material properties
    // -------------------------------------------------

    // Number of vertices and triangles
    MPI_Send(surf_props.data(), 2, MPI_UNSIGNED, TERRAIN_NODE_RANK, 0, MPI_COMM_WORLD);

    cout << m_prefix << " vertices = " << surf_props[0] << "  triangles = " << surf_props[1] << endl;

    // Mesh connectivity
    std::vector<ChVector<>> vert_pos;
    std::vector<ChVector<>> vert_vel;  // ignored here
    std::vector<ChVector<int>> triangles;
    m_tire_wrapper->GetMeshState(vert_pos, vert_vel, triangles);

    unsigned int num_tri = (unsigned int)triangles.size();
    int* tri_data = new int[3 * num_tri];
    for (unsigned int it = 0; it < num_tri; it++) {
        tri_data[3 * it + 0] = triangles[it].x();
        tri_data[3 * it + 1] = triangles[it].y();
        tri_data[3 * it + 2] = triangles[it].z();
    }
    MPI_Send(tri_data, 3 * num_tri, MPI_INT, TERRAIN_NODE_RANK, 0, MPI_COMM_WORLD);
    delete[] tri_data;

    // Vertex locations
    unsigned int num_vert = (unsigned int)vert_pos.size();
    double* vert_data = new double[3 * num_vert];
    for (unsigned int iv = 0; iv < num_vert; iv++) {
        vert_data[3 * iv + 0] = vert_pos[iv].x();
        vert_data[3 * iv + 1] = vert_pos[iv].y();
        vert_data[3 * iv + 2] = vert_pos[iv].z();
    }
    MPI_Send(vert_data, 3 * num_vert, MPI_DOUBLE, TERRAIN_NODE_RANK, 0, MPI_COMM_WORLD);
    delete[] vert_data;

    // Material properties
    MPI_Send(mat_props.data(), 8, MPI_FLOAT, TERRAIN_NODE_RANK, 0, MPI_COMM_WORLD);

    cout << m_prefix << " friction = " << mat_props[0] << endl;

    // ----------------------------------
    // Write file with tire node settings
    // ----------------------------------

    std::ofstream outf;
    outf.open(m_node_out_dir + "/settings.dat", std::ios::out);

    outf << "System settings" << endl;
    outf << "   Integration step size = " << m_step_size << endl;
    outf << "Tire specification" << endl;
    outf << "   JSON file: " << m_tire_json << endl;
    outf << "   Tire type: " << m_type << endl;
    outf << "   Pressure enabled? " << (m_tire_pressure ? "YES" : "NO") << endl;
    outf << "Rim body" << endl;
    outf << "   Mass = " << m_rim_mass << endl;
}

// -----------------------------------------------------------------------------
// Constructors for the tire wrappers
// -----------------------------------------------------------------------------

TireANCF::TireANCF(const std::string& json, bool enable_pressure) {
    m_tire = std::make_shared<ANCFTire>(json);
    m_tire->EnablePressure(enable_pressure);
    m_tire->EnableContact(true);
    m_tire->EnableRimConnection(true);
    m_tire->SetContactSurfaceType(ChDeformableTire::TRIANGLE_MESH);
}

TireRigid::TireRigid(const std::string& json) {
    m_tire = std::make_shared<RigidTire>(json);
    assert(m_tire->UseContactMesh());
}

// -----------------------------------------------------------------------------
// Initialize underlying tire and return surface contact properties
// -----------------------------------------------------------------------------

void TireANCF::Initialize(std::shared_ptr<ChBody> rim,
                          VehicleSide side,
                          std::array<int, 2>& surf_props,
                          std::array<float, 8>& mat_props) {
    // Initialize underlying tire
    m_tire->Initialize(rim, side);

    // Create a mesh load for contact forces and add it to the tire's load container
    auto contact_surface = std::static_pointer_cast<fea::ChContactSurfaceMesh>(m_tire->GetContactSurface());
    m_contact_load = std::make_shared<fea::ChLoadContactSurfaceMesh>(contact_surface);
    m_tire->GetLoadContainer()->Add(m_contact_load);

    // Preprocess the tire mesh and store neighbor element information for each vertex
    // and vertex indices for each element. This data is used in output.
    auto mesh = m_tire->GetMesh();
    m_adjElements.resize(mesh->GetNnodes());
    m_adjVertices.resize(mesh->GetNelements());

    int nodeOrder[] = {0, 1, 2, 3};
    for (unsigned int ie = 0; ie < mesh->GetNelements(); ie++) {
        auto element = mesh->GetElement(ie);
        for (int in = 0; in < 4; in++) {
            auto node = element->GetNodeN(nodeOrder[in]);
            auto node_itr = std::find(mesh->GetNodes().begin(), mesh->GetNodes().end(), node);
            auto iv = std::distance(mesh->GetNodes().begin(), node_itr);
            m_adjElements[iv].push_back(ie);
            m_adjVertices[ie].push_back(iv);
        }
    }

    // Extract number of vertices and faces from tire mesh
    surf_props[0] = contact_surface->GetNumVertices();
    surf_props[1] = contact_surface->GetNumTriangles();

    // Extract tire contact properties
    mat_props[0] = m_tire->GetCoefficientFriction();
    mat_props[1] = m_tire->GetCoefficientRestitution();
    mat_props[2] = m_tire->GetYoungModulus();
    mat_props[3] = m_tire->GetPoissonRatio();
    mat_props[4] = m_tire->GetKn();
    mat_props[5] = m_tire->GetGn();
    mat_props[6] = m_tire->GetKt();
    mat_props[7] = m_tire->GetGt();
}

void TireRigid::Initialize(std::shared_ptr<ChBody> rim,
                           VehicleSide side,
                           std::array<int, 2>& surf_props,
                           std::array<float, 8>& mat_props) {
    // Initialize underlying tire
    m_tire->Initialize(rim, side);

    // Preprocess the tire mesh and store neighbor element information for each vertex.
    // Calculate mesh triangle areas.
    m_adjElements.resize(m_tire->GetNumVertices());
    std::vector<double> triArea(m_tire->GetNumTriangles());
    const std::vector<ChVector<>>& vertices = m_tire->GetMeshVertices();
    const std::vector<ChVector<int>>& triangles = m_tire->GetMeshConnectivity();
    for (unsigned int ie = 0; ie < m_tire->GetNumTriangles(); ie++) {
        int iv1 = triangles[ie].x();
        int iv2 = triangles[ie].y();
        int iv3 = triangles[ie].z();
        ChVector<> v1 = vertices[iv1];
        ChVector<> v2 = vertices[iv2];
        ChVector<> v3 = vertices[iv3];
        triArea[ie] = 0.5 * Vcross(v2 - v1, v3 - v1).Length();
        m_adjElements[iv1].push_back(ie);
        m_adjElements[iv2].push_back(ie);
        m_adjElements[iv3].push_back(ie);
    }

    // Preprocess the tire mesh and store representative area for each vertex.
    m_vertexArea.resize(m_tire->GetNumVertices());
    for (unsigned int in = 0; in < m_tire->GetNumVertices(); in++) {
        double area = 0;
        for (unsigned int ie = 0; ie < m_adjElements[in].size(); ie++) {
            area += triArea[m_adjElements[in][ie]];
        }
        m_vertexArea[in] = area / m_adjElements[in].size();
    }

    // Extract number of vertices and faces from tire mesh
    surf_props[0] = m_tire->GetNumVertices();
    surf_props[1] = m_tire->GetNumTriangles();

    // Extract tire contact properties
    auto contact_mat = rim->GetMaterialSurfaceSMC();
    mat_props[0] = contact_mat->GetSfriction();
    mat_props[1] = contact_mat->GetRestitution();
    mat_props[2] = contact_mat->GetYoungModulus();
    mat_props[3] = contact_mat->GetPoissonRatio();
    mat_props[4] = contact_mat->GetKn();
    mat_props[5] = contact_mat->GetGn();
    mat_props[6] = contact_mat->GetKt();
    mat_props[7] = contact_mat->GetGt();
}

// -----------------------------------------------------------------------------
// Synchronization of the tire node:
// - extract and send (to terrain node) tire mesh vertex states
// - receive (from terrain node) and apply vertex contact forces
// - accumulate and send (to vehicle node) tire-rim connection forces
// - receive (from vehicle node) updated wheel state
// -----------------------------------------------------------------------------
void TireNode::Synchronize(int step_number, double time) {
    // -------------------------------
    // Communication with VEHICLE node
    // -------------------------------

    // Get tire force as applied to the rim
    TerrainForce tire_force;
    m_tire_wrapper->GetTireForce(tire_force);

    // Send tire force to the vehicle node
    double bufTF[9];
    bufTF[0] = tire_force.force.x();
    bufTF[1] = tire_force.force.y();
    bufTF[2] = tire_force.force.z();
    bufTF[3] = tire_force.moment.x();
    bufTF[4] = tire_force.moment.y();
    bufTF[5] = tire_force.moment.z();
    bufTF[6] = tire_force.point.x();
    bufTF[7] = tire_force.point.y();
    bufTF[8] = tire_force.point.z();
    MPI_Send(bufTF, 9, MPI_DOUBLE, VEHICLE_NODE_RANK, m_wheel_id.id(), MPI_COMM_WORLD);

    if (m_verbose_forces) {
        cout << m_prefix << " sent tire forces: " << bufTF[0] << " " << bufTF[1] << " " << bufTF[2] << "  ,  ";
        cout << bufTF[3] << " " << bufTF[4] << " " << bufTF[5] << "  ,  ";
        cout << bufTF[6] << " " << bufTF[7] << " " << bufTF[8] << endl;
    }

    // Receive wheel state from the vehicle node
    double bufWS[14];
    MPI_Status statusWS;
    MPI_Recv(bufWS, 14, MPI_DOUBLE, VEHICLE_NODE_RANK, m_wheel_id.id(), MPI_COMM_WORLD, &statusWS);
    WheelState wheel_state;
    wheel_state.pos = ChVector<>(bufWS[0], bufWS[1], bufWS[2]);
    wheel_state.rot = ChQuaternion<>(bufWS[3], bufWS[4], bufWS[5], bufWS[6]);
    wheel_state.lin_vel = ChVector<>(bufWS[7], bufWS[8], bufWS[9]);
    wheel_state.ang_vel = ChVector<>(bufWS[10], bufWS[11], bufWS[12]);
    wheel_state.omega = bufWS[13];

    if (m_verbose_states) {
        cout << m_prefix << " recv rim state: " << bufWS[0] << " " << bufWS[1] << " " << bufWS[2] << "  ,  ";
        cout << bufWS[3] << " " << bufWS[4] << " " << bufWS[5] << " " << bufWS[6] << "\n";
        cout << m_prefix << "                 " << bufWS[7] << " " << bufWS[8] << " " << bufWS[9] << "  ,  ";
        cout << bufWS[10] << " " << bufWS[11] << " " << bufWS[12] << endl;
    }

    m_rim->SetPos(wheel_state.pos);
    m_rim->SetRot(wheel_state.rot);
    m_rim->SetPos_dt(wheel_state.lin_vel);
    m_rim->SetWvel_par(wheel_state.ang_vel);

    // -------------------------------
    // Communication with TERRAIN node
    // -------------------------------

    // Extract tire mesh vertex locations and velocites.
    std::vector<ChVector<>> vert_pos;
    std::vector<ChVector<>> vert_vel;
    std::vector<ChVector<int>> triangles;  // ignored here
    m_tire_wrapper->GetMeshState(vert_pos, vert_vel, triangles);

    // Display information on lowest contact vertex.
    if (m_verbose_states) {
        PrintLowestVertex(vert_pos, vert_vel);
    }

    // Send tire mesh vertex locations and velocities to the terrain node
    unsigned int num_vert = (unsigned int)vert_pos.size();
    double* vert_data = new double[2 * 3 * num_vert];
    for (unsigned int iv = 0; iv < num_vert; iv++) {
        vert_data[3 * iv + 0] = vert_pos[iv].x();
        vert_data[3 * iv + 1] = vert_pos[iv].y();
        vert_data[3 * iv + 2] = vert_pos[iv].z();
    }
    for (unsigned int iv = 0; iv < num_vert; iv++) {
        vert_data[3 * num_vert + 3 * iv + 0] = vert_vel[iv].x();
        vert_data[3 * num_vert + 3 * iv + 1] = vert_vel[iv].y();
        vert_data[3 * num_vert + 3 * iv + 2] = vert_vel[iv].z();
    }
    MPI_Send(vert_data, 2 * 3 * num_vert, MPI_DOUBLE, TERRAIN_NODE_RANK, step_number, MPI_COMM_WORLD);

    // Receive terrain forces.
    // Note that we use MPI_Probe to figure out the number of indices and forces received.
    MPI_Status status;
    int count;
    MPI_Probe(TERRAIN_NODE_RANK, step_number, MPI_COMM_WORLD, &status);
    MPI_Get_count(&status, MPI_INT, &count);
    int* index_data = new int[count];
    double* force_data = new double[3 * count];
    MPI_Recv(index_data, count, MPI_INT, TERRAIN_NODE_RANK, step_number, MPI_COMM_WORLD, &status);
    MPI_Recv(force_data, 3 * count, MPI_DOUBLE, TERRAIN_NODE_RANK, step_number, MPI_COMM_WORLD, &status);

    if (m_verbose)
        cout << m_prefix << " step number: " << step_number << "  vertices in contact: " << count << endl;

    // Repack data and apply forces to the mesh vertices
    m_vert_indices.resize(count);
    m_vert_pos.resize(count);
    m_vert_forces.resize(count);
    for (int iv = 0; iv < count; iv++) {
        int index = index_data[iv];
        m_vert_indices[iv] = index;
        m_vert_pos[iv] = vert_pos[index];
        m_vert_forces[iv] = ChVector<>(force_data[3 * iv + 0], force_data[3 * iv + 1], force_data[3 * iv + 2]);
    }
    m_tire_wrapper->SetContactForces(m_rim, m_vert_indices, m_vert_pos, m_vert_forces);

    if (m_verbose_forces) {
        PrintContactData(m_vert_forces, m_vert_indices);
    }

    delete[] vert_data;

    delete[] index_data;
    delete[] force_data;
}

// -----------------------------------------------------------------------------
// Extract mesh state information (SEND to terrain node)
// -----------------------------------------------------------------------------

void TireANCF::GetMeshState(std::vector<ChVector<>>& vert_pos,
                            std::vector<ChVector<>>& vert_vel,
                            std::vector<ChVector<int>>& triangles) {
    m_contact_load->OutputSimpleMesh(vert_pos, vert_vel, triangles);
}

void TireRigid::GetMeshState(std::vector<ChVector<>>& vert_pos,
                             std::vector<ChVector<>>& vert_vel,
                             std::vector<ChVector<int>>& triangles) {
    triangles = m_tire->GetMeshConnectivity();
    m_tire->GetMeshVertexStates(vert_pos, vert_vel);
}

// -----------------------------------------------------------------------------
// Apply contact forces (RECV from terrain node)
// -----------------------------------------------------------------------------

void TireANCF::SetContactForces(std::shared_ptr<chrono::ChBody> rim,
                                const std::vector<int>& vert_indices,
                                const std::vector<chrono::ChVector<>>& vert_pos,
                                const std::vector<chrono::ChVector<>>& vert_forces) {
    m_contact_load->InputSimpleForces(vert_forces, vert_indices);
}

void TireRigid::SetContactForces(std::shared_ptr<chrono::ChBody> rim,
                                 const std::vector<int>& vert_indices,
                                 const std::vector<chrono::ChVector<>>& vert_pos,
                                 const std::vector<chrono::ChVector<>>& vert_forces) {
    rim->Empty_forces_accumulators();
    for (size_t i = 0; i < vert_indices.size(); ++i) {
        rim->Accumulate_force(vert_forces[i], vert_pos[i], false);
    }

    m_tire_force.force = rim->Get_accumulated_force();
    m_tire_force.moment = rim->Get_accumulated_torque();
    m_tire_force.point = rim->GetPos();
}

// -----------------------------------------------------------------------------
// Extract tire force on wheel (SEND to vehicle node)
// -----------------------------------------------------------------------------

void TireANCF::GetTireForce(chrono::vehicle::TerrainForce& tire_force) {
    tire_force = m_tire->ReportTireForce(nullptr);
}

void TireRigid::GetTireForce(chrono::vehicle::TerrainForce& tire_force) {
    tire_force = m_tire_force;
}

// -----------------------------------------------------------------------------
// Advance simulation of the tire node by the specified duration
// -----------------------------------------------------------------------------
void TireNode::Advance(double step_size) {
    m_timer.reset();
    m_timer.start();
    double t = 0;
    while (t < step_size) {
        m_tire_wrapper->OnAdvance();
        double h = std::min<>(m_step_size, step_size - t);
        m_system->DoStepDynamics(h);
        t += h;
    }
    m_timer.stop();
    m_cum_sim_time += m_timer();
}

// -----------------------------------------------------------------------------
// Callback invoked before taking a new step
// -----------------------------------------------------------------------------

void TireANCF::OnAdvance() {
    m_tire->GetMesh()->ResetCounters();
    m_tire->GetMesh()->ResetTimers();
}

// -----------------------------------------------------------------------------
// Append current information to cumulative output stream
// -----------------------------------------------------------------------------
void TireNode::OutputData(int frame) {
    // Append to results output file
    if (m_outf.is_open()) {
        std::string del("  ");

        const ChVector<>& rim_pos = m_rim->GetPos();
        const ChVector<>& rim_vel = m_rim->GetPos_dt();
        const ChVector<>& rim_angvel = m_rim->GetWvel_loc();

        m_outf << m_system->GetChTime() << del;
        // Body states
        m_outf << rim_pos.x() << del << rim_pos.y() << del << rim_pos.z() << del;
        m_outf << rim_vel.x() << del << rim_vel.y() << del << rim_vel.z() << del;
        m_outf << rim_angvel.x() << del << rim_angvel.y() << del << rim_angvel.z() << del;
        // Solver statistics (for last integration step)
        m_outf << m_system->GetTimerStep() << del << m_system->GetTimerSetup() << del << m_system->GetTimerSolver()
               << del << m_system->GetTimerUpdate() << del;
        m_outf << m_integrator->GetNumIterations() << del << m_integrator->GetNumSetupCalls() << del
               << m_integrator->GetNumSolveCalls() << del;
        // Tire-specific stats
        m_tire_wrapper->OutputData(m_outf, del);
        m_outf << endl;
    }

    // Create and write frame output file.
    utils::CSV_writer csv(" ");

    csv << m_system->GetChTime() << endl;
    csv << m_rim->GetIdentifier() << m_rim->GetPos() << m_rim->GetRot() << m_rim->GetPos_dt() << m_rim->GetRot_dt()
        << endl;

    // Write tire state infromation, connectivity and strain state, and vertex contact forces
    m_tire_wrapper->WriteStateInformation(csv);
    m_tire_wrapper->WriteMeshInformation(csv);
    m_tire_wrapper->WriteContactInformation(csv, m_rim, m_vert_indices, m_vert_pos, m_vert_forces);

    char filename[100];
    sprintf(filename, "%s/data_%04d.dat", m_node_out_dir.c_str(), frame + 1);
    csv.write_to_file(filename);

    if (m_verbose)
        cout << m_prefix << " write output file ==> " << filename << endl;
}

// -----------------------------------------------------------------------------
// Append tire-specific solution stats in cumulative output stream
// -----------------------------------------------------------------------------

void TireANCF::OutputData(std::ofstream& outf, const std::string& del) {
    auto mesh = m_tire->GetMesh();

    outf << mesh->GetTimeInternalForces() << del << mesh->GetTimeJacobianLoad() << del;
    outf << mesh->GetNumCallsInternalForces() << del << mesh->GetNumCallsJacobianLoad() << del;
}

// -----------------------------------------------------------------------------
// Write tire mesh node state information
// -----------------------------------------------------------------------------

void TireANCF::WriteStateInformation(utils::CSV_writer& csv) {
    // Extract vertex states from mesh
    auto mesh = m_tire->GetMesh();
    ChState x(mesh->GetDOF(), NULL);
    ChStateDelta v(mesh->GetDOF_w(), NULL);
    unsigned int offset_x = 0;
    unsigned int offset_v = 0;
    double t;
    for (unsigned int in = 0; in < mesh->GetNnodes(); in++) {
        auto node = mesh->GetNode(in);
        node->NodeIntStateGather(offset_x, x, offset_v, v, t);
        offset_x += node->Get_ndof_x();
        offset_v += node->Get_ndof_w();
    }

    // Write number of vertices, number of DOFs
    csv << mesh->GetNnodes() << mesh->GetDOF() << mesh->GetDOF_w() << endl;

    // Write mesh vertex positions and velocities
    for (int ix = 0; ix < x.GetLength(); ix++)
        csv << x(ix) << endl;
    for (int iv = 0; iv < v.GetLength(); iv++)
        csv << v(iv) << endl;
}

void TireRigid::WriteStateInformation(utils::CSV_writer& csv) {
    // Write number of vertices
    unsigned int num_vertices = m_tire->GetNumVertices();
    csv << num_vertices << endl;

    // Write mesh vertex positions and velocities
    std::vector<ChVector<>> pos;
    std::vector<ChVector<>> vel;
    m_tire->GetMeshVertexStates(pos, vel);
    for (unsigned int in = 0; in < num_vertices; in++)
        csv << pos[in] << endl;
    for (unsigned int in = 0; in < num_vertices; in++)
        csv << vel[in] << endl;
}

// -----------------------------------------------------------------------------
// Write tire mesh connectivity and strain information
// -----------------------------------------------------------------------------

void TireANCF::WriteMeshInformation(utils::CSV_writer& csv) {
    // Extract mesh
    auto mesh = m_tire->GetMesh();

    // Vector to roughly interpolate strain information
    std::vector<std::vector<int>> NodeNeighborElement;
    NodeNeighborElement.resize(mesh->GetNnodes());

    // Print tire mesh connectivity
    csv << "\n Connectivity " << mesh->GetNelements() << 5 * mesh->GetNelements() << "\n";

    for (unsigned int ie = 0; ie < mesh->GetNelements(); ie++) {
        for (unsigned int in = 0; in < m_adjVertices[ie].size(); in++) {
            csv << m_adjVertices[ie][in];
        }
        csv << endl;
    }

    // Print strain information: eps_xx, eps_yy, eps_xy averaged over surrounding elements
    csv << "\n Vectors of Strains \n";
    for (unsigned int in = 0; in < mesh->GetNnodes(); in++) {
        double areaX = 0, areaY = 0, areaZ = 0;
        double area = 0;
        for (unsigned int ie = 0; ie < m_adjElements[in].size(); ie++) {
            auto element = std::static_pointer_cast<fea::ChElementShellANCF>(mesh->GetElement(m_adjElements[in][ie]));
            ChVector<> StrainVector = element->EvaluateSectionStrains();
            double dx = element->GetLengthX();
            double dy = element->GetLengthY();
            area += dx * dy / 4;
            areaX += StrainVector.x() * dx * dy / 4;
            areaY += StrainVector.y() * dx * dy / 4;
            areaZ += StrainVector.z() * dx * dy / 4;
        }
        csv << areaX / area << " " << areaY / area << " " << areaZ / area << endl;
    }
}

void TireRigid::WriteMeshInformation(utils::CSV_writer& csv) {
    // Print tire mesh connectivity
    csv << "\n Connectivity " << m_tire->GetNumTriangles() << endl;

    const std::vector<ChVector<int>>& triangles = m_tire->GetMeshConnectivity();
    for (unsigned int ie = 0; ie < m_tire->GetNumTriangles(); ie++) {
        csv << triangles[ie] << endl;
    }
}

// -----------------------------------------------------------------------------
// Write contact forces on tire mesh vertices
// -----------------------------------------------------------------------------

void TireANCF::WriteContactInformation(utils::CSV_writer& csv,
                                       std::shared_ptr<chrono::ChBody> rim,
                                       const std::vector<int>& vert_indices,
                                       const std::vector<chrono::ChVector<>>& vert_pos,
                                       const std::vector<chrono::ChVector<>>& vert_forces) {
    // Extract mesh
    auto mesh = m_tire->GetMesh();

    // Write the number of vertices in contact
    csv << vert_indices.size() << endl;

    // For each vertex in contact, calculate a representative area by averaging
    // the areas of its adjacent elements.
    for (unsigned int iv = 0; iv < vert_indices.size(); iv++) {
        int in = vert_indices[iv];
        auto node = std::dynamic_pointer_cast<fea::ChNodeFEAxyzD>(mesh->GetNode(in));
        double area = 0;
        for (unsigned int ie = 0; ie < m_adjElements[in].size(); ie++) {
            auto element = std::static_pointer_cast<fea::ChElementShellANCF>(mesh->GetElement(m_adjElements[in][ie]));
            double dx = element->GetLengthX();
            double dy = element->GetLengthY();
            area += dx * dy / 4;
        }
        area /= m_adjElements[in].size();
        // Output vertex index, position, contact force, normal, and area.
        csv << in << vert_pos[iv] << vert_forces[iv] << node->GetD().GetNormalized() << area << endl;
    }
}

void TireRigid::WriteContactInformation(utils::CSV_writer& csv,
                                        std::shared_ptr<chrono::ChBody> rim,
                                        const std::vector<int>& vert_indices,
                                        const std::vector<chrono::ChVector<>>& vert_pos,
                                        const std::vector<chrono::ChVector<>>& vert_forces) {
    // Write the number of vertices in contact
    csv << vert_indices.size() << endl;

    // For each vertex in contact, output vertex index, contact force, normal, and area.
    const std::vector<ChVector<>>& normals = m_tire->GetMeshNormals();
    for (unsigned int iv = 0; iv < vert_indices.size(); iv++) {
        int in = vert_indices[iv];
        ChVector<> nrm = rim->TransformDirectionLocalToParent(normals[in]);
        csv << in << vert_pos[iv] << vert_forces[iv] << nrm << m_vertexArea[in] << endl;
    }
}

// -----------------------------------------------------------------------------
// Log various stats
// -----------------------------------------------------------------------------
void TireNode::PrintLowestVertex(const std::vector<ChVector<>>& vert_pos, const std::vector<ChVector<>>& vert_vel) {
    auto lowest = std::min_element(vert_pos.begin(), vert_pos.end(),
                                   [](const ChVector<>& a, const ChVector<>& b) { return a.z() < b.z(); });
    auto index = lowest - vert_pos.begin();
    const ChVector<>& vel = vert_vel[index];
    cout << m_prefix << " lowest vertex:  index = " << index << "  height = " << (*lowest).z() << "  velocity = " << vel.x()
         << "  " << vel.y() << "  " << vel.z() << endl;
}

void TireNode::PrintContactData(const std::vector<ChVector<>>& forces, const std::vector<int>& indices) {
    if (indices.size() == 0)
        return;

    cout << m_prefix << " contact forces" << endl;
    for (int i = 0; i < indices.size(); i++) {
        cout << "  id = " << indices[i] << "  force = " << forces[i].x() << "  " << forces[i].y() << "  " << forces[i].z()
             << endl;
    }
}
