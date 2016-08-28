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
#include "mpi.h"

#include "chrono/ChConfig.h"
#include "chrono/physics/ChLinkLock.h"
#include "chrono/physics/ChSystemDEM.h"
#include "chrono/timestepper/ChState.h"
#include "chrono/utils/ChUtilsCreators.h"

#include "chrono_fea/ChLoadContactSurfaceMesh.h"
#include "chrono_fea/ChElementShellANCF.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ANCFTire.h"

#ifdef CHRONO_MKL
#include "chrono_mkl/ChSolverMKL.h"
#endif

#include "TireNode.h"

using std::cout;
using std::endl;

using namespace chrono;
using namespace chrono::vehicle;

// =============================================================================

// -----------------------------------------------------------------------------
// Construction of the tire node:
// - create the (sequential) Chrono system and set solver parameters
// -----------------------------------------------------------------------------
TireNode::TireNode(WheelID wheel_id, int num_threads) : BaseNode(""), m_wheel_id(wheel_id) {
    m_name = "TIRE_" + std::to_string(m_wheel_id.id());
    m_prefix = "[Tire node " + std::to_string(m_wheel_id.id()) + " ]";

    cout << m_prefix << " axle = " << m_wheel_id.axle() << " side = " << m_wheel_id.side()
         << " num_threads = " << num_threads << endl;

    // ------------------------
    // Default model parameters
    // ------------------------

    //// TODO: should these be user-specified? received from vehicle node?
    m_rim_mass = 15;
    m_rim_inertia = ChVector<>(1, 1, 1);
    m_rim_fixed = false;

    m_tire_pressure = true;

    // ----------------------------------
    // Create the (sequential) DEM system
    // ----------------------------------

    m_system = new ChSystemDEM;
    m_system->Set_G_acc(ChVector<>(0, 0, m_gacc));

    // Set number threads
    m_system->SetParallelThreadNumber(num_threads);
    CHOMPfunctions::SetNumThreads(num_threads);

#ifdef CHRONO_MKL
    // Solver settings
    ChSolverMKL<>* mkl_solver_stab = new ChSolverMKL<>;
    ChSolverMKL<>* mkl_solver_speed = new ChSolverMKL<>;
    m_system->ChangeSolverStab(mkl_solver_stab);
    m_system->ChangeSolverSpeed(mkl_solver_speed);
    mkl_solver_speed->SetSparsityPatternLock(true);
    mkl_solver_stab->SetSparsityPatternLock(true);
#else
    // Solver settings
    m_system->SetMaxItersSolverSpeed(100);
    m_system->SetMaxItersSolverStab(100);
    m_system->SetSolverType(ChSystem::SOLVER_SOR);
    m_system->SetTol(1e-10);
    m_system->SetTolForce(1e-8);
#endif

    // Integrator settings
    m_system->SetIntegrationType(ChSystem::INT_HHT);
    m_integrator = std::static_pointer_cast<ChTimestepperHHT>(m_system->GetTimestepper());
    m_integrator->SetAlpha(-0.2);
    m_integrator->SetMaxiters(50);
    m_integrator->SetAbsTolerances(5e-05, 1.8e00);
    m_integrator->SetMode(ChTimestepperHHT::POSITION);
    m_integrator->SetScaling(true);
    m_integrator->SetVerbose(true);
    m_integrator->SetMaxItersSuccess(5);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
TireNode::~TireNode() {
    delete m_system;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------

void TireNode::SetTireJSONFile(const std::string& filename) {
    m_tire_json = filename;
}

void TireNode::EnableTirePressure(bool val) {
    m_tire_pressure = val;
}

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
    m_rim = std::make_shared<ChBody>();
    m_rim->SetMass(m_rim_mass);
    m_rim->SetInertiaXX(m_rim_inertia);
    m_rim->SetBodyFixed(m_rim_fixed);
    m_system->AddBody(m_rim);

    // Initialize rim body (always zero linear and angular velocities)
    m_rim->SetPos(loc);
    m_rim->SetRot(rot);
    m_rim->SetPos_dt(ChVector<>(0, 0, 0));
    m_rim->SetWvel_loc(ChVector<>(0, 0, 0));

    // Create the tire
    m_tire = std::make_shared<ANCFTire>(m_tire_json);
    m_tire->EnablePressure(m_tire_pressure);
    m_tire->EnableContact(true);
    m_tire->EnableRimConnection(true);
    m_tire->SetContactSurfaceType(ChDeformableTire::TRIANGLE_MESH);

    // Initialize tire
    m_tire->Initialize(m_rim, LEFT);

    // Create a mesh load for contact forces and add it to the tire's load container
    auto contact_surface = std::static_pointer_cast<fea::ChContactSurfaceMesh>(m_tire->GetContactSurface());
    m_contact_load = std::make_shared<fea::ChLoadContactSurfaceMesh>(contact_surface);
    m_tire->GetLoadContainer()->Add(m_contact_load);

    // Mark completion of system construction
    m_system->SetupInitial();

    // ---------------------------------------
    // Send tire contact surface specification
    // ---------------------------------------

    unsigned int surf_props[2];
    surf_props[0] = contact_surface->GetNumVertices();
    surf_props[1] = contact_surface->GetNumTriangles();
    MPI_Send(surf_props, 2, MPI_UNSIGNED, TERRAIN_NODE_RANK, 0, MPI_COMM_WORLD);

    cout << m_prefix << " vertices = " << surf_props[0] << "  triangles = " << surf_props[1] << endl;

    // -------------------------------------
    // Send tire contact material properties
    // -------------------------------------

    auto contact_mat = m_tire->GetContactMaterial();
    float mat_props[8] = {m_tire->GetCoefficientFriction(),
                          m_tire->GetCoefficientRestitution(),
                          m_tire->GetYoungModulus(),
                          m_tire->GetPoissonRatio(),
                          m_tire->GetKn(),
                          m_tire->GetGn(),
                          m_tire->GetKt(),
                          m_tire->GetGt()};

    MPI_Send(mat_props, 8, MPI_FLOAT, TERRAIN_NODE_RANK, 0, MPI_COMM_WORLD);

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
    outf << "   Pressure enabled? " << (m_tire_pressure ? "YES" : "NO") << endl;
    outf << "Rim body" << endl;
    outf << "   mass = " << m_rim_mass << endl;
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
    // Communication with TERRAIN node
    // -------------------------------

    // Extract tire mesh vertex locations and velocites.
    std::vector<ChVector<>> vert_pos;
    std::vector<ChVector<>> vert_vel;
    std::vector<ChVector<int>> triangles;
    m_contact_load->OutputSimpleMesh(vert_pos, vert_vel, triangles);

    // Display information on lowest mesh node and lowest contact vertex.
    PrintLowestNode();
    PrintLowestVertex(vert_pos, vert_vel);

    // Send tire mesh vertex locations and velocities to the terrain node
    unsigned int num_vert = (unsigned int)vert_pos.size();
    unsigned int num_tri = (unsigned int)triangles.size();
    double* vert_data = new double[2 * 3 * num_vert];
    int* tri_data = new int[3 * num_tri];
    for (unsigned int iv = 0; iv < num_vert; iv++) {
        vert_data[3 * iv + 0] = vert_pos[iv].x;
        vert_data[3 * iv + 1] = vert_pos[iv].y;
        vert_data[3 * iv + 2] = vert_pos[iv].z;
    }
    for (unsigned int iv = 0; iv < num_vert; iv++) {
        vert_data[3 * num_vert + 3 * iv + 0] = vert_vel[iv].x;
        vert_data[3 * num_vert + 3 * iv + 1] = vert_vel[iv].y;
        vert_data[3 * num_vert + 3 * iv + 2] = vert_vel[iv].z;
    }
    for (unsigned int it = 0; it < num_tri; it++) {
        tri_data[3 * it + 0] = triangles[it].x;
        tri_data[3 * it + 1] = triangles[it].y;
        tri_data[3 * it + 2] = triangles[it].z;
    }
    MPI_Send(vert_data, 2 * 3 * num_vert, MPI_DOUBLE, TERRAIN_NODE_RANK, step_number, MPI_COMM_WORLD);
    MPI_Send(tri_data, 3 * num_tri, MPI_INT, TERRAIN_NODE_RANK, step_number, MPI_COMM_WORLD);

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
    m_contact_load->InputSimpleForces(m_vert_forces, m_vert_indices);

    PrintContactData(m_vert_forces, m_vert_indices);

    delete[] vert_data;
    delete[] tri_data;

    delete[] index_data;
    delete[] force_data;

    // -------------------------------
    // Communication with VEHICLE node
    // -------------------------------

    //// TODO check this

    // Send tire force to the vehicle node
    TireForce tire_force = m_tire->GetTireForce(true);
    double bufTF[9];
    bufTF[0] = tire_force.force.x;
    bufTF[1] = tire_force.force.y;
    bufTF[2] = tire_force.force.z;
    bufTF[3] = tire_force.moment.x;
    bufTF[4] = tire_force.moment.y;
    bufTF[5] = tire_force.moment.z;
    bufTF[6] = tire_force.point.x;
    bufTF[7] = tire_force.point.y;
    bufTF[8] = tire_force.point.z;
    MPI_Send(bufTF, 9, MPI_DOUBLE, VEHICLE_NODE_RANK, m_wheel_id.id(), MPI_COMM_WORLD);

    cout << m_prefix << " sent tire forces: " << bufTF[0] << " " << bufTF[1] << " " << bufTF[2] << "  ,  ";
    cout << bufTF[3] << " " << bufTF[4] << " " << bufTF[5] << "  ,  ";
    cout << bufTF[6] << " " << bufTF[7] << " " << bufTF[8] << endl;

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

    cout << m_prefix << " recv rim state: " << bufWS[0] << " " << bufWS[1] << " " << bufWS[2] << "  ,  ";
    cout << bufWS[3] << " " << bufWS[4] << " " << bufWS[5] << " " << bufWS[6] << endl;

    m_rim->SetPos(wheel_state.pos);
    m_rim->SetRot(wheel_state.rot);
    m_rim->SetPos_dt(wheel_state.lin_vel);
    m_rim->SetWvel_par(wheel_state.ang_vel);
}

// -----------------------------------------------------------------------------
// Advance simulation of the tire node by the specified duration
// -----------------------------------------------------------------------------
void TireNode::Advance(double step_size) {
    m_timer.reset();
    m_timer.start();
    double t = 0;
    while (t < step_size) {
        m_tire->GetMesh()->ResetCounters();
        m_tire->GetMesh()->ResetTimers();
        double h = std::min<>(m_step_size, step_size - t);
        m_system->DoStepDynamics(h);
        t += h;
    }
    m_timer.stop();
    m_cum_sim_time += m_timer();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void TireNode::OutputData(int frame) {
    // Append to results output file
    if (m_outf.is_open()) {
        std::string del("  ");

        const ChVector<>& rim_pos = m_rim->GetPos();
        const ChVector<>& rim_vel = m_rim->GetPos_dt();
        const ChVector<>& rim_angvel = m_rim->GetWvel_loc();

        auto mesh = m_tire->GetMesh();

        m_outf << m_system->GetChTime() << del;
        // Body states
        m_outf << rim_pos.x << del << rim_pos.y << del << rim_pos.z << del;
        m_outf << rim_vel.x << del << rim_vel.y << del << rim_vel.z << del;
        m_outf << rim_angvel.x << del << rim_angvel.y << del << rim_angvel.z << del;
        // Solver statistics (for last integration step)
        m_outf << m_system->GetTimerStep() << del << m_system->GetTimerSetup() << del << m_system->GetTimerSolver()
               << del << m_system->GetTimerUpdate();
        m_outf << mesh->GetTimeInternalForces() << del << mesh->GetTimeJacobianLoad();
        m_outf << m_integrator->GetNumIterations() << del << m_integrator->GetNumSetupCalls() << del
               << m_integrator->GetNumSolveCalls();
        m_outf << mesh->GetNumCallsInternalForces() << del << mesh->GetNumCallsJacobianLoad();
        m_outf << endl;
    }

    // Create and write frame output file.
    char filename[100];
    sprintf(filename, "%s/data_%04d.dat", m_node_out_dir.c_str(), frame + 1);

    utils::CSV_writer csv(" ");
    csv << m_system->GetChTime() << endl;  // current time
    WriteStateInformation(csv);            // state of bodies and tire
    WriteMeshInformation(csv);             // connectivity and strain state
    WriteContactInformation(csv);          // vertex contact forces
    csv.write_to_file(filename);

    cout << m_prefix << " write output file ==> " << filename << endl;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void TireNode::WriteStateInformation(utils::CSV_writer& csv) {
    csv << m_rim->GetIdentifier() << m_rim->GetPos() << m_rim->GetRot() << m_rim->GetPos_dt() << m_rim->GetRot_dt()
        << endl;

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

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void TireNode::WriteMeshInformation(utils::CSV_writer& csv) {
    // Extract mesh
    auto my_mesh = m_tire->GetMesh();

    // Vector to roughly interpolate strain information
    std::vector<std::vector<int>> NodeNeighborElement;
    NodeNeighborElement.resize(my_mesh->GetNnodes());

    // Print tire mesh connectivity
    std::vector<std::shared_ptr<fea::ChNodeFEAbase>> myvector;
    myvector.resize(my_mesh->GetNnodes());
    for (unsigned int i = 0; i < my_mesh->GetNnodes(); i++) {
        myvector[i] = std::dynamic_pointer_cast<fea::ChNodeFEAbase>(my_mesh->GetNode(i));
    }
    csv << "\n Connectivity " << my_mesh->GetNelements() << 5 * my_mesh->GetNelements() << "\n";

    for (unsigned int iele = 0; iele < my_mesh->GetNelements(); iele++) {
        auto element = my_mesh->GetElement(iele);
        int nodeOrder[] = {0, 1, 2, 3};
        for (int myNodeN = 0; myNodeN < 4; myNodeN++) {
            auto nodeA = element->GetNodeN(nodeOrder[myNodeN]);
            std::vector<std::shared_ptr<fea::ChNodeFEAbase>>::iterator it;
            it = find(myvector.begin(), myvector.end(), nodeA);
            if (it != myvector.end()) {
                auto index = std::distance(myvector.begin(), it);
                csv << (unsigned int)index << " ";
                NodeNeighborElement[index].push_back(iele);
            }
        }
        csv << "\n";
    }

    // Print strain information: eps_xx, eps_yy, eps_xy averaged over 4 surrounding elements
    csv << "\n Vectors of Strains \n";
    for (unsigned int i = 0; i < my_mesh->GetNnodes(); i++) {
        double areaAve1 = 0, areaAve2 = 0, areaAve3 = 0;
        double myarea = 0;
        for (int j = 0; j < NodeNeighborElement[i].size(); j++) {
            int myelemInx = NodeNeighborElement[i][j];
            auto element = std::dynamic_pointer_cast<fea::ChElementShellANCF>(my_mesh->GetElement(myelemInx));
            ChVector<> StrainVector = element->EvaluateSectionStrains();
            double dx = element->GetLengthX();
            double dy = element->GetLengthY();
            myarea += dx * dy / 4;
            areaAve1 += StrainVector.x * dx * dy / 4;
            areaAve2 += StrainVector.y * dx * dy / 4;
            areaAve3 += StrainVector.z * dx * dy / 4;
        }
        csv << areaAve1 / myarea << " " << areaAve2 / myarea << " " << areaAve3 / myarea << "\n";
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void TireNode::WriteContactInformation(utils::CSV_writer& csv) {
    csv << m_vert_indices.size() << endl;
    // Output nodal position, contact force, and normal vectors; and
    // representative nodal area

    // Extract mesh
    auto my_mesh = m_tire->GetMesh();

    // Vector to identify surrounding elements to a node (4 max, 2 min)
    std::vector<std::vector<int>> NodeNeighborElement;
    NodeNeighborElement.resize(my_mesh->GetNnodes());

    // Create vector with all nodes
    std::vector<std::shared_ptr<fea::ChNodeFEAbase>> myvector;
    myvector.resize(my_mesh->GetNnodes());
    for (unsigned int i = 0; i < my_mesh->GetNnodes(); i++) {
        myvector[i] = std::dynamic_pointer_cast<fea::ChNodeFEAbase>(my_mesh->GetNode(i));
    }

    // Go through the nodes of all elements and store neighboring elements to each node
    for (unsigned int iele = 0; iele < my_mesh->GetNelements(); iele++) {
        auto element = my_mesh->GetElement(iele);
        int nodeOrder[] = {0, 1, 2, 3};
        for (int myNodeN = 0; myNodeN < 4; myNodeN++) {
            auto nodeA = element->GetNodeN(nodeOrder[myNodeN]);
            std::vector<std::shared_ptr<fea::ChNodeFEAbase>>::iterator it;
            it = find(myvector.begin(), myvector.end(), nodeA);
            if (it != myvector.end()) {
                auto index = std::distance(myvector.begin(), it);
                NodeNeighborElement[index].push_back(iele);
            }
        }
    }

    // Loop to calculate representative area of a contacting node (node with net contact force)
    for (unsigned int iv = 0; iv < m_vert_indices.size(); iv++) {
        double myarea = 0;

        for (int j = 0; j < NodeNeighborElement[iv].size(); j++) {
            int myelemInx = NodeNeighborElement[iv][j];
            auto element = std::dynamic_pointer_cast<fea::ChElementShellANCF>(my_mesh->GetElement(myelemInx));
            double dx = element->GetLengthX();
            double dy = element->GetLengthY();
            myarea += dx * dy / NodeNeighborElement[iv].size();
        }
        // Output index, position, force, and normal vectors, and representative area
        csv << m_vert_indices[iv] << m_vert_pos[iv] << m_vert_forces[iv]
            << std::dynamic_pointer_cast<fea::ChNodeFEAxyzD>(m_tire->GetMesh()->GetNode(m_vert_indices[iv]))
                   ->GetD()
                   .GetNormalized()
            << myarea << endl;
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void TireNode::PrintLowestNode() {
    // Unfortunately, we do not have access to the node container of a mesh,
    // so we cannot use some nice algorithm here...
    unsigned int num_nodes = m_tire->GetMesh()->GetNnodes();
    unsigned int index = 0;
    double zmin = 1e10;
    for (unsigned int i = 0; i < num_nodes; ++i) {
        // Ugly casting here. (Note also that we need dynamic downcasting, due to the virtual base)
        auto node = std::dynamic_pointer_cast<fea::ChNodeFEAxyz>(m_tire->GetMesh()->GetNode(i));
        if (node->GetPos().z < zmin) {
            zmin = node->GetPos().z;
            index = i;
        }
    }

    ChVector<> vel = std::dynamic_pointer_cast<fea::ChNodeFEAxyz>(m_tire->GetMesh()->GetNode(index))->GetPos_dt();
    cout << m_prefix << " lowest node:    index = " << index << "  height = " << zmin << "  velocity = " << vel.x
         << "  " << vel.y << "  " << vel.z << endl;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void TireNode::PrintLowestVertex(const std::vector<ChVector<>>& vert_pos, const std::vector<ChVector<>>& vert_vel) {
    auto lowest = std::min_element(vert_pos.begin(), vert_pos.end(),
                                   [](const ChVector<>& a, const ChVector<>& b) { return a.z < b.z; });
    int index = lowest - vert_pos.begin();
    const ChVector<>& vel = vert_vel[index];
    cout << m_prefix << " lowest vertex:  index = " << index << "  height = " << (*lowest).z << "  velocity = " << vel.x
         << "  " << vel.y << "  " << vel.z << endl;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void TireNode::PrintContactData(const std::vector<ChVector<>>& forces, const std::vector<int>& indices) {
    cout << m_prefix << " contact forces" << endl;
    for (int i = 0; i < indices.size(); i++) {
        cout << "  id = " << indices[i] << "  force = " << forces[i].x << "  " << forces[i].y << "  " << forces[i].z
             << endl;
    }
}
