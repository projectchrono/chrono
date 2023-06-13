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
// Definition of the base vehicle co-simulation TIRE NODE class.
//
// The global reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include "chrono/ChConfig.h"
#include "chrono/solver/ChIterativeSolver.h"
#include "chrono/solver/ChDirectSolverLS.h"

#ifdef CHRONO_PARDISO_MKL
    #include "chrono_pardisomkl/ChSolverPardisoMKL.h"
#endif

#ifdef CHRONO_MUMPS
#include "chrono_mumps/ChSolverMumps.h"
#endif

#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/cosim/ChVehicleCosimTireNode.h"

#include "chrono_thirdparty/rapidjson/filereadstream.h"
#include "chrono_thirdparty/rapidjson/istreamwrapper.h"

using std::cout;
using std::endl;

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// =============================================================================

// Dummy ChWheel subsystem (needed to attach a ChTire)
class DummyWheel : public ChWheel {
  public:
    DummyWheel() : ChWheel("tire_wheel"), m_inertia(ChVector<>(0)) {}
    virtual double GetWheelMass() const override { return 0; }
    virtual const ChVector<>& GetWheelInertia() const override { return m_inertia; }
    virtual double GetRadius() const override { return 1; }
    virtual double GetWidth() const override { return 1; }

  private:
    ChVector<> m_inertia;
};

// =============================================================================

ChVehicleCosimTireNode::ChVehicleCosimTireNode(int index, const std::string& tire_json)
    : ChVehicleCosimBaseNode("TIRE_" + std::to_string(index)), m_index(index), m_tire_pressure(true) {
    // Default integrator and solver types
    m_int_type = ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED;
    m_slv_type = ChSolver::Type::BARZILAIBORWEIN;

    // Create the (sequential) SMC system
    m_system = new ChSystemSMC;
    m_system->Set_G_acc(ChVector<>(0, 0, m_gacc));

    // Create a tire subsystem from JSON specification file (if provided)
    if (!tire_json.empty())
        m_tire = ReadTireJSON(tire_json);
}

// -----------------------------------------------------------------------------

std::string ChVehicleCosimTireNode::GetTireTypeAsString(TireType type) {
    switch (type) {
        case TireType::RIGID:
            return "RIGID";
        case TireType::FLEXIBLE:
            return "FLEXIBLE";
        default:
            return "UNKNOWN";
    }
}

ChVehicleCosimTireNode::TireType ChVehicleCosimTireNode::GetTireTypeFromString(const std::string& type) {
    if (type == "RIGID")
        return TireType::RIGID;
    if (type == "FLEXIBLE")
        return TireType::FLEXIBLE;

    return TireType::UNKNOWN;
}

bool ChVehicleCosimTireNode::ReadSpecfile(const std::string& specfile, Document& d) {
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

ChVehicleCosimTireNode::TireType ChVehicleCosimTireNode::GetTireTypeFromSpecfile(const std::string& specfile) {
    Document d;
    if (!ReadSpecfile(specfile, d)) {
        return TireType::UNKNOWN;
    }

    if (!d.HasMember("Type") || std::string(d["Type"].GetString()).compare("Tire") != 0) {
        cout << "ERROR: JSON file " << specfile << " is not a tire JSON specification file!\n" << endl;
        return TireType::UNKNOWN;
    }

    std::string tire_template = d["Template"].GetString();
    if (tire_template.compare("RigidTire") == 0) {
        if (d.HasMember("Contact Mesh"))
            return TireType::RIGID;
    }
    if (tire_template.compare("ANCFTire") == 0 || tire_template.compare("ReissnerTire") == 0) {
        return TireType::FLEXIBLE;
    }

    return TireType::UNKNOWN;
}

// -----------------------------------------------------------------------------

void ChVehicleCosimTireNode::EnableTirePressure(bool val) {
    m_tire_pressure = val;
}

void ChVehicleCosimTireNode::SetNumThreads(int num_threads) {
    m_system->SetNumThreads(num_threads, 1, 1);
}

void ChVehicleCosimTireNode::SetIntegratorType(ChTimestepper::Type int_type, ChSolver::Type slv_type) {
    m_int_type = int_type;
    m_slv_type = slv_type;
#ifndef CHRONO_PARDISO_MKL
    if (m_slv_type == ChSolver::Type::PARDISO_MKL)
        m_slv_type = ChSolver::Type::BARZILAIBORWEIN;
#endif
#ifndef CHRONO_MUMPS
    if (m_slv_type == ChSolver::Type::MUMPS)
        m_slv_type = ChSolver::Type::BARZILAIBORWEIN;
#endif
}

void ChVehicleCosimTireNode::Initialize() {
    // Invoke the base class method to figure out distribution of node types
    ChVehicleCosimBaseNode::Initialize();

    // Complete setup of the underlying ChSystem
    InitializeSystem();

    MPI_Status status;

    // Create the spindle body
    m_spindle = chrono_types::make_shared<ChBody>();
    m_system->AddBody(m_spindle);

    // Create the wheel subsystem, arbitrarily assuming LEFT side
    m_wheel = chrono_types::make_shared<DummyWheel>();
    m_wheel->Initialize(m_spindle, LEFT);
    m_wheel->SetVisualizationType(VisualizationType::NONE);

    // Receive from the MBS node the initial location of this tire.
    double loc_data[3];
    MPI_Recv(loc_data, 3, MPI_DOUBLE, MBS_NODE_RANK, 0, MPI_COMM_WORLD, &status);

    // Let derived classes initialize the tire and attach it to the provided ChWheel.
    // Initialize the tire at the specified location (as received from the MBS node).
    InitializeTire(m_wheel, {loc_data[0], loc_data[1], loc_data[2]});

    // Send the tire radius and mass to the (wheeled) MBS node
    double tire_mass = GetTireMass();
    double tire_radius = GetTireRadius();
    double tire_width = GetTireWidth();
    double tire_info[] = {tire_mass, tire_radius, tire_width};
    MPI_Send(tire_info, 3, MPI_DOUBLE, MBS_NODE_RANK, 0, MPI_COMM_WORLD);

    // Receive from the MBS node the load on this tire
    double load_mass;
    MPI_Recv(&load_mass, 1, MPI_DOUBLE, MBS_NODE_RANK, 0, MPI_COMM_WORLD, &status);

    // Overwrite spindle mass and inertia
    ChVector<> spindle_inertia(1, 1, 1);  //// TODO
    m_spindle->SetMass(load_mass);
    m_spindle->SetInertiaXX(spindle_inertia);

    // Send the expected communication interface type to the TERRAIN node (only tire 0 does this)
    if (m_index == 0) {
        char comm_type = (GetInterfaceType() == InterfaceType::BODY) ? 0 : 1;
        MPI_Send(&comm_type, 1, MPI_CHAR, TERRAIN_NODE_RANK, 0, MPI_COMM_WORLD);
    }

    // Send tire geometry
    SendGeometry(m_geometry, TERRAIN_NODE_RANK);

    // Send load on this tire (include the mass of the tire)
    load_mass += GetTireMass();
    MPI_Send(&load_mass, 1, MPI_DOUBLE, TERRAIN_NODE_RANK, 0, MPI_COMM_WORLD);
    if (m_verbose)
        cout << "[Tire node " << m_index << " ] Send: load mass = " << load_mass << endl;
}

void ChVehicleCosimTireNode::InitializeSystem() {
    // Change solver
    switch (m_slv_type) {
        case ChSolver::Type::PARDISO_MKL: {
#ifdef CHRONO_PARDISO_MKL
            auto solver = chrono_types::make_shared<ChSolverPardisoMKL>();
            solver->LockSparsityPattern(true);
            m_system->SetSolver(solver);
#endif
            break;
        }
        case ChSolver::Type::MUMPS: {
#ifdef CHRONO_MUMPS
            auto solver = chrono_types::make_shared<ChSolverMumps>();
            solver->LockSparsityPattern(true);
            m_system->SetSolver(solver);
#endif
            break;
        }
        case ChSolver::Type::SPARSE_LU: {
            auto solver = chrono_types::make_shared<ChSolverSparseLU>();
            solver->LockSparsityPattern(true);
            m_system->SetSolver(solver);
            break;
        }
        case ChSolver::Type::SPARSE_QR: {
            auto solver = chrono_types::make_shared<ChSolverSparseQR>();
            solver->LockSparsityPattern(true);
            m_system->SetSolver(solver);
            break;
        }
        case ChSolver::Type::PSOR:
        case ChSolver::Type::PSSOR:
        case ChSolver::Type::PJACOBI:
        case ChSolver::Type::PMINRES:
        case ChSolver::Type::BARZILAIBORWEIN:
        case ChSolver::Type::APGD:
        case ChSolver::Type::GMRES:
        case ChSolver::Type::MINRES:
        case ChSolver::Type::BICGSTAB: {
            m_system->SetSolverType(m_slv_type);
            auto solver = std::dynamic_pointer_cast<ChIterativeSolver>(m_system->GetSolver());
            assert(solver);
            solver->SetMaxIterations(100);
            solver->SetTolerance(1e-10);
            break;
        }
        default: {
            cout << "Solver type not supported!" << endl;
            return;
        }
    }

    // Change integrator
    switch (m_int_type) {
        case ChTimestepper::Type::HHT:
            m_system->SetTimestepperType(ChTimestepper::Type::HHT);
            m_integrator = std::static_pointer_cast<ChTimestepperHHT>(m_system->GetTimestepper());
            m_integrator->SetAlpha(-0.2);
            m_integrator->SetMaxiters(50);
            m_integrator->SetAbsTolerances(1e-04, 1e2);
            m_integrator->SetMode(ChTimestepperHHT::ACCELERATION);
            m_integrator->SetStepControl(false);
            m_integrator->SetModifiedNewton(false);
            m_integrator->SetScaling(false);
            m_integrator->SetVerbose(false);
            break;

        default:
            break;
    }
}

void ChVehicleCosimTireNode::Synchronize(int step_number, double time) {
    switch (GetInterfaceType()) {
        case InterfaceType::BODY:
            SynchronizeBody(step_number, time);
            break;
        case InterfaceType::MESH:
            SynchronizeMesh(step_number, time);
            break;
    }
}

void ChVehicleCosimTireNode::SynchronizeBody(int step_number, double time) {
    // Act as a simple counduit between the MBS and TERRAIN nodes
    MPI_Status status;

    // Receive spindle state data from MBS node
    double state_data[13];
    MPI_Recv(state_data, 13, MPI_DOUBLE, MBS_NODE_RANK, step_number, MPI_COMM_WORLD, &status);

    BodyState spindle_state;
    spindle_state.pos = ChVector<>(state_data[0], state_data[1], state_data[2]);
    spindle_state.rot = ChQuaternion<>(state_data[3], state_data[4], state_data[5], state_data[6]);
    spindle_state.lin_vel = ChVector<>(state_data[7], state_data[8], state_data[9]);
    spindle_state.ang_vel = ChVector<>(state_data[10], state_data[11], state_data[12]);

    // Pass it to derived class
    ApplySpindleState(spindle_state);

    // Send spindle state data to Terrain node
    MPI_Send(state_data, 13, MPI_DOUBLE, TERRAIN_NODE_RANK, step_number, MPI_COMM_WORLD);
    if (m_verbose)
        cout << "[Tire node " << m_index << " ] Send: spindle position = " << spindle_state.pos << endl;

    // Receive spindle force from TERRAIN NODE and send to MBS node
    double force_data[6];
    MPI_Recv(force_data, 6, MPI_DOUBLE, TERRAIN_NODE_RANK, step_number, MPI_COMM_WORLD, &status);

    TerrainForce spindle_force;
    spindle_force.force = ChVector<>(force_data[0], force_data[1], force_data[2]);
    spindle_force.moment = ChVector<>(force_data[3], force_data[4], force_data[5]);

    if (m_verbose)
        cout << "[Tire node " << m_index << " ] Recv: spindle force = " << spindle_force.force << endl;

    // Pass it to derived class
    ApplySpindleForce(spindle_force);

    // Send spindle force to MBS node
    MPI_Send(force_data, 6, MPI_DOUBLE, MBS_NODE_RANK, step_number, MPI_COMM_WORLD);
}

void ChVehicleCosimTireNode::SynchronizeMesh(int step_number, double time) {
    MPI_Status status;

    // Receive spindle state data from MBS node
    double state_data[13];
    MPI_Recv(state_data, 13, MPI_DOUBLE, MBS_NODE_RANK, step_number, MPI_COMM_WORLD, &status);

    BodyState spindle_state;
    spindle_state.pos = ChVector<>(state_data[0], state_data[1], state_data[2]);
    spindle_state.rot = ChQuaternion<>(state_data[3], state_data[4], state_data[5], state_data[6]);
    spindle_state.lin_vel = ChVector<>(state_data[7], state_data[8], state_data[9]);
    spindle_state.ang_vel = ChVector<>(state_data[10], state_data[11], state_data[12]);

    // Pass it to derived class.
    ApplySpindleState(spindle_state);

    // Send mesh state (vertex locations and velocities) to TERRAIN node
    MeshState mesh_state;
    LoadMeshState(mesh_state);
    unsigned int nvs = (unsigned int)mesh_state.vpos.size();
    double* vert_data = new double[2 * 3 * nvs];
    for (unsigned int iv = 0; iv < nvs; iv++) {
        vert_data[3 * iv + 0] = mesh_state.vpos[iv].x();
        vert_data[3 * iv + 1] = mesh_state.vpos[iv].y();
        vert_data[3 * iv + 2] = mesh_state.vpos[iv].z();
    }
    for (unsigned int iv = 0; iv < nvs; iv++) {
        vert_data[3 * nvs + 3 * iv + 0] = mesh_state.vvel[iv].x();
        vert_data[3 * nvs + 3 * iv + 1] = mesh_state.vvel[iv].y();
        vert_data[3 * nvs + 3 * iv + 2] = mesh_state.vvel[iv].z();
    }
    MPI_Send(vert_data, 2 * 3 * nvs, MPI_DOUBLE, TERRAIN_NODE_RANK, step_number, MPI_COMM_WORLD);

    // Receive mesh forces from TERRAIN node.
    // Note that we use MPI_Probe to figure out the number of indices and forces received.
    int nvc = 0;
    MPI_Probe(TERRAIN_NODE_RANK, step_number, MPI_COMM_WORLD, &status);
    MPI_Get_count(&status, MPI_INT, &nvc);
    int* index_data = new int[nvc];
    double* mesh_contact_data = new double[3 * nvc];
    MPI_Recv(index_data, nvc, MPI_INT, TERRAIN_NODE_RANK, step_number, MPI_COMM_WORLD, &status);
    MPI_Recv(mesh_contact_data, 3 * nvc, MPI_DOUBLE, TERRAIN_NODE_RANK, step_number, MPI_COMM_WORLD, &status);

    MeshContact mesh_contact;
    mesh_contact.nv = nvc;
    mesh_contact.vidx.resize(nvc);
    mesh_contact.vforce.resize(nvc);
    for (int iv = 0; iv < nvc; iv++) {
        int index = index_data[iv];
        mesh_contact.vidx[iv] = index;
        mesh_contact.vforce[iv] =
            ChVector<>(mesh_contact_data[3 * iv + 0], mesh_contact_data[3 * iv + 1], mesh_contact_data[3 * iv + 2]);
    }

    if (m_verbose)
        cout << "[Tire node " << m_index << " ] step number: " << step_number
             << "  vertices in contact: " << mesh_contact.nv << endl;

    // Pass the mesh contact forces to the derived class
    ApplyMeshForces(mesh_contact);

    // Send spindle forces to MBS node
    TerrainForce spindle_force;
    LoadSpindleForce(spindle_force);
    double force_data[] = {spindle_force.force.x(),  spindle_force.force.y(),  spindle_force.force.z(),
                           spindle_force.moment.x(), spindle_force.moment.y(), spindle_force.moment.z()};
    MPI_Send(force_data, 6, MPI_DOUBLE, MBS_NODE_RANK, step_number, MPI_COMM_WORLD);

    delete[] vert_data;
    delete[] index_data;
    delete[] mesh_contact_data;
}

void ChVehicleCosimTireNode::OutputData(int frame) {
    OnOutputData(frame);
}

}  // namespace vehicle
}  // namespace chrono
