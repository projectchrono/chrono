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
// Definition of the RIG NODE.
//
// The global reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include <fstream>
#include <algorithm>
#include <set>
#include <vector>
#include <mpi.h>

#include "chrono/ChConfig.h"
#include "chrono/physics/ChLinkLock.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/solver/ChIterativeSolver.h"
#include "chrono/solver/ChDirectSolverLS.h"
#include "chrono/timestepper/ChState.h"
#include "chrono/utils/ChUtilsCreators.h"

#include "chrono/fea/ChLoadContactSurfaceMesh.h"
#include "chrono/fea/ChElementShellANCF.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#ifdef CHRONO_PARDISO_MKL
    #include "chrono_pardisomkl/ChSolverPardisoMKL.h"
#endif

////#ifdef CHRONO_MUMPS
////#include "chrono_mumps/ChSolverMumps.h"
////#endif

#include "chrono_vehicle/cosim/ChVehicleCosimRigNode.h"

#include "chrono_thirdparty/rapidjson/filereadstream.h"
#include "chrono_thirdparty/rapidjson/istreamwrapper.h"

using std::cout;
using std::endl;

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// =============================================================================

class ChFunction_SlipAngle : public chrono::ChFunction {
  public:
    ChFunction_SlipAngle(double max_angle) : m_max_angle(max_angle) {}

    virtual ChFunction_SlipAngle* Clone() const override { return new ChFunction_SlipAngle(m_max_angle); }

    virtual double Get_y(double t) const override {
        // Ramp for 1 second then stay at prescribed value
        double delay = 0.2;
        if (t <= delay)
            return 0;
        double t1 = t - delay;
        if (t1 < 1)
            return -m_max_angle * t1;

        return -m_max_angle;
    }

  private:
    double m_max_angle;
};

// =============================================================================

// Dummy wheel subsystem (needed to attach a Chrono::Vehicle tire)
class RigNodeWheel : public ChWheel {
  public:
    RigNodeWheel() : ChWheel("rig_wheel") {}
    virtual double GetMass() const override { return 0; }
    virtual ChVector<> GetInertia() const override { return ChVector<>(0); }
    virtual double GetRadius() const override { return 1; }
    virtual double GetWidth() const override { return 1; }
};

// =============================================================================

std::string ChVehicleCosimRigNode::GetActuationTypeAsString(ActuationType type) {
    switch (type) {
        case ActuationType::SET_LIN_VEL:
            return "SET_LIN_VEL";
        case ActuationType::SET_ANG_VEL:
            return "SET_ANG_VEL";
        default:
            return "UNKNOWN";
    }
}

ChVehicleCosimRigNode::ActuationType ChVehicleCosimRigNode::GetActuationTypeFromString(const std::string& type) {
    if (type == "SET_LIN_VEL")
        return ActuationType::SET_LIN_VEL;
    if (type == "SET_ANG_VEL")
        return ActuationType::SET_ANG_VEL;

    return ActuationType::UNKNOWN;
}

std::string ChVehicleCosimRigNode::GetTireTypeAsString(TireType type) {
    switch (type) {
        case TireType::RIGID:
            return "RIGID";
        case TireType::FLEXIBLE:
            return "FLEXIBLE";
        default:
            return "UNKNOWN";
    }
}

ChVehicleCosimRigNode::TireType ChVehicleCosimRigNode::GetTireTypeFromString(const std::string& type) {
    if (type == "RIGID")
        return TireType::RIGID;
    if (type == "FLEXIBLE")
        return TireType::FLEXIBLE;

    return TireType::UNKNOWN;
}

bool ChVehicleCosimRigNode::ReadSpecfile(const std::string& specfile, Document& d) {
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

ChVehicleCosimRigNode::TireType ChVehicleCosimRigNode::GetTireTypeFromSpecfile(const std::string& specfile) {
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
// Construction of the rig node:
// - create the (sequential) Chrono system and set solver parameters
// -----------------------------------------------------------------------------
ChVehicleCosimRigNode::ChVehicleCosimRigNode(TireType tire_type,
                                             ActuationType act_type,
                                             double base_vel,
                                             double slip)
    : ChVehicleCosimBaseNode("RIG"),
      m_tire_type(tire_type),
      m_act_type(act_type),
      m_base_vel(base_vel),
      m_slip(slip),
      m_toe_angle(0),
      m_lin_vel(0),
      m_ang_vel(0),
      m_dbp_filter(nullptr),
      m_dbp_filter_window(0.1),
      m_dbp(0),
      m_constructed(false) {
    // Default model parameters
    m_total_mass = 100;
    m_tire_pressure = true;

    // Default integrator and solver types
    m_int_type = ChTimestepper::Type::HHT;
#if defined(CHRONO_PARDISO_MKL)
    m_slv_type = ChSolver::Type::PARDISO_MKL;
////#elif defined(CHRONO_MUMPS)
////    m_slv_type = ChSolver::Type::MUMPS;
#else
    m_slv_type == ChSolver::Type::BARZILAIBORWEIN;
#endif

    // Create the (sequential) SMC system
    m_system = new ChSystemSMC;
    m_system->Set_G_acc(ChVector<>(0, 0, m_gacc));

    // Set default number of threads
    m_system->SetNumThreads(1, 1, 1);
}

ChVehicleCosimRigNode::~ChVehicleCosimRigNode() {
    delete m_dbp_filter;
    delete m_system;
}

// -----------------------------------------------------------------------------

void ChVehicleCosimRigNode::SetNumThreads(int num_threads) {
    m_system->SetNumThreads(num_threads, 1, 1);
}

void ChVehicleCosimRigNode::SetIntegratorType(ChTimestepper::Type int_type, ChSolver::Type slv_type) {
    m_int_type = int_type;
    m_slv_type = slv_type;
#ifndef CHRONO_PARDISO_MKL
    if (m_slv_type == ChSolver::Type::PARDISO_MKL)
        m_slv_type = ChSolver::Type::BARZILAIBORWEIN;
#endif
    ////#ifndef CHRONO_MUMPS
    ////    if (m_slv_type == ChSolver::Type::MUMPS)
    ////        m_slv_type = ChSolver::Type::BARZILAIBORWEIN;
    ////#endif
}

void ChVehicleCosimRigNode::SetTireFromSpecfile(const std::string& filename) {
    m_tire_json = filename;
}

void ChVehicleCosimRigNode::EnableTirePressure(bool val) {
    m_tire_pressure = val;
}

// -----------------------------------------------------------------------------

int ChVehicleCosimRigNode::PartnerRank(unsigned int i) {
    if (m_num_tire_nodes == 0)
        return TERRAIN_NODE_RANK;

    return TIRE_NODE_RANK(i);
}

// -----------------------------------------------------------------------------
// Initialization of the rig node:
// - complete system construction
// - receive terrain height and container half-length
// - initialize the mechanism bodies
// - initialize the mechanism joints
// - call the virtual method InitializeTire which does the following:
//   - initialize the tire
//   - set tire mesh data
//   - set tire contact material
//   - set tire mass
// - send tire mesh data (vertices, normals, and triangles)
// - send rig mass
// - send information on tire contact material
// -----------------------------------------------------------------------------
void ChVehicleCosimRigNode::Initialize() {
    // Invoke the base class method to figure out distribution of node types
    ChVehicleCosimBaseNode::Initialize();

    // --------------------------------------
    // Initialize the rig bodies and the tire
    // --------------------------------------

    // Construct the mechanical system
    Construct();

    // Receive initial terrain dimensions: terrain height and container half-length
    double init_dim[3];
    MPI_Status status;
    MPI_Recv(init_dim, 3, MPI_DOUBLE, TERRAIN_NODE_RANK, 0, MPI_COMM_WORLD, &status);

    if (m_verbose) {
        cout << "[Rig node    ] Received initial terrain height = " << init_dim[0] << endl;
        cout << "[Rig node    ] Received terrain half-length    =  " << init_dim[1] << endl;
    }

    double init_height = init_dim[0];
    double half_length = init_dim[1];

    // Calculate initial rig location and set linear velocity of all rig bodies
    double tire_radius = GetTireRadius();
    ChVector<> origin(-half_length + 1.5 * tire_radius, 0, init_height + tire_radius);
    ChVector<> rig_vel(m_lin_vel, 0, 0);

    // Initialize chassis body
    m_chassis->SetPos(origin);
    m_chassis->SetRot(QUNIT);
    m_chassis->SetPos_dt(rig_vel);

    // Initialize the set_toe body
    m_set_toe->SetPos(origin);
    m_set_toe->SetRot(QUNIT);
    m_set_toe->SetPos_dt(rig_vel);

    // Initialize spindle body
    m_spindle->SetPos(origin);
    m_spindle->SetRot(QUNIT);
    m_spindle->SetPos_dt(rig_vel);
    m_spindle->SetWvel_loc(ChVector<>(0, m_ang_vel, 0));

    // Initialize axle body
    m_upright->SetPos(origin);
    m_upright->SetRot(QUNIT);
    m_upright->SetPos_dt(rig_vel);

    // -----------------------------------
    // Initialize the rig mechanism joints
    // -----------------------------------

    // Revolute engine on set_toe (about Z axis)
    m_slip_motor->SetAngleFunction(chrono_types::make_shared<ChFunction_SlipAngle>(m_toe_angle));
    m_slip_motor->Initialize(m_set_toe, m_chassis, ChFrame<>(m_set_toe->GetPos(), QUNIT));

    // Prismatic constraint on the chassis (along X axis)
    m_prism_vel->Initialize(m_ground, m_chassis, ChCoordsys<>(m_chassis->GetPos(), Q_from_AngY(CH_C_PI_2)));

    // Impose velocity actuation on the prismatic joint
    m_lin_actuator->Set_dist_funct(chrono_types::make_shared<ChFunction_Ramp>(0.0, m_lin_vel));
    m_lin_actuator->Initialize(m_ground, m_chassis, false, ChCoordsys<>(m_chassis->GetPos(), QUNIT),
                               ChCoordsys<>(m_chassis->GetPos() + ChVector<>(1, 0, 0), QUNIT));

    // Prismatic constraint on the toe-upright (along Z axis)
    m_prism_axl->Initialize(m_set_toe, m_upright, ChCoordsys<>(m_set_toe->GetPos(), QUNIT));

    // Connect spindle to upright: Impose rotation on the spindle
    m_rev_motor->SetAngleFunction(chrono_types::make_shared<ChFunction_Ramp>(0, -m_ang_vel));
    m_rev_motor->Initialize(m_spindle, m_upright, ChFrame<>(m_spindle->GetPos(), Q_from_AngAxis(CH_C_PI / 2.0, VECT_X)));

    // -----------------------------------
    // Initialize the wheel and tire
    // -----------------------------------

    // Initialize the wheel, arbitrarily assuming LEFT side
    m_wheel->Initialize(m_spindle, LEFT);
    m_wheel->SetVisualizationType(VisualizationType::NONE);

    // Let the derived class initialize the tire and set tire contact information
    InitializeTire();

    // -----------------------------------
    // Send mesh info and contact material
    // -----------------------------------

    unsigned int surf_props[] = {m_mesh_data.nv, m_mesh_data.nn, m_mesh_data.nt};
    MPI_Send(surf_props, 3, MPI_UNSIGNED, TERRAIN_NODE_RANK, 0, MPI_COMM_WORLD);
    if (m_verbose)
        cout << "[Rig node    ] vertices = " << surf_props[0] << "  triangles = " << surf_props[2] << endl;

    double* vert_data = new double[3 * m_mesh_data.nv + 3 * m_mesh_data.nn];
    int* tri_data = new int[3 * m_mesh_data.nt + 3 * m_mesh_data.nt];
    for (unsigned int iv = 0; iv < m_mesh_data.nv; iv++) {
        vert_data[3 * iv + 0] = m_mesh_data.verts[iv].x();
        vert_data[3 * iv + 1] = m_mesh_data.verts[iv].y();
        vert_data[3 * iv + 2] = m_mesh_data.verts[iv].z();
    }
    for (unsigned int in = 0; in < m_mesh_data.nn; in++) {
        vert_data[3 * m_mesh_data.nv + 3 * in + 0] = m_mesh_data.norms[in].x();
        vert_data[3 * m_mesh_data.nv + 3 * in + 1] = m_mesh_data.norms[in].y();
        vert_data[3 * m_mesh_data.nv + 3 * in + 2] = m_mesh_data.norms[in].z();
    }
    for (unsigned int it = 0; it < m_mesh_data.nt; it++) {
        tri_data[6 * it + 0] = m_mesh_data.idx_verts[it].x();
        tri_data[6 * it + 1] = m_mesh_data.idx_verts[it].y();
        tri_data[6 * it + 2] = m_mesh_data.idx_verts[it].z();
        tri_data[6 * it + 3] = m_mesh_data.idx_norms[it].x();
        tri_data[6 * it + 4] = m_mesh_data.idx_norms[it].y();
        tri_data[6 * it + 5] = m_mesh_data.idx_norms[it].z();
    }
    MPI_Send(vert_data, 3 * m_mesh_data.nv + 3 * m_mesh_data.nn, MPI_DOUBLE, PartnerRank(0), 0, MPI_COMM_WORLD);
    MPI_Send(tri_data, 3 * m_mesh_data.nt + 3 * m_mesh_data.nt, MPI_INT, PartnerRank(0), 0, MPI_COMM_WORLD);

    MPI_Send(&m_total_mass, 1, MPI_DOUBLE, PartnerRank(0), 0, MPI_COMM_WORLD);

    float mat_props[8] = {m_contact_mat->GetKfriction(),    m_contact_mat->GetRestitution(),
                          m_contact_mat->GetYoungModulus(), m_contact_mat->GetPoissonRatio(),
                          m_contact_mat->GetKn(),           m_contact_mat->GetGn(),
                          m_contact_mat->GetKt(),           m_contact_mat->GetGt()};
    MPI_Send(mat_props, 8, MPI_FLOAT, PartnerRank(0), 0, MPI_COMM_WORLD);
    if (m_verbose)
        cout << "[Rig node    ] friction = " << mat_props[0] << endl;
}

// -----------------------------------------------------------------------------
// Construct the mechanical system.
// This function is invoked automatically from Initialize.
// - create (but do not initialize) the rig mechanism bodies and joints
// - create (but do not initialize) the tire
// -----------------------------------------------------------------------------
void ChVehicleCosimRigNode::Construct() {
    if (m_constructed)
        return;

    if (m_verbose)
        cout << "[Rig node    ] Actuation: " << GetActuationTypeAsString(m_act_type)  //
             << " base velocity = " << m_base_vel << " slip = " << m_slip << endl;

    // -------------------------------
    // Change solver and integrator
    // -------------------------------

    switch (m_slv_type) {
        case ChSolver::Type::PARDISO_MKL: {
#ifdef CHRONO_PARDISO_MKL
            auto solver = chrono_types::make_shared<ChSolverPardisoMKL>();
            solver->LockSparsityPattern(true);
            m_system->SetSolver(solver);
#endif
            break;
        }
            ////        case ChSolver::Type::MUMPS: {
            ////#ifdef CHRONO_MUMPS
            ////            auto solver = chrono_types::make_shared<ChSolverMumps>();
            ////            solver->LockSparsityPattern(true);
            ////            m_system->SetSolver(solver);
            ////#endif
            ////            break;
            ////        }
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

    switch (m_int_type) {
        case ChTimestepper::Type::HHT:
            m_system->SetTimestepperType(ChTimestepper::Type::HHT);
            m_integrator = std::static_pointer_cast<ChTimestepperHHT>(m_system->GetTimestepper());
            m_integrator->SetAlpha(-0.2);
            m_integrator->SetMaxiters(50);
            m_integrator->SetAbsTolerances(5e-05, 1.8e00);
            m_integrator->SetMode(ChTimestepperHHT::POSITION);
            m_integrator->SetScaling(true);
            m_integrator->SetVerbose(false);
            m_integrator->SetMaxItersSuccess(5);

            break;
    }

    // ---------------------
    // Create wheel and tire
    // ---------------------

    m_wheel = chrono_types::make_shared<RigNodeWheel>();
    ConstructTire();

    // ---------------------
    // Calculate body masses
    // ---------------------

    // Distribute remaining mass (after subtracting tire mass) equally between upright and spindle in order to prevent
    // large mass discrepancies.
    // Notes:
    // - the dummy wheel component is assigned a zero mass (i.e., it does not affect the spindle's mass)
    // - the chassis and set_toe bodies are assigned the same mass, but they are not counted in the total mass, as they
    // are moving only horizontally due to the chassis ground connection.
    double tire_mass = GetTireMass();
    if (m_total_mass - tire_mass < 2)
        m_total_mass = tire_mass + 2;
    double body_mass = (m_total_mass - tire_mass) / 2;

    if (m_verbose) {
        cout << "[Rig node    ] total mass = " << m_total_mass << endl;
        cout << "[Rig node    ] tire mass  = " << tire_mass << endl;
        cout << "[Rig node    ] body mass  = " << body_mass << endl;
    }

    // --------------------------------------------------------
    // Calculate rig linear velocity and wheel angular velocity
    // --------------------------------------------------------

    double tire_radius = GetTireRadius();
    switch (m_act_type) {
        case ActuationType::SET_ANG_VEL:
            m_ang_vel = m_base_vel;
            m_lin_vel = (m_ang_vel * tire_radius) * (1.0 - m_slip);
            break;
        case ActuationType::SET_LIN_VEL:
            m_lin_vel = m_base_vel;
            m_ang_vel = m_lin_vel / (tire_radius * (1.0 - m_slip));
            break;
    }

    if (m_verbose) {
        cout << "[Rig node    ] rig linear velocity    = " << m_lin_vel << endl;
        cout << "[Rig node    ] wheel angular velocity = " << m_ang_vel << endl;
        cout << "[Rig node    ] tire radius            = " << tire_radius << endl;
    }

    // -------------------------------
    // Create the rig mechanism bodies
    // -------------------------------

    ChVector<> chassis_inertia(0.1, 0.1, 0.1);
    ChVector<> set_toe_inertia(0.1, 0.1, 0.1);
    ChVector<> upright_inertia(1, 1, 1);
    ChVector<> spindle_inertia(1, 1, 1);

    // Create ground body.
    m_ground = chrono_types::make_shared<ChBody>();
    m_ground->SetBodyFixed(true);
    m_system->AddBody(m_ground);

    // Create the chassis body.
    m_chassis = chrono_types::make_shared<ChBody>();
    m_chassis->SetMass(body_mass);
    m_chassis->SetInertiaXX(chassis_inertia);
    m_system->AddBody(m_chassis);

    // Create the set toe body.
    m_set_toe = chrono_types::make_shared<ChBody>();
    m_set_toe->SetMass(body_mass);
    m_set_toe->SetInertiaXX(set_toe_inertia);
    m_system->AddBody(m_set_toe);

    // Create the upright body.
    m_upright = chrono_types::make_shared<ChBody>();
    m_upright->SetMass(body_mass);
    m_upright->SetInertiaXX(upright_inertia);
    m_system->AddBody(m_upright);

    // Create the spindle body.
    m_spindle = chrono_types::make_shared<ChBody>();
    m_spindle->SetMass(body_mass);
    m_spindle->SetInertiaXX(spindle_inertia);
    m_system->AddBody(m_spindle);

    // -------------------------------
    // Create the rig mechanism joints
    // -------------------------------

    // Connect chassis to set_toe body through an actuated revolute joint.
    m_slip_motor = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    m_slip_motor->SetName("engine_set_slip");
    m_system->AddLink(m_slip_motor);

    // Prismatic constraint on the toe
    m_prism_vel = chrono_types::make_shared<ChLinkLockPrismatic>();
    m_prism_vel->SetName("Prismatic_chassis_ground");
    m_system->AddLink(m_prism_vel);

    // Impose velocity actuation on the prismatic joint
    m_lin_actuator = chrono_types::make_shared<ChLinkLinActuator>();
    m_lin_actuator->SetName("Prismatic_actuator");
    m_lin_actuator->Set_lin_offset(1);  // Set actuator distance offset
    m_system->AddLink(m_lin_actuator);

    // Prismatic constraint on the toe-axle: Connects chassis to axle
    m_prism_axl = chrono_types::make_shared<ChLinkLockPrismatic>();
    m_prism_axl->SetName("Prismatic_vertical");
    m_system->AddLink(m_prism_axl);

    // Create revolute motor to impose rotation on the spindle
    m_rev_motor = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    m_rev_motor->SetName("Motor_ang_vel");
    m_system->AddLink(m_rev_motor);

    // ---------------------------------
    // Create DBP running average filter
    // ---------------------------------

    int nw = static_cast<int>(std::round(m_dbp_filter_window / m_step_size));
    m_dbp_filter = new utils::ChRunningAverage(nw);

    // ---------------------------------
    // Write file with rig node settings
    // ---------------------------------

    std::ofstream outf;
    outf.open(m_node_out_dir + "/settings.info", std::ios::out);

    outf << "System settings" << endl;
    outf << "   Integration step size = " << m_step_size << endl;
    outf << "Tire specification" << endl;
    outf << "   JSON file: " << m_tire_json << endl;
    outf << "   Pressure enabled? " << (m_tire_pressure ? "YES" : "NO") << endl;
    outf << "   Tire radius = " << GetTireRadius() << endl;
    outf << "   Tire mass   = " << GetTireMass() << endl;
    outf << "Rig body masses" << endl;
    outf << "   total equivalent mass = " << m_total_mass << endl;
    outf << "   individual body mass  = " << body_mass << endl;
    outf << "Actuation" << endl;
    outf << "   Type: " << GetActuationTypeAsString(m_act_type) << endl;
    outf << "   Base velocity          = " << m_base_vel << endl;
    outf << "   Longitudinal slip      = " << m_slip << endl;
    outf << "   Rig linear velocity    = " << m_lin_vel << endl;
    outf << "   Wheel angular velocity = " << m_ang_vel << endl;
    outf << endl;

    // Mark system as constructed.
    m_constructed = true;
}

// -----------------------------------------------------------------------------
// Construct tires
// -----------------------------------------------------------------------------

void ChVehicleCosimRigNodeFlexibleTire::ConstructTire() {
    m_tire = chrono_types::make_shared<ANCFTire>(m_tire_json);
    m_tire->EnablePressure(m_tire_pressure);
    m_tire->EnableContact(true);
    m_tire->EnableRimConnection(true);
    m_tire->SetContactSurfaceType(ChDeformableTire::TRIANGLE_MESH);
}

void ChVehicleCosimRigNodeRigidTire::ConstructTire() {
    m_tire = chrono_types::make_shared<RigidTire>(m_tire_json);
    assert(m_tire->UseContactMesh());
}

// -----------------------------------------------------------------------------
// Initialize tires
// -----------------------------------------------------------------------------

void ChVehicleCosimRigNodeFlexibleTire::InitializeTire() {
    // Initialize the ANCF tire
    m_wheel->SetTire(m_tire);                                       // technically not really needed here
    std::static_pointer_cast<ChTire>(m_tire)->Initialize(m_wheel);  // hack to call protected virtual method

    // Create a mesh load for contact forces and add it to the tire's load container.
    auto contact_surface = std::static_pointer_cast<fea::ChContactSurfaceMesh>(m_tire->GetContactSurface());
    m_contact_load = chrono_types::make_shared<fea::ChLoadContactSurfaceMesh>(contact_surface);
    m_tire->GetLoadContainer()->Add(m_contact_load);

    // Set mesh data (initial configuration, vertex positions in local frame)
    //// TODO: vertex normals?
    m_mesh_data.nv = contact_surface->GetNumVertices();
    m_mesh_data.nn = contact_surface->GetNumVertices();
    m_mesh_data.nt = contact_surface->GetNumTriangles();
    std::vector<ChVector<>> vvel;
    m_mesh_data.verts.resize(m_mesh_data.nv);
    m_mesh_data.norms.resize(m_mesh_data.nn);
    m_contact_load->OutputSimpleMesh(m_mesh_data.verts, vvel, m_mesh_data.idx_verts);
    m_mesh_data.idx_norms = m_mesh_data.idx_verts;
    for (unsigned int iv = 0; iv < m_mesh_data.nv; iv++) {
        m_mesh_data.verts[iv] = m_spindle->TransformPointParentToLocal(m_mesh_data.verts[iv]);
        m_mesh_data.norms[iv] = ChVector<>(0, 0, 1);  //// TODO
    }

    // Tire contact material
    m_contact_mat = m_tire->GetContactMaterial();

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
            m_adjVertices[ie].push_back((unsigned int)iv);
        }
    }
}

void ChVehicleCosimRigNodeRigidTire::InitializeTire() {
    // Initialize the rigid tire
    m_wheel->SetTire(m_tire);                                       // technically not really needed here
    std::static_pointer_cast<ChTire>(m_tire)->Initialize(m_wheel);  // hack to call protected virtual method

    // Set mesh data (vertex positions in local frame)
    m_mesh_data.nv = m_tire->GetNumVertices();
    m_mesh_data.nn = m_tire->GetNumNormals();
    m_mesh_data.nt = m_tire->GetNumTriangles();
    m_mesh_data.verts = m_tire->GetMeshVertices();
    m_mesh_data.norms = m_tire->GetMeshNormals();
    m_mesh_data.idx_verts = m_tire->GetMeshConnectivity();
    m_mesh_data.idx_norms = m_tire->GetMeshNormalIndices();

    // Tire contact material
    m_contact_mat = std::static_pointer_cast<ChMaterialSurfaceSMC>(m_tire->GetContactMaterial());

    // Preprocess the tire mesh and store neighbor element information for each vertex.
    // Calculate mesh triangle areas.
    m_adjElements.resize(m_mesh_data.nv);
    std::vector<double> triArea(m_mesh_data.nt);
    for (unsigned int ie = 0; ie < m_mesh_data.nt; ie++) {
        int iv1 = m_mesh_data.idx_verts[ie].x();
        int iv2 = m_mesh_data.idx_verts[ie].y();
        int iv3 = m_mesh_data.idx_verts[ie].z();
        ChVector<> v1 = m_mesh_data.verts[iv1];
        ChVector<> v2 = m_mesh_data.verts[iv2];
        ChVector<> v3 = m_mesh_data.verts[iv3];
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
}

// -----------------------------------------------------------------------------
// Synchronization of the rig node:
// - extract and send tire mesh vertex states
// - receive and apply vertex contact forces
// -----------------------------------------------------------------------------

void ChVehicleCosimRigNodeFlexibleTire::Synchronize(int step_number, double time) {
    // Extract tire mesh vertex locations and velocites.
    std::vector<ChVector<int>> triangles;
    m_contact_load->OutputSimpleMesh(m_mesh_state.vpos, m_mesh_state.vvel, triangles);

    // Display information on lowest mesh node and lowest contact vertex.
    if (m_verbose) {
        PrintLowestNode();
        PrintLowestVertex(m_mesh_state.vpos, m_mesh_state.vvel);
    }

    // Send tire mesh vertex locations and velocities to the terrain node
    double* vert_data = new double[2 * 3 * m_mesh_data.nv];
    for (unsigned int iv = 0; iv < m_mesh_data.nv; iv++) {
        vert_data[3 * iv + 0] = m_mesh_state.vpos[iv].x();
        vert_data[3 * iv + 1] = m_mesh_state.vpos[iv].y();
        vert_data[3 * iv + 2] = m_mesh_state.vpos[iv].z();
    }
    for (unsigned int iv = 0; iv < m_mesh_data.nv; iv++) {
        vert_data[3 * m_mesh_data.nv + 3 * iv + 0] = m_mesh_state.vvel[iv].x();
        vert_data[3 * m_mesh_data.nv + 3 * iv + 1] = m_mesh_state.vvel[iv].y();
        vert_data[3 * m_mesh_data.nv + 3 * iv + 2] = m_mesh_state.vvel[iv].z();
    }
    MPI_Send(vert_data, 2 * 3 * m_mesh_data.nv, MPI_DOUBLE, PartnerRank(0), step_number, MPI_COMM_WORLD);

    // Receive terrain forces.
    // Note that we use MPI_Probe to figure out the number of indices and forces received.
    MPI_Status status;
    MPI_Probe(TERRAIN_NODE_RANK, step_number, MPI_COMM_WORLD, &status);
    MPI_Get_count(&status, MPI_INT, &m_mesh_contact.nv);
    int* index_data = new int[m_mesh_contact.nv];
    double* force_data = new double[3 * m_mesh_contact.nv];
    MPI_Recv(index_data, m_mesh_contact.nv, MPI_INT, PartnerRank(0), step_number, MPI_COMM_WORLD, &status);
    MPI_Recv(force_data, 3 * m_mesh_contact.nv, MPI_DOUBLE, PartnerRank(0), step_number, MPI_COMM_WORLD, &status);

    if (m_verbose)
        cout << "[Rig node    ] step number: " << step_number << "  vertices in contact: " << m_mesh_contact.nv << endl;

    // Repack data and apply forces to the mesh vertices
    m_mesh_contact.vidx.resize(m_mesh_contact.nv);
    m_mesh_contact.vforce.resize(m_mesh_contact.nv);
    for (int iv = 0; iv < m_mesh_contact.nv; iv++) {
        int index = index_data[iv];
        m_mesh_contact.vidx[iv] = index;
        m_mesh_contact.vforce[iv] = ChVector<>(force_data[3 * iv + 0], force_data[3 * iv + 1], force_data[3 * iv + 2]);
    }
    m_contact_load->InputSimpleForces(m_mesh_contact.vforce, m_mesh_contact.vidx);

    PrintContactData(m_mesh_contact.vforce, m_mesh_contact.vidx);

    delete[] vert_data;
    delete[] index_data;
    delete[] force_data;
}

void ChVehicleCosimRigNodeRigidTire::Synchronize(int step_number, double time) {
    // Send wheel state to the terrain node
    m_wheel_state = m_wheel->GetState();
    double state_data[] = {
        m_wheel_state.pos.x(),     m_wheel_state.pos.y(),     m_wheel_state.pos.z(),                              //
        m_wheel_state.rot.e0(),    m_wheel_state.rot.e1(),    m_wheel_state.rot.e2(),    m_wheel_state.rot.e3(),  //
        m_wheel_state.lin_vel.x(), m_wheel_state.lin_vel.y(), m_wheel_state.lin_vel.z(),                          //
        m_wheel_state.ang_vel.x(), m_wheel_state.ang_vel.y(), m_wheel_state.ang_vel.z(),                          //
        m_wheel_state.omega                                                                                       //
    };
    MPI_Send(state_data, 14, MPI_DOUBLE, PartnerRank(0), step_number, MPI_COMM_WORLD);

    // Receive terrain force.
    // Note that we assume this is the resultant wrench at the wheel origin (expressed in absolute frame).
    double force_data[6];
    MPI_Status status;
    MPI_Recv(force_data, 6, MPI_DOUBLE, PartnerRank(0), step_number, MPI_COMM_WORLD, &status);
    m_wheel_contact.point = m_wheel->GetPos();
    m_wheel_contact.force = ChVector<>(force_data[0], force_data[1], force_data[2]);
    m_wheel_contact.moment = ChVector<>(force_data[3], force_data[4], force_data[5]);

    m_spindle->Empty_forces_accumulators();
    m_spindle->Accumulate_force(m_wheel_contact.force, m_wheel_contact.point, false);
    m_spindle->Accumulate_torque(m_wheel_contact.moment, false);
}

// -----------------------------------------------------------------------------
// Advance simulation of the rig node by the specified duration
// -----------------------------------------------------------------------------

void ChVehicleCosimRigNodeFlexibleTire::Advance(double step_size) {
    m_timer.reset();
    m_timer.start();
    double t = 0;
    while (t < step_size) {
        m_dbp = m_dbp_filter->Add(m_lin_actuator->Get_react_force().x());
        m_tire->GetMesh()->ResetCounters();
        m_tire->GetMesh()->ResetTimers();
        double h = std::min<>(m_step_size, step_size - t);
        m_system->DoStepDynamics(h);
        t += h;
    }
    m_timer.stop();
    m_cum_sim_time += m_timer();
}

void ChVehicleCosimRigNodeRigidTire::Advance(double step_size) {
    m_timer.reset();
    m_timer.start();
    double t = 0;
    while (t < step_size) {
        m_dbp = m_dbp_filter->Add(m_lin_actuator->Get_react_force().x());
        double h = std::min<>(m_step_size, step_size - t);
        m_system->DoStepDynamics(h);
        t += h;
    }
    m_timer.stop();
    m_cum_sim_time += m_timer();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChVehicleCosimRigNode::OutputData(int frame) {
    // Append to results output file
    if (m_outf.is_open()) {
        std::string del("  ");

        const ChVector<>& chassis_pos = m_chassis->GetPos();
        const ChVector<>& spindle_pos = m_spindle->GetPos();
        const ChVector<>& spindle_vel = m_spindle->GetPos_dt();
        const ChVector<>& spindle_angvel = m_spindle->GetWvel_loc();

        const ChVector<>& rfrc_prsm = m_prism_vel->Get_react_force();
        const ChVector<>& rtrq_prsm = m_prism_vel->Get_react_torque();
        const ChVector<>& rfrc_act = m_lin_actuator->Get_react_force();  // drawbar pull
        const ChVector<>& rtrq_act = m_lin_actuator->Get_react_torque();
        const ChVector<>& rfrc_motor = m_rev_motor->Get_react_force();
        const ChVector<>& rtrq_motor = m_rev_motor->GetMotorTorque();

        m_outf << m_system->GetChTime() << del;
        // Body states
        m_outf << spindle_pos.x() << del << spindle_pos.y() << del << spindle_pos.z() << del;
        m_outf << spindle_vel.x() << del << spindle_vel.y() << del << spindle_vel.z() << del;
        m_outf << spindle_angvel.x() << del << spindle_angvel.y() << del << spindle_angvel.z() << del;
        m_outf << chassis_pos.x() << del << chassis_pos.y() << del << chassis_pos.z() << del;
        // Filtered actuator force X component (drawbar pull)
        m_outf << m_dbp << del;
        // Joint reactions
        m_outf << rfrc_prsm.x() << del << rfrc_prsm.y() << del << rfrc_prsm.z() << del;
        m_outf << rtrq_prsm.x() << del << rtrq_prsm.y() << del << rtrq_prsm.z() << del;
        m_outf << rfrc_act.x() << del << rfrc_act.y() << del << rfrc_act.z() << del;
        m_outf << rtrq_act.x() << del << rtrq_act.y() << del << rtrq_act.z() << del;
        m_outf << rfrc_motor.x() << del << rfrc_motor.y() << del << rfrc_motor.z() << del;
        m_outf << rtrq_motor.x() << del << rtrq_motor.y() << del << rtrq_motor.z() << del;
        // Solver statistics (for last integration step)
        m_outf << m_system->GetTimerStep() << del << m_system->GetTimerLSsetup() << del << m_system->GetTimerLSsolve()
               << del << m_system->GetTimerUpdate() << del;
        if (m_int_type == ChTimestepper::Type::HHT) {
            m_outf << m_integrator->GetNumIterations() << del << m_integrator->GetNumSetupCalls() << del
                   << m_integrator->GetNumSolveCalls() << del;
        }
        // Tire statistics
        OutputTireData(del);
        m_outf << endl;
    }

    // Create and write frame output file.
    utils::CSV_writer csv(" ");
    csv << m_system->GetChTime() << endl;  // current time
    WriteBodyInformation(csv);             // rig body states
    WriteTireInformation(csv);             // tire-related data

    std::string filename = OutputFilename(m_node_out_dir, "data", "dat", frame + 1, 5);
    csv.write_to_file(filename);

    if (m_verbose)
        cout << "[Rig node    ] write output file ==> " << filename << endl;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChVehicleCosimRigNodeFlexibleTire::OutputTireData(const std::string& del) {
    auto mesh = m_tire->GetMesh();

    m_outf << mesh->GetTimeInternalForces() << del << mesh->GetTimeJacobianLoad();
    m_outf << mesh->GetNumCallsInternalForces() << del << mesh->GetNumCallsJacobianLoad();
}

void ChVehicleCosimRigNodeRigidTire::OutputTireData(const std::string& del) {
    //// TODO
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChVehicleCosimRigNode::WriteBodyInformation(utils::CSV_writer& csv) {
    // Write number of bodies
    csv << "4" << endl;

    // Write body state information
    csv << m_chassis->GetIdentifier() << m_chassis->GetPos() << m_chassis->GetRot() << m_chassis->GetPos_dt()
        << m_chassis->GetRot_dt() << endl;
    csv << m_set_toe->GetIdentifier() << m_set_toe->GetPos() << m_set_toe->GetRot() << m_set_toe->GetPos_dt()
        << m_set_toe->GetRot_dt() << endl;
    csv << m_spindle->GetIdentifier() << m_spindle->GetPos() << m_spindle->GetRot() << m_spindle->GetPos_dt()
        << m_spindle->GetRot_dt() << endl;
    csv << m_upright->GetIdentifier() << m_upright->GetPos() << m_upright->GetRot() << m_upright->GetPos_dt()
        << m_upright->GetRot_dt() << endl;
}

void ChVehicleCosimRigNodeFlexibleTire::WriteTireInformation(utils::CSV_writer& csv) {
    WriteTireStateInformation(csv);
    WriteTireMeshInformation(csv);
    WriteTireContactInformation(csv);
}

void ChVehicleCosimRigNodeRigidTire::WriteTireInformation(utils::CSV_writer& csv) {
    WriteTireStateInformation(csv);
    WriteTireMeshInformation(csv);
    WriteTireContactInformation(csv);
}

void ChVehicleCosimRigNodeFlexibleTire::WriteTireStateInformation(utils::CSV_writer& csv) {
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
    for (int ix = 0; ix < x.size(); ix++)
        csv << x(ix) << endl;
    for (int iv = 0; iv < v.size(); iv++)
        csv << v(iv) << endl;
}

void ChVehicleCosimRigNodeFlexibleTire::WriteTireMeshInformation(utils::CSV_writer& csv) {
    // Extract mesh
    auto mesh = m_tire->GetMesh();

    // Print tire mesh connectivity
    csv << "\n Connectivity " << mesh->GetNelements() << 5 * mesh->GetNelements() << endl;

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
            auto StrainStress = element->EvaluateSectionStrainStress(ChVector<>(0, 0, 0), 0);
            ChVector<> StrainVector = StrainStress.strain;
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

void ChVehicleCosimRigNodeFlexibleTire::WriteTireContactInformation(utils::CSV_writer& csv) {
    // Extract mesh
    auto mesh = m_tire->GetMesh();

    // Write the number of vertices in contact
    csv << m_mesh_contact.vidx.size() << endl;

    // For each vertex in contact, calculate a representative area by averaging
    // the areas of its adjacent elements.
    for (unsigned int iv = 0; iv < m_mesh_contact.vidx.size(); iv++) {
        int in = m_mesh_contact.vidx[iv];
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
        csv << in << m_mesh_state.vpos[in] << m_mesh_contact.vforce[iv] << node->GetD().GetNormalized() << area << endl;
    }
}

void ChVehicleCosimRigNodeRigidTire::WriteTireStateInformation(utils::CSV_writer& csv) {
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

void ChVehicleCosimRigNodeRigidTire::WriteTireMeshInformation(utils::CSV_writer& csv) {
    // Print tire mesh connectivity
    csv << "\n Connectivity " << m_tire->GetNumTriangles() << endl;

    const std::vector<ChVector<int>>& triangles = m_tire->GetMeshConnectivity();
    for (unsigned int ie = 0; ie < m_tire->GetNumTriangles(); ie++) {
        csv << triangles[ie] << endl;
    }
}

void ChVehicleCosimRigNodeRigidTire::WriteTireContactInformation(utils::CSV_writer& csv) {
    //// TODO
    /*
    // Write the number of vertices in contact
    csv << m_mesh_contact.vidx.size() << endl;

    // For each vertex in contact, output vertex index, contact force, normal, and area.
    const std::vector<ChVector<>>& normals = m_tire->GetMeshNormals();
    for (unsigned int iv = 0; iv < m_mesh_contact.vidx.size(); iv++) {
        int in = m_mesh_contact.vidx[iv];
        ChVector<> nrm = m_spindle->TransformDirectionLocalToParent(normals[in]);
        csv << in << m_mesh_state.vpos[in] << m_mesh_contact.vforce[iv] << nrm << m_vertexArea[in] << endl;
    }
    */
}

// -----------------------------------------------------------------------------

void ChVehicleCosimRigNodeFlexibleTire::PrintLowestVertex(const std::vector<ChVector<>>& vert_pos,
                                                          const std::vector<ChVector<>>& vert_vel) {
    auto lowest = std::min_element(vert_pos.begin(), vert_pos.end(),
                                   [](const ChVector<>& a, const ChVector<>& b) { return a.z() < b.z(); });
    auto index = lowest - vert_pos.begin();
    const ChVector<>& vel = vert_vel[index];
    cout << "[Rig node    ] lowest vertex:  index = " << index << "  height = " << (*lowest).z()
         << "  velocity = " << vel.x() << "  " << vel.y() << "  " << vel.z() << endl;
}

void ChVehicleCosimRigNodeFlexibleTire::PrintContactData(const std::vector<ChVector<>>& forces,
                                                         const std::vector<int>& indices) {
    cout << "[Rig node    ] contact forces" << endl;
    for (int i = 0; i < indices.size(); i++) {
        cout << "  id = " << indices[i] << "  force = " << forces[i].x() << "  " << forces[i].y() << "  "
             << forces[i].z() << endl;
    }
}

void ChVehicleCosimRigNodeFlexibleTire::PrintLowestNode() {
    // Unfortunately, we do not have access to the node container of a mesh,
    // so we cannot use some nice algorithm here...
    unsigned int num_nodes = m_tire->GetMesh()->GetNnodes();
    unsigned int index = 0;
    double zmin = 1e10;
    for (unsigned int i = 0; i < num_nodes; ++i) {
        // Ugly casting here. (Note also that we need dynamic downcasting, due to the virtual base)
        auto node = std::dynamic_pointer_cast<fea::ChNodeFEAxyz>(m_tire->GetMesh()->GetNode(i));
        if (node->GetPos().z() < zmin) {
            zmin = node->GetPos().z();
            index = i;
        }
    }

    ChVector<> vel = std::dynamic_pointer_cast<fea::ChNodeFEAxyz>(m_tire->GetMesh()->GetNode(index))->GetPos_dt();
    cout << "[Rig node    ] lowest node:    index = " << index << "  height = " << zmin << "  velocity = " << vel.x()
         << "  " << vel.y() << "  " << vel.z() << endl;
}

}  // end namespace vehicle
}  // end namespace chrono
