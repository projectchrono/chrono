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
// Mechanism for a single-wheel testing rig co-simulated with a tire and a
// terrain system.
//
// The global reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include <fstream>
#include <algorithm>
#include <set>
#include <vector>

#include "chrono/ChConfig.h"
#include "chrono/utils/ChUtilsCreators.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_vehicle/cosim/mbs/ChVehicleCosimRigNode.h"

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

ChVehicleCosimRigNode::ChVehicleCosimRigNode(ActuationType act_type, double base_vel, double slip)
    : ChVehicleCosimMBSNode(),
      m_act_type(act_type),
      m_base_vel(base_vel),
      m_total_mass(100),
      m_slip(slip),
      m_toe_angle(0),
      m_lin_vel(0),
      m_ang_vel(0),
      m_dbp_filter(nullptr),
      m_dbp_filter_window(0.1),
      m_dbp(0) {}

ChVehicleCosimRigNode::~ChVehicleCosimRigNode() {
    delete m_dbp_filter;
}

// -----------------------------------------------------------------------------

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

// -----------------------------------------------------------------------------

void ChVehicleCosimRigNode::InitializeMBS(const std::vector<ChVector<>>& tire_info,
                                          const ChVector2<>& terrain_size,
                                          double terrain_height) {
    if (m_verbose)
        cout << "[Rig node    ] Actuation: " << GetActuationTypeAsString(m_act_type)  //
             << " base velocity = " << m_base_vel << " slip = " << m_slip << endl;

    assert(m_num_tire_nodes == 1);
    assert(tire_info.size() == 1);
    double tire_mass = tire_info[0].x();
    double tire_radius = tire_info[0].y();
    ////double tire_width = tire_info[0].z();

    // Calculate initial rig location and set linear velocity of all rig bodies
    ChVector<> origin(-terrain_size.x() / 2 + 1.5 * tire_radius, 0, terrain_height + tire_radius);
    ChVector<> rig_vel(m_lin_vel, 0, 0);

    // Calculate body masses.
    // Distribute remaining mass (after subtracting tire mass) equally between upright and spindle in order to prevent
    // large mass discrepancies.
    // Notes:
    // - the dummy wheel component is assigned a zero mass (i.e., it does not affect the spindle's mass)
    // - the chassis and set_toe bodies are assigned the same mass, but they are not counted in the total mass, as they
    // are moving only horizontally due to the chassis ground connection.
    if (m_total_mass - tire_mass < 2)
        m_total_mass = tire_mass + 2;
    double body_mass = (m_total_mass - tire_mass) / 2;

    if (m_verbose) {
        cout << "[Rig node    ] total mass = " << m_total_mass << endl;
        cout << "[Rig node    ] tire mass  = " << tire_mass << endl;
        cout << "[Rig node    ] body mass  = " << body_mass << endl;
    }

    // Calculate rig linear velocity and wheel angular velocity
    switch (m_act_type) {
        case ActuationType::SET_ANG_VEL:
            m_ang_vel = m_base_vel;
            m_lin_vel = (m_ang_vel * tire_radius) * (1.0 - m_slip);
            break;
        case ActuationType::SET_LIN_VEL:
            m_lin_vel = m_base_vel;
            m_ang_vel = m_lin_vel / (tire_radius * (1.0 - m_slip));
            break;
        default:
            m_ang_vel = 0;
            m_lin_vel = 0;
            break;
    }

    if (m_verbose) {
        cout << "[Rig node    ] rig linear velocity    = " << m_lin_vel << endl;
        cout << "[Rig node    ] wheel angular velocity = " << m_ang_vel << endl;
        cout << "[Rig node    ] tire radius            = " << tire_radius << endl;
    }

    // Construct the mechanical system
    ChVector<> chassis_inertia(0.1, 0.1, 0.1);
    ChVector<> set_toe_inertia(0.1, 0.1, 0.1);
    ChVector<> upright_inertia(1, 1, 1);
    ChVector<> spindle_inertia(1, 1, 1);

    // Create ground body
    m_ground = chrono_types::make_shared<ChBody>();
    m_ground->SetBodyFixed(true);
    m_system->AddBody(m_ground);

    // Create the chassis body
    m_chassis = chrono_types::make_shared<ChBody>();
    m_chassis->SetMass(body_mass);
    m_chassis->SetInertiaXX(chassis_inertia);
    m_chassis->SetPos(origin);
    m_chassis->SetRot(QUNIT);
    m_chassis->SetPos_dt(rig_vel);
    m_system->AddBody(m_chassis);

    // Create the set toe body
    m_set_toe = chrono_types::make_shared<ChBody>();
    m_set_toe->SetMass(body_mass);
    m_set_toe->SetInertiaXX(set_toe_inertia);
    m_set_toe->SetPos(origin);
    m_set_toe->SetRot(QUNIT);
    m_set_toe->SetPos_dt(rig_vel);
    m_system->AddBody(m_set_toe);

    // Create the upright body
    m_upright = chrono_types::make_shared<ChBody>();
    m_upright->SetMass(body_mass);
    m_upright->SetInertiaXX(upright_inertia);
    m_upright->SetPos(origin);
    m_upright->SetRot(QUNIT);
    m_upright->SetPos_dt(rig_vel);
    m_system->AddBody(m_upright);

    // Create the spindle body
    m_spindle = chrono_types::make_shared<ChBody>();
    m_spindle->SetMass(body_mass);
    m_spindle->SetInertiaXX(spindle_inertia);
    m_spindle->SetPos(origin);
    m_spindle->SetRot(QUNIT);
    m_spindle->SetPos_dt(rig_vel);
    m_spindle->SetWvel_loc(ChVector<>(0, m_ang_vel, 0));
    m_system->AddBody(m_spindle);

    // Connect chassis to set_toe body through an actuated revolute joint.
    m_slip_motor = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    m_slip_motor->SetName("engine_set_slip");
    m_slip_motor->SetAngleFunction(chrono_types::make_shared<ChFunction_SlipAngle>(m_toe_angle));
    m_slip_motor->Initialize(m_set_toe, m_chassis, ChFrame<>(m_set_toe->GetPos(), QUNIT));
    m_system->AddLink(m_slip_motor);

    // Prismatic constraint on the toe
    m_prism_vel = chrono_types::make_shared<ChLinkLockPrismatic>();
    m_prism_vel->SetName("Prismatic_chassis_ground");
    m_prism_vel->Initialize(m_ground, m_chassis, ChCoordsys<>(m_chassis->GetPos(), Q_from_AngY(CH_C_PI_2)));
    m_system->AddLink(m_prism_vel);

    // Impose velocity actuation on the prismatic joint
    m_lin_actuator = chrono_types::make_shared<ChLinkLinActuator>();
    m_lin_actuator->SetName("Prismatic_actuator");
    m_lin_actuator->Set_lin_offset(1);  // Set actuator distance offset
    m_lin_actuator->Set_dist_funct(chrono_types::make_shared<ChFunction_Ramp>(0.0, m_lin_vel));
    m_lin_actuator->Initialize(m_ground, m_chassis, false, ChCoordsys<>(m_chassis->GetPos(), QUNIT),
                               ChCoordsys<>(m_chassis->GetPos() + ChVector<>(1, 0, 0), QUNIT));
    m_system->AddLink(m_lin_actuator);

    // Prismatic constraint on the toe-axle: Connects chassis to axle
    m_prism_axl = chrono_types::make_shared<ChLinkLockPrismatic>();
    m_prism_axl->SetName("Prismatic_vertical");
    m_prism_axl->Initialize(m_set_toe, m_upright, ChCoordsys<>(m_set_toe->GetPos(), QUNIT));
    m_system->AddLink(m_prism_axl);

    // Create revolute motor to impose rotation on the spindle
    m_rev_motor = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    m_rev_motor->SetName("Motor_ang_vel");
    m_rev_motor->SetAngleFunction(chrono_types::make_shared<ChFunction_Ramp>(0, -m_ang_vel));
    m_rev_motor->Initialize(m_spindle, m_upright,
                            ChFrame<>(m_spindle->GetPos(), Q_from_AngAxis(CH_C_PI / 2.0, VECT_X)));
    m_system->AddLink(m_rev_motor);

    // Create DBP running average filter
    int nw = static_cast<int>(std::round(m_dbp_filter_window / m_step_size));
    m_dbp_filter = new utils::ChRunningAverage(nw);

    // Write file with rig node settings
    std::ofstream outf;
    outf.open(m_node_out_dir + "/settings.info", std::ios::out);

    outf << "System settings" << endl;
    outf << "   Integration step size = " << m_step_size << endl;
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
}

// -----------------------------------------------------------------------------

void ChVehicleCosimRigNode::ApplySpindleForce(unsigned int i, const TerrainForce& spindle_force) {
    assert(i == 0);

    m_spindles[i]->Empty_forces_accumulators();
    m_spindles[i]->Accumulate_force(spindle_force.force, spindle_force.point, false);
    m_spindles[i]->Accumulate_torque(spindle_force.moment, false);
}

BodyState ChVehicleCosimRigNode::GetSpindleState(unsigned int i) const {
    BodyState state;

    state.pos = m_spindles[i]->GetPos();
    state.rot = m_spindles[i]->GetRot();
    state.lin_vel = m_spindles[i]->GetPos_dt();
    state.ang_vel = m_spindles[i]->GetWvel_par();
    
    return state;
}

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
        m_outf << endl;
    }

    // Create and write frame output file.
    utils::CSV_writer csv(" ");
    csv << m_system->GetChTime() << endl;  // current time
    WriteBodyInformation(csv);             // rig body states

    std::string filename = OutputFilename(m_node_out_dir, "data", "dat", frame + 1, 5);
    csv.write_to_file(filename);

    if (m_verbose)
        cout << "[Rig node    ] write output file ==> " << filename << endl;
}

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

}  // end namespace vehicle
}  // end namespace chrono
