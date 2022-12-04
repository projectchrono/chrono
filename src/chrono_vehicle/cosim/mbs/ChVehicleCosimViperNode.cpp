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
// Viper rover co-simulated with "tire" nodes and a terrain node.
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

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

#include "chrono_vehicle/cosim/mbs/ChVehicleCosimViperNode.h"

using std::cout;
using std::endl;

namespace chrono {
namespace vehicle {

using namespace viper;

// -----------------------------------------------------------------------------

static ViperWheelID wheel_id(unsigned int i) {
    switch (i) {
        default:
        case 0:
            return ViperWheelID::V_LF;
        case 1:
            return ViperWheelID::V_RF;
        case 2:
            return ViperWheelID::V_LB;
        case 3:
            return ViperWheelID::V_RB;
    }
}

// -----------------------------------------------------------------------------

// Custom Viper driver class used when a DBP rig is attached.
class ViperDBPDriver : public ViperDriver {
  public:
    ViperDBPDriver(std::shared_ptr<ChFunction> dbp_mot_func) : m_func(dbp_mot_func) {}
    ~ViperDBPDriver() {}

    virtual DriveMotorType GetDriveMotorType() const override { return DriveMotorType::SPEED; }

    virtual void Update(double time) {
        double driving = m_func->Get_y(time);
        double steering = 0;
        double lifting = 0;
        for (int i = 0; i < 4; i++) {
            drive_speeds[i] = driving;
            steer_angles[i] = steering;
            lift_angles[i] = lifting;
        }
    }

    std::shared_ptr<ChFunction> m_func;
};

// -----------------------------------------------------------------------------

ChVehicleCosimViperNode::ChVehicleCosimViperNode() : ChVehicleCosimWheeledMBSNode(), m_num_spindles(0) {
    m_viper = chrono_types::make_shared<Viper>(m_system);
}

ChVehicleCosimViperNode::~ChVehicleCosimViperNode() {}

void ChVehicleCosimViperNode::InitializeMBS(const std::vector<ChVector<>>& tire_info,
                                            const ChVector2<>& terrain_size,
                                            double terrain_height) {
    // Initialize vehicle
    ChFrame<> init_pos(m_init_loc + ChVector<>(0, 0, terrain_height), Q_from_AngZ(m_init_yaw));

    m_viper->SetDriver(m_driver);
    m_viper->SetWheelVisualization(false);
    m_viper->Initialize(init_pos);

    // Extract and cache spindle bodies
    m_num_spindles = 4;
    assert(m_num_spindles == (int)m_num_tire_nodes);

    auto total_mass = m_viper->GetRoverMass();
    for (int is = 0; is < m_num_spindles; is++) {
        m_spindle_loads.push_back(total_mass / m_num_spindles);
    }
}

// -----------------------------------------------------------------------------
int ChVehicleCosimViperNode::GetNumSpindles() const {
    return m_num_spindles;
}

std::shared_ptr<ChBody> ChVehicleCosimViperNode::GetSpindleBody(unsigned int i) const {
    return m_viper->GetWheel(wheel_id(i))->GetBody();
}

double ChVehicleCosimViperNode::GetSpindleLoad(unsigned int i) const {
    return m_spindle_loads[i];
}

BodyState ChVehicleCosimViperNode::GetSpindleState(unsigned int i) const {
    BodyState state;

    state.pos = m_viper->GetWheel(wheel_id(i))->GetPos();
    state.rot = m_viper->GetWheel(wheel_id(i))->GetRot();
    state.lin_vel = m_viper->GetWheel(wheel_id(i))->GetLinVel();
    state.ang_vel = m_viper->GetWheel(wheel_id(i))->GetAngVel();

    return state;
}

std::shared_ptr<ChBody> ChVehicleCosimViperNode::GetChassisBody() const {
    return m_viper->GetChassis()->GetBody();
}

void ChVehicleCosimViperNode::OnInitializeDBPRig(std::shared_ptr<ChFunction> func) {
    // Overwrite any driver attached to the underlying Viper rover with a custom driver which imposes zero steering and
    // wheel angular speeds as returned by the provided motor function.
    m_viper->SetDriver(chrono_types::make_shared<ViperDBPDriver>(func));
}

// -----------------------------------------------------------------------------

void ChVehicleCosimViperNode::PreAdvance() {
    m_viper->Update();
}

void ChVehicleCosimViperNode::ApplySpindleForce(unsigned int i, const TerrainForce& spindle_force) {
    auto spindle_body = m_viper->GetWheel(wheel_id(i))->GetBody();
    spindle_body->Empty_forces_accumulators();
    spindle_body->Accumulate_force(spindle_force.force, spindle_force.point, false);
    spindle_body->Accumulate_torque(spindle_force.moment, false);
}

// -----------------------------------------------------------------------------

void ChVehicleCosimViperNode::OnOutputData(int frame) {
    // Append to results output file
    if (m_outf.is_open()) {
        std::string del("  ");

        const ChVector<>& pos = m_viper->GetChassis()->GetPos();

        m_outf << m_system->GetChTime() << del;
        // Body states
        m_outf << pos.x() << del << pos.y() << del << pos.z() << del;
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
    WriteBodyInformation(csv);             // vehicle body states

    std::string filename = OutputFilename(m_node_out_dir + "/simulation", "data", "dat", frame + 1, 5);
    csv.write_to_file(filename);

    if (m_verbose)
        cout << "[Vehicle node] write output file ==> " << filename << endl;
}

void ChVehicleCosimViperNode::WriteBodyInformation(utils::CSV_writer& csv) {
    // Write number of bodies
    csv << 1 + 4 * 4 << endl;

    // Write body state information
    auto ch = m_viper->GetChassis();
    csv << ch->GetPos() << ch->GetRot() << endl;
    for (unsigned int i = 0; i < 4; i++) {
        auto id = wheel_id(i);
        auto ua = m_viper->GetUpperArm(id);
        auto la = m_viper->GetLowerArm(id);
        auto ur = m_viper->GetUpright(id);
        auto wh = m_viper->GetWheel(id);
        csv << ua->GetPos() << ua->GetRot() << endl;
        csv << la->GetPos() << la->GetRot() << endl;
        csv << ur->GetPos() << ur->GetRot() << endl;
        csv << wh->GetPos() << wh->GetRot() << endl;
    }
}

}  // end namespace vehicle
}  // end namespace chrono
