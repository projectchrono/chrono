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

ChVehicleCosimViperNode::ChVehicleCosimViperNode()
    : ChVehicleCosimMBSNode(), m_num_spindles(0) {
    m_viper = chrono_types::make_shared<Viper>(m_system);
}

ChVehicleCosimViperNode::~ChVehicleCosimViperNode() {}

// -----------------------------------------------------------------------------

static WheelID wheel_id(unsigned int i) {
    switch (i) {
        default:
        case 0:
            return WheelID::LF;
        case 1:
            return WheelID::RF;
        case 2:
            return WheelID::LB;
        case 3:
            return WheelID::RB;
    }
}

// -----------------------------------------------------------------------------

void ChVehicleCosimViperNode::InitializeMBS(const std::vector<ChVector<>>& tire_info,
                                            const ChVector2<>& terrain_size,
                                            double terrain_height) {
    // Initialize vehicle
    ChFrame<> init_pos(m_init_loc + ChVector<>(0, 0, terrain_height), Q_from_AngZ(m_init_yaw));

    m_viper->SetDriver(m_driver);
    m_viper->Initialize(init_pos);

    // Extract and cache spindle bodies
    m_num_spindles = 4;
    assert(m_num_spindles == (int)m_num_tire_nodes);

    m_spindles.resize(m_num_spindles);
    auto total_mass = m_viper->GetRoverMass();
    for (int is = 0; is < m_num_spindles; is++) {
        m_spindles[is] = m_viper->GetWheel(wheel_id(is))->GetBody();
        m_spindle_loads.push_back(total_mass / m_num_spindles);
    }
}

// -----------------------------------------------------------------------------
int ChVehicleCosimViperNode::GetNumSpindles() const {
    return m_num_spindles;
}

std::shared_ptr<ChBody> ChVehicleCosimViperNode::GetSpindleBody(unsigned int i) const {
    return m_spindles[i];
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

// -----------------------------------------------------------------------------

void ChVehicleCosimViperNode::PreAdvance() {
    m_viper->Update();
}

void ChVehicleCosimViperNode::ApplySpindleForce(unsigned int i, const TerrainForce& spindle_force) {
    m_spindles[i]->Empty_forces_accumulators();
    m_spindles[i]->Accumulate_force(spindle_force.force, spindle_force.point, false);
    m_spindles[i]->Accumulate_torque(spindle_force.moment, false);
}

// -----------------------------------------------------------------------------

void ChVehicleCosimViperNode::OutputData(int frame) {
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

    std::string filename = OutputFilename(m_node_out_dir, "data", "dat", frame + 1, 5);
    csv.write_to_file(filename);

    if (m_verbose)
        cout << "[Vehicle node] write output file ==> " << filename << endl;
}

void ChVehicleCosimViperNode::WriteBodyInformation(utils::CSV_writer& csv) {
    // Write number of bodies
    csv << 1 + m_num_spindles << endl;

    // Write body state information
    auto chassis_pos = m_viper->GetChassis()->GetPos();
    auto chassis_rot = m_viper->GetChassis()->GetRot();
    csv << chassis_pos << chassis_rot << endl;
    for (int is = 0; is < m_num_spindles; is++) {
        csv << m_spindles[is]->GetFrame_REF_to_abs().GetPos() << m_spindles[is]->GetFrame_REF_to_abs().GetRot() << endl;
    }
}

}  // end namespace vehicle
}  // end namespace chrono
