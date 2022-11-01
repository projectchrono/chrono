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
// Curiosity rover co-simulated with "tire" nodes and a terrain node.
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

#include "chrono_vehicle/cosim/mbs/ChVehicleCosimCuriosityNode.h"

using std::cout;
using std::endl;

namespace chrono {
namespace vehicle {

using namespace curiosity;

// -----------------------------------------------------------------------------

static CuriosityWheelID wheel_id(unsigned int i) {
    switch (i) {
        default:
        case 0:
            return CuriosityWheelID::C_LF;
        case 1:
            return CuriosityWheelID::C_RF;
        case 2:
            return CuriosityWheelID::C_LM;
        case 3:
            return CuriosityWheelID::C_RM;
        case 4:
            return CuriosityWheelID::C_LB;
        case 5:
            return CuriosityWheelID::C_RB;
    }
}

// -----------------------------------------------------------------------------

// Custom Curiosity driver class used when a DBP rig is attached.
class CuriosityDBPDriver : public CuriosityDriver {
  public:
    CuriosityDBPDriver(std::shared_ptr<ChFunction> dbp_mot_func) : m_func(dbp_mot_func) {}
    ~CuriosityDBPDriver() {}

    virtual DriveMotorType GetDriveMotorType() const override { return DriveMotorType::SPEED; }

    virtual void Update(double time) {
        double driving = m_func->Get_y(time);
        double steering = 0;
        for (int i = 0; i < 6; i++) {
            drive_speeds[i] = driving;
        }
        for (int i = 0; i < 4; i++) {
            steer_angles[i] = steering;        
        }
    }

    std::shared_ptr<ChFunction> m_func;
};

// -----------------------------------------------------------------------------

ChVehicleCosimCuriosityNode::ChVehicleCosimCuriosityNode() : ChVehicleCosimWheeledMBSNode() {
    m_curiosity = chrono_types::make_shared<Curiosity>(m_system);
}

ChVehicleCosimCuriosityNode::~ChVehicleCosimCuriosityNode() {}

void ChVehicleCosimCuriosityNode::InitializeMBS(const std::vector<ChVector<>>& tire_info,
                                            const ChVector2<>& terrain_size,
                                            double terrain_height) {
    // Initialize vehicle
    ChFrame<> init_pos(m_init_loc + ChVector<>(0, 0, terrain_height), Q_from_AngZ(m_init_yaw));

    m_curiosity->SetDriver(m_driver);
    m_curiosity->SetWheelVisualization(false);
    m_curiosity->Initialize(init_pos);

    // Extract and cache spindle bodies
    assert(6 == (int)m_num_tire_nodes);

    auto total_mass = m_curiosity->GetRoverMass();
    for (int is = 0; is < 6; is++) {
        m_spindle_loads.push_back(total_mass / 6);
    }
}

// -----------------------------------------------------------------------------

std::shared_ptr<ChBody> ChVehicleCosimCuriosityNode::GetSpindleBody(unsigned int i) const {
    return m_curiosity->GetWheel(wheel_id(i))->GetBody();
}

double ChVehicleCosimCuriosityNode::GetSpindleLoad(unsigned int i) const {
    return m_spindle_loads[i];
}

BodyState ChVehicleCosimCuriosityNode::GetSpindleState(unsigned int i) const {
    BodyState state;

    state.pos = m_curiosity->GetWheel(wheel_id(i))->GetPos();
    state.rot = m_curiosity->GetWheel(wheel_id(i))->GetRot();
    state.lin_vel = m_curiosity->GetWheel(wheel_id(i))->GetLinVel();
    state.ang_vel = m_curiosity->GetWheel(wheel_id(i))->GetAngVel();

    return state;
}

std::shared_ptr<ChBody> ChVehicleCosimCuriosityNode::GetChassisBody() const {
    return m_curiosity->GetChassis()->GetBody();
}

void ChVehicleCosimCuriosityNode::OnInitializeDBPRig(std::shared_ptr<ChFunction> func) {
    // Overwrite any driver attached to the underlying Curiosity rover with a custom driver which imposes zero steering and
    // wheel angular speeds as returned by the provided motor function.
    m_curiosity->SetDriver(chrono_types::make_shared<CuriosityDBPDriver>(func));
}

// -----------------------------------------------------------------------------

void ChVehicleCosimCuriosityNode::PreAdvance() {
    m_curiosity->Update();
}

void ChVehicleCosimCuriosityNode::ApplySpindleForce(unsigned int i, const TerrainForce& spindle_force) {
    auto spindle_body = m_curiosity->GetWheel(wheel_id(i))->GetBody();
    spindle_body->Empty_forces_accumulators();
    spindle_body->Accumulate_force(spindle_force.force, spindle_force.point, false);
    spindle_body->Accumulate_torque(spindle_force.moment, false);
}

// -----------------------------------------------------------------------------

void ChVehicleCosimCuriosityNode::OnOutputData(int frame) {
    // Append to results output file
    if (m_outf.is_open()) {
        std::string del("  ");

        const ChVector<>& pos = m_curiosity->GetChassis()->GetPos();

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

void ChVehicleCosimCuriosityNode::WriteBodyInformation(utils::CSV_writer& csv) {
    // Write number of bodies
    csv << 1 + 6 << endl;

    // Write body state information
    auto ch = m_curiosity->GetChassis();
    csv << ch->GetPos() << ch->GetRot() << endl;
    for (unsigned int i = 0; i < 6; i++) {
        auto id = wheel_id(i);
        auto wh = m_curiosity->GetWheel(id);
        csv << wh->GetPos() << wh->GetRot() << endl;
    }
}

}  // end namespace vehicle
}  // end namespace chrono
