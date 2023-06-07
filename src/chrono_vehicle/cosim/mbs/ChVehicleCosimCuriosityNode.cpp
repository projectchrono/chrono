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
#ifdef CHRONO_IRRLICHT
    #include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
#endif
#ifdef CHRONO_VSG
    #include "chrono_vsg/ChVisualSystemVSG.h"
#endif

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

    virtual void Update(double time) override {
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

void ChVehicleCosimCuriosityNode::InitializeMBS(const ChVector2<>& terrain_size, double terrain_height) {
    // Initialize vehicle
    ChFrame<> init_pos(m_init_loc + ChVector<>(0, 0, terrain_height), Q_from_AngZ(m_init_yaw));

    m_curiosity->SetDriver(m_driver);
    m_curiosity->SetWheelVisualization(false);
    m_curiosity->Initialize(init_pos);

    // Calculate load on each spindle (excluding the wheels)
    assert(6 == (int)m_num_tire_nodes);

    auto total_mass = m_curiosity->GetRoverMass() - 6 * m_curiosity->GetWheelMass();
    for (int is = 0; is < 6; is++) {
        m_spindle_loads.push_back(total_mass / 6);
    }

    // Initialize run-time visualization
    if (m_renderRT) {
#if defined(CHRONO_VSG)
        auto vsys_vsg = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();
        vsys_vsg->AttachSystem(m_system);
        vsys_vsg->SetWindowTitle("Curiosity Rover Node");
        vsys_vsg->SetWindowSize(ChVector2<int>(1280, 720));
        vsys_vsg->SetWindowPosition(ChVector2<int>(100, 300));
        vsys_vsg->AddCamera(m_cam_pos, m_cam_target);
        vsys_vsg->AddGrid(1.0, 1.0, (int)(terrain_size.x() / 1.0), (int)(terrain_size.y() / 1.0), CSYSNORM,
                          ChColor(0.1f, 0.3f, 0.1f));
        vsys_vsg->Initialize();

        m_vsys = vsys_vsg;
#elif defined(CHRONO_IRRLICHT)
        auto vsys_irr = chrono_types::make_shared<irrlicht::ChVisualSystemIrrlicht>();
        vsys_irr->AttachSystem(m_system);
        vsys_irr->SetWindowTitle("Curiosity Rover Node");
        vsys_irr->SetCameraVertical(CameraVerticalDir::Z);
        vsys_irr->SetWindowSize(1280, 720);
        vsys_irr->Initialize();
        vsys_irr->AddLogo();
        vsys_irr->AddSkyBox();
        vsys_irr->AddCamera(m_cam_pos, m_cam_target);
        vsys_irr->AddTypicalLights();

        m_vsys = vsys_irr;

#endif
    }
}

void ChVehicleCosimCuriosityNode::ApplyTireInfo(const std::vector<ChVector<>>& tire_info) {
    //// TODO
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

void ChVehicleCosimCuriosityNode::PreAdvance(double step_size) {
    m_curiosity->Update();
}

void ChVehicleCosimCuriosityNode::ApplySpindleForce(unsigned int i, const TerrainForce& spindle_force) {
    auto spindle_body = m_curiosity->GetWheel(wheel_id(i))->GetBody();
    spindle_body->Empty_forces_accumulators();
    spindle_body->Accumulate_force(spindle_force.force, spindle_force.point, false);
    spindle_body->Accumulate_torque(spindle_force.moment, false);
}

// -----------------------------------------------------------------------------

void ChVehicleCosimCuriosityNode::OnRender() {
    if (!m_vsys)
        return;
    if (!m_vsys->Run())
        MPI_Abort(MPI_COMM_WORLD, 1);

    if (m_track) {
        m_vsys->UpdateCamera(m_cam_pos, m_curiosity->GetChassisPos());
    }

    m_vsys->BeginScene();
    m_vsys->Render();
    m_vsys->EndScene();
}

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
