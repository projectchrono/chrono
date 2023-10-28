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

#include "chrono/physics/ChLoadContainer.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#ifdef CHRONO_IRRLICHT
    #include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
#endif
#ifdef CHRONO_VSG
    #include "chrono_vsg/ChVisualSystemVSG.h"
#endif

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

    virtual void Update(double time) override {
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

ChVehicleCosimViperNode::ChVehicleCosimViperNode() : ChVehicleCosimWheeledMBSNode() {
    m_viper = chrono_types::make_shared<Viper>(m_system);
}

ChVehicleCosimViperNode::~ChVehicleCosimViperNode() {}

void ChVehicleCosimViperNode::InitializeMBS(const ChVector2<>& terrain_size, double terrain_height) {
    // Initialize vehicle
    ChFrame<> init_pos(m_init_loc + ChVector<>(0, 0, terrain_height), Q_from_AngZ(m_init_yaw));

    m_viper->SetDriver(m_driver);
    m_viper->SetWheelVisualization(false);
    m_viper->Initialize(init_pos);

    // Calculate load on each spindle (excluding the wheels)
    assert(4 == (int)m_num_tire_nodes);

    auto total_mass = m_viper->GetRoverMass() - 4 * m_viper->GetWheelMass();
    for (int is = 0; is < 4; is++) {
        m_spindle_vertical_loads.push_back(total_mass / 4);
    }

    // Create ChLoad objects to apply terrain forces on spindles
    auto load_container = chrono_types::make_shared<ChLoadContainer>();
    m_system->Add(load_container);

    for (int is = 0; is < 4; is++) {
        auto spindle = m_viper->GetWheel(wheel_id(is))->GetBody();
        auto force = chrono_types::make_shared<ChLoadBodyForce>(spindle, VNULL, false, VNULL, false);
        m_spindle_terrain_forces.push_back(force);
        load_container->Add(force);
        auto torque = chrono_types::make_shared<ChLoadBodyTorque>(spindle, VNULL, false);
        m_spindle_terrain_torques.push_back(torque);
        load_container->Add(torque);
    }

    // Initialize run-time visualization
    if (m_renderRT) {
#if defined(CHRONO_VSG)
        auto vsys_vsg = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();
        vsys_vsg->AttachSystem(m_system);
        vsys_vsg->SetWindowTitle("Viper Rover Node");
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
        vsys_irr->SetWindowTitle("Viper Rover Node");
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

void ChVehicleCosimViperNode::ApplyTireInfo(const std::vector<ChVector<>>& tire_info) {
    //// TODO
}

// -----------------------------------------------------------------------------

std::shared_ptr<ChBody> ChVehicleCosimViperNode::GetSpindleBody(unsigned int i) const {
    return m_viper->GetWheel(wheel_id(i))->GetBody();
}

double ChVehicleCosimViperNode::GetSpindleLoad(unsigned int i) const {
    return m_spindle_vertical_loads[i];
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

void ChVehicleCosimViperNode::PreAdvance(double step_size) {
    m_viper->Update();
}

void ChVehicleCosimViperNode::ApplySpindleForce(unsigned int i, const TerrainForce& spindle_force) {
    m_spindle_terrain_forces[i]->SetForce(spindle_force.force, false);
    m_spindle_terrain_forces[i]->SetApplicationPoint(spindle_force.point, false);
    m_spindle_terrain_torques[i]->SetTorque(spindle_force.moment, false);
}

// -----------------------------------------------------------------------------

void ChVehicleCosimViperNode::OnRender() {
    if (!m_vsys)
        return;
    if (!m_vsys->Run())
        MPI_Abort(MPI_COMM_WORLD, 1);

    if (m_track) {
        m_vsys->UpdateCamera(m_cam_pos, m_viper->GetChassisPos());
    }

    m_vsys->BeginScene();
    m_vsys->Render();
    m_vsys->EndScene();
}

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
