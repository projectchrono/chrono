// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2020 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Aaron Young
// =============================================================================
//
// Class for an agent that wraps a Chrono::Vehicle wheeled vehicle. The
// underlying dynamics are those of a wheeled vehicle, state data consists of
// the position and orientation of the COM and the wheels of the vehicle
//
// =============================================================================

#include "chrono_synchrono/agent/SynWheeledVehicleAgent.h"

#include "chrono_synchrono/utils/SynLog.h"

#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_vehicle/chassis/ChRigidChassis.h"

using namespace chrono::vehicle;

namespace chrono {
namespace synchrono {

SynWheeledVehicleAgent::SynWheeledVehicleAgent(ChWheeledVehicle* vehicle, const std::string& filename)
    : SynAgent(), m_vehicle(vehicle) {
    m_state = chrono_types::make_shared<SynWheeledVehicleStateMessage>(AgentKey(), AgentKey());
    m_description = chrono_types::make_shared<SynWheeledVehicleDescriptionMessage>();

    if (!filename.empty()) {
        SetZombieVisualizationFilesFromJSON(filename);
    } else if (vehicle && std::dynamic_pointer_cast<ChRigidChassis>(vehicle->GetChassis())) {
        SetZombieVisualizationFiles(std::static_pointer_cast<ChRigidChassis>(vehicle->GetChassis())->GetMeshFilename(),
                                    vehicle->GetWheel(0, LEFT)->GetMeshFilename(),
                                    vehicle->GetWheel(0, LEFT)->GetTire()->GetMeshFilename());
        int num_wheels = 0;
        for (auto axle : vehicle->GetAxles())
            num_wheels += static_cast<int>(axle->GetWheels().size());
        SetNumWheels(num_wheels);
    }
}

SynWheeledVehicleAgent::~SynWheeledVehicleAgent() {}

void SynWheeledVehicleAgent::InitializeZombie(ChSystem* system) {
    m_zombie_body = CreateChassisZombieBody(m_description->chassis_vis_file, system);

    // For each wheel, create a tire mesh and wheel mesh.
    // If it is a right side wheel, a 180 degree rotation is made

    for (int i = 0; i < m_description->num_wheels; i++) {
        auto tire_trimesh = CreateMeshZombieComponent(m_description->tire_vis_file);
        auto wheel_trimesh = CreateMeshZombieComponent(m_description->wheel_vis_file);

        //// RADU - pass this transform to AddVisualShape
        ChQuaternion<> rot = (i % 2 == 0) ? QuatFromAngleZ(0) : QuatFromAngleZ(CH_PI);
        wheel_trimesh->GetMesh()->Transform(ChVector3d(), ChMatrix33<>(rot));
        tire_trimesh->GetMesh()->Transform(ChVector3d(), ChMatrix33<>(rot));

        auto wheel = chrono_types::make_shared<ChBodyAuxRef>();
        wheel->AddVisualShape(wheel_trimesh);
        wheel->AddVisualShape(tire_trimesh);
        wheel->EnableCollision(false);
        wheel->SetFixed(true);
        system->Add(wheel);

        m_wheel_list.push_back(wheel);
    }
}

void SynWheeledVehicleAgent::SynchronizeZombie(std::shared_ptr<SynMessage> message) {
    if (auto state = std::dynamic_pointer_cast<SynWheeledVehicleStateMessage>(message)) {
        m_zombie_body->SetFrameRefToAbs(state->chassis.GetFrame());
        for (int i = 0; i < state->wheels.size(); i++)
            m_wheel_list[i]->SetFrameRefToAbs(state->wheels[i].GetFrame());
    }
}

void SynWheeledVehicleAgent::Update() {
    if (!m_vehicle)
        return;

    auto chassis_abs = m_vehicle->GetChassisBody()->GetFrameRefToAbs();
    SynPose chassis(chassis_abs.GetPos(), chassis_abs.GetRot());
    chassis.GetFrame().SetPosDt(chassis_abs.GetPosDt());
    chassis.GetFrame().SetPosDt2(chassis_abs.GetPosDt2());
    chassis.GetFrame().SetRotDt(chassis_abs.GetRotDt());
    chassis.GetFrame().SetRotDt2(chassis_abs.GetRotDt2());

    std::vector<SynPose> wheels;
    for (auto axle : m_vehicle->GetAxles()) {
        for (auto wheel : axle->GetWheels()) {
            auto state = wheel->GetState();
            auto wheel_abs = wheel->GetSpindle()->GetFrameRefToAbs();
            SynPose frame(state.pos, state.rot);
            frame.GetFrame().SetPosDt(wheel_abs.GetPosDt());
            frame.GetFrame().SetPosDt2(wheel_abs.GetPosDt2());
            frame.GetFrame().SetRotDt(wheel_abs.GetRotDt());
            frame.GetFrame().SetRotDt2(wheel_abs.GetRotDt2());
            wheels.emplace_back(frame);
        }
    }

    auto time = m_vehicle->GetSystem()->GetChTime();
    m_state->SetState(time, chassis, wheels);
}

// ------------------------------------------------------------------------

void SynWheeledVehicleAgent::SetZombieVisualizationFilesFromJSON(const std::string& filename) {
    m_description->SetZombieVisualizationFilesFromJSON(filename);
}

void SynWheeledVehicleAgent::SetKey(AgentKey agent_key) {
    m_description->SetSourceKey(agent_key);
    m_state->SetSourceKey(agent_key);
    m_agent_key = agent_key;
}

// ------------------------------------------------------------------------

std::shared_ptr<ChVisualShapeTriangleMesh> SynWheeledVehicleAgent::CreateMeshZombieComponent(
    const std::string& filename) {
    auto trimesh = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    if (!filename.empty()) {
        auto mesh = ChTriangleMeshConnected::CreateFromWavefrontFile(GetVehicleDataFile(filename), true, true);
        trimesh->SetMesh(mesh);
        trimesh->SetName(filesystem::path(filename).stem());
    }
    return trimesh;
}

std::shared_ptr<ChBodyAuxRef> SynWheeledVehicleAgent::CreateChassisZombieBody(const std::string& filename,
                                                                              ChSystem* system) {
    auto trimesh = CreateMeshZombieComponent(filename);

    auto zombie_body = chrono_types::make_shared<ChBodyAuxRef>();
    zombie_body->AddVisualShape(trimesh);
    zombie_body->EnableCollision(false);
    zombie_body->SetFixed(true);
    zombie_body->SetFrameCOMToRef(ChFrame<>({0, 0, -0.2}, {1, 0, 0, 0}));
    system->Add(zombie_body);

    return zombie_body;
}

}  // namespace synchrono
}  // namespace chrono