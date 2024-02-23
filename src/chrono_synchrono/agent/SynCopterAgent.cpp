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
// Authors: Simone Benatti
// =============================================================================
//
// Class for an agent that wraps a Chrono::Copter hexacopter. The
// underlying dynamics are those of a copter robot, state data consists of
// the position and orientation of the COM and the propellers of the copter
//
// =============================================================================

#include "chrono_synchrono/agent/SynCopterAgent.h"
#include "chrono_thirdparty/filesystem/path.h"

namespace chrono {
namespace synchrono {

SynCopterAgent::SynCopterAgent(chrono::copter::Copter<6>* copter) : SynAgent(), m_copter(copter) {
    m_state = chrono_types::make_shared<SynCopterStateMessage>();
    m_description = chrono_types::make_shared<SynCopterDescriptionMessage>();
    if (copter) {
        SetZombieVisualizationFiles(copter->GetChassisMeshFilename(), copter->GetPropellerMeshFilename());
        SetNumProps(6);
    }
}

SynCopterAgent::~SynCopterAgent() {}

void SynCopterAgent::InitializeZombie(ChSystem* system) {
    m_zombie_body = CreateChassisZombieBody(m_description->chassis_vis_file, system);

    // For each prop, create a tire prop mesh.

    for (int i = 0; i < m_description->GetNumProps(); i++) {
        auto prop_trimesh = CreateMeshZombieComponent(m_description->propeller_vis_file);

        auto prop = chrono_types::make_shared<ChBody>();
        prop->AddVisualShape(prop_trimesh);
        prop->SetCollide(false);
        prop->SetBodyFixed(true);
        system->Add(prop);

        m_prop_list.push_back(prop);
    }
}

void SynCopterAgent::SynchronizeZombie(std::shared_ptr<SynMessage> message) {
    if (auto state = std::dynamic_pointer_cast<SynCopterStateMessage>(message)) {
        m_zombie_body->SetCsys(state->chassis.GetFrame().GetPos(), state->chassis.GetFrame().GetRot());
        for (int i = 0; i < state->props.size(); i++)
            m_prop_list[i]->SetCsys(state->props[i].GetFrame().GetPos(), state->props[i].GetFrame().GetRot());
    }
}

void SynCopterAgent::Update() {
    if (!m_copter)
        return;

    auto chassis_body = m_copter->GetChassis();
    SynPose chassis_pose(chassis_body->GetPos(), chassis_body->GetRot());
    chassis_pose.GetFrame().SetPosDer(chassis_body->GetPosDer());
    chassis_pose.GetFrame().SetPosDer2(chassis_body->GetPosDer2());
    chassis_pose.GetFrame().SetRotDer(chassis_body->GetRotDer());
    chassis_pose.GetFrame().SetRotDer2(chassis_body->GetRotDer2());

    std::vector<SynPose> props_poses;
    for (auto prop : m_copter->GetProps() ) {
        SynPose frame(prop->GetPos(), prop->GetRot());
        frame.GetFrame().SetPosDer(prop->GetPosDer());
        frame.GetFrame().SetPosDer2(prop->GetPosDer2());
        frame.GetFrame().SetRotDer(prop->GetRotDer());
        frame.GetFrame().SetRotDer2(prop->GetRotDer2());
        props_poses.emplace_back(frame);
    }

    auto time = chassis_body->GetSystem()->GetChTime();
    m_state->SetState(time, chassis_pose, props_poses);
}

// ------------------------------------------------------------------------

void SynCopterAgent::SetKey(AgentKey agent_key) {
    m_description->SetSourceKey(agent_key);
    m_state->SetSourceKey(agent_key);
    m_agent_key = agent_key;
}

// ------------------------------------------------------------------------

std::shared_ptr<ChVisualShapeTriangleMesh> SynCopterAgent::CreateMeshZombieComponent(const std::string& filename) {
    auto trimesh = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    if (!filename.empty()) {
        auto mesh =
            ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile(filename), false, false);
        trimesh->SetMesh(mesh);
        trimesh->SetMutable(false);
        trimesh->SetName(filesystem::path(filename).stem());
    }
    return trimesh;
}

std::shared_ptr<ChBody> SynCopterAgent::CreateChassisZombieBody(const std::string& filename, ChSystem* system) {
    auto trimesh = CreateMeshZombieComponent(filename);

    auto zombie_body = chrono_types::make_shared<ChBody>();
    zombie_body->AddVisualShape(trimesh);
    zombie_body->SetCollide(false);
    zombie_body->SetBodyFixed(true);
    system->Add(zombie_body);

    return zombie_body;
}

}  // namespace synchrono
}  // namespace chrono
