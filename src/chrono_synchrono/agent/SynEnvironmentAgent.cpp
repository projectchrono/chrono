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
// Authors: Yan Xiao
// =============================================================================
//
// Agent class that defines a series of intersections, lanes and traffic lights.
// No visual or physical assets are associated with this agent, it communicates
// with other (vehicle) agents through SPAT and MAP messages.
//
// =============================================================================

#include "chrono_synchrono/agent/SynEnvironmentAgent.h"

#include "chrono_synchrono/flatbuffer/message/SynMAPMessage.h"
#include "chrono_synchrono/flatbuffer/message/SynSPATMessage.h"

namespace chrono {
namespace synchrono {

SynEnvironmentAgent::SynEnvironmentAgent(ChSystem* system) : SynAgent(), m_system(system) {
    m_message = chrono_types::make_shared<SynEnvironmentMessage>();
}

SynEnvironmentAgent::~SynEnvironmentAgent() {}

void SynEnvironmentAgent::InitializeZombie(ChSystem* system) {}

void SynEnvironmentAgent::SynchronizeZombie(std::shared_ptr<SynMessage> message) {}

void SynEnvironmentAgent::Update() {
    auto sim_time = m_system->GetChTime();

    for (auto& lane : m_lane_data) {
        // its time to change
        if (lane.time_to_change <= sim_time) {
            lane.color = FindNextColor(lane.color);
            SetColor(lane.intersection, lane.approach, lane.lane_number, lane.color);
            // proceed to next cycle
            if (lane.current_schedule + 1 >= lane.schedule.size()) {
                lane.current_schedule = 0;
            } else {
                lane.current_schedule++;
            }
            // find the next light changing time
            lane.time_to_change = sim_time + lane.schedule[lane.current_schedule];
        }
    }

    m_message->time = sim_time;
}

// ------------------------------------------------------------------------

int SynEnvironmentAgent::AddLane(int intersection,
                                 int approach,
                                 ApproachLane lane,
                                 LaneColor color,
                                 std::vector<double> behaviour) {
    // Add this lane in EnvironmentAgent, MapMessage, SPATMessage
    int lane_number = m_message->map_message->AddLane(intersection, approach, lane);
    LaneData this_lane = {intersection, approach, lane_number, color, behaviour, 0, behaviour[0]};
    m_lane_data.push_back(this_lane);
    m_message->spat_message->SetColor(intersection, approach, lane_number, color);
    return lane_number;
}

void SynEnvironmentAgent::SetColor(int intersection, int approach, int lane, LaneColor color) {
    m_message->spat_message->SetColor(intersection, approach, lane, color);
}

void SynEnvironmentAgent::SetKey(AgentKey agent_key) {
    m_message->SetSourceKey(agent_key);
    m_message->spat_message->SetSourceKey(agent_key);
    m_message->map_message->SetSourceKey(agent_key);
    m_agent_key = agent_key;
}

// ------------------------------------------------------------------------

LaneColor SynEnvironmentAgent::FindNextColor(LaneColor color) {
    switch (color) {
        case LaneColor::RED:
            return LaneColor::GREEN;
            break;
        case LaneColor::GREEN:
            return LaneColor::YELLOW;
            break;
        case LaneColor::YELLOW:
            return LaneColor::RED;
            break;
        default:
            return LaneColor::RED;
            break;
    }
}

}  // namespace synchrono
}  // namespace chrono