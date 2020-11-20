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
// Authors: 肖言 (Yan Xiao)
// =============================================================================
//
// Agent class that defines a series of intersections, lanes and traffic lights.
// No visual or physical assets are associated with this agent, it communicates
// with other (vehicle) agents through SPAT and MAP messages.
//
// =============================================================================

#include "chrono_synchrono/agent/SynEnvironmentAgent.h"

#include "chrono_synchrono/flatbuffer/message/SynEnvironmentMessage.h"

namespace chrono {
namespace synchrono {

SynEnvironmentAgent::SynEnvironmentAgent(int rank, ChSystem* system)
    : SynAgent(rank, SynAgentType::ENVIRONMENT, system) {
    m_spat_msg = chrono_types::make_shared<SynSPATMessage>(rank);
    m_map_msg = chrono_types::make_shared<SynMAPMessage>(rank);
}

SynEnvironmentAgent::~SynEnvironmentAgent() {}

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

void SynEnvironmentAgent::Advance(double time_of_next_sync) {
    m_current_time = time_of_next_sync;

    for (auto& lane : m_lane_light_data) {
        // its time to change
        if (lane.time_to_change <= m_current_time) {
            lane.color = FindNextColor(lane.color);
            SetColor(lane.intersection, lane.approach, lane.lane_number, lane.color);
            // proceed to next cycle
            if (lane.current_schedule + 1 >= lane.schedule.size()) {
                lane.current_schedule = 0;
            } else {
                lane.current_schedule++;
            }
            // find the next light changing time
            lane.time_to_change = m_current_time + lane.schedule[lane.current_schedule];
        }
    }
    m_map_msg->GetState()->time = m_current_time;
    m_spat_msg->GetState()->time = m_current_time;
}

void SynEnvironmentAgent::InitializeZombie(ChSystem* system) {}
void SynEnvironmentAgent::SynchronizeZombie(SynMessage* message) {}

void SynEnvironmentAgent::GenerateMessagesToSend(std::vector<SynMessage*>& messages) {
    // Important that we push in this order as agents expect to process MAP before SPAT
    messages.push_back(new SynMAPMessage(m_rank, m_map_msg->GetMAPState()));
    messages.push_back(new SynSPATMessage(m_rank, m_spat_msg->GetSPATState()));
}

std::shared_ptr<SynMessageState> SynEnvironmentAgent::GetState() {
    // Environments have no state as such, they have state for SPAT + MAP
    return nullptr;
}

std::shared_ptr<SynAgentMessage> SynEnvironmentAgent::GetMessage() {
    // Environments have no agent message as such, they send SPAT + MAP
    // This is needed to generate the description message
    return chrono_types::make_shared<SynEnvironmentMessage>(m_rank);
}

int SynEnvironmentAgent::AddLane(int intersection,
                                 int approach,
                                 ApproachLane lane,
                                 LaneColor color,
                                 std::vector<double> behaviour) {
    // Add this lane in EnvironmentAgent, MapMessage, SPATMessage
    int lane_number = m_map_msg->AddLane(intersection, approach, lane);
    LaneData this_lane = {intersection, approach, lane_number, color, behaviour, 0, behaviour[0]};
    m_lane_light_data.push_back(this_lane);
    m_spat_msg->setColor(intersection, approach, lane_number, color);
    return lane_number;
}

void SynEnvironmentAgent::SetColor(int intersection, int approach, int lane, LaneColor color) {
    m_spat_msg->setColor(intersection, approach, lane, color);
}

}  // namespace synchrono
}  // namespace chrono
