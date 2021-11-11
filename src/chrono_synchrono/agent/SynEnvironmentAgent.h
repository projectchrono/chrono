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

#ifndef SYN_ENVIRONMENT_AGENT_H
#define SYN_ENVIRONMENT_AGENT_H

#include "chrono_synchrono/SynApi.h"
#include "chrono_synchrono/agent/SynAgent.h"

#include "chrono_synchrono/flatbuffer/message/SynEnvironmentMessage.h"

namespace chrono {
namespace synchrono {

/// @addtogroup synchrono_agent
/// @{

/// Derived agent class. Acts as a traffic light and distributes MAP and/or SPAT data
class SYN_API SynEnvironmentAgent : public SynAgent {
  public:
    /// @brief It defines the traffic light color and schedule for one lane
    struct LaneData {
        int intersection;
        int approach;
        int lane_number;
        LaneColor color;               ///< Current traffic light color of the lane
        std::vector<double> schedule;  ///< The schedule of the traffic light. How many seconds until it changes to the
                                       ///< next color. The color will always follow a RED GREEN YELLOW RED schedule
        int current_schedule;   ///< Current position in the schedule vector . If the schedule vector is exhausted, will
                                ///< start over from position 0;
        double time_to_change;  ///< When the color should change to the next one
    };

    ///@brief Default Constructor
    ///
    ///@param system the ChSystem used for grabbing the current sim time
    SynEnvironmentAgent(ChSystem* system);

    ///@brief Destructor.
    virtual ~SynEnvironmentAgent();

    ///@brief Initialize this agents zombie representation
    /// Bodies are added and represented in the lead agent's world.
    ///
    ///@param system the ChSystem used to initialize the zombie
    virtual void InitializeZombie(ChSystem* system) override;

    ///@brief Synchronize this agents zombie with the rest of the simulation.
    /// Updates agent based on the passed message.
    /// Any message can be passed, so a check should be done to ensure this message was intended for this agent.
    ///
    ///@param message the message to process and is used to update the position of the zombie
    virtual void SynchronizeZombie(std::shared_ptr<SynMessage> message) override;

    ///@brief Update this agent
    /// Typically used to update the state representation of the agent to be distributed to other agents
    ///
    virtual void Update() override;

    ///@brief Generates messages to be sent to other nodes
    /// Will create or get messages and pass them into the referenced message vector
    ///
    ///@param messages a referenced vector containing messages to be distributed from this rank
    virtual void GatherMessages(SynMessageList& messages) override { messages.push_back(m_message); }

    ///@brief Get the description messages for this agent
    /// A single agent may have multiple description messages
    ///
    ///@param messages a referenced vector containing messages to be distributed from this rank
    virtual void GatherDescriptionMessages(SynMessageList& messages) override { messages.push_back(m_message); }

    // ------------------------------------------------------------------------

    /// @brief Add a lane to the environment, need to specify which intersection and which approach
    ///
    /// @param intersection particular intersection to add to
    /// @param approach particular approach to add to
    /// @param lane particular lane to add to
    /// @param color current color of the lane
    /// @param behaviour Indicates when the traffic light's color will change. If currently is RED and behaviour is
    /// {a,b,c}, it will stay RED for a seconds, switch to GREEN for b seconds, and Yellow for c seconds. Then it will
    /// switch back to RED and do the cycle again.
    /// @return The lane's position in that Approach
    int AddLane(int intersection, int approach, ApproachLane lane, LaneColor color, std::vector<double> behaviour);

    /// @brief Change the current color of a light at an intersection. Does not alter cycles
    ///
    /// @param intersection which intersection has the light we want to chane the color of
    /// @param approach which approach has the light we want to change the color of
    /// @param lane the lane's position in that approach, returned in AddLane
    /// @param color which color to change to
    void SetColor(int intersection, int approach, int lane, LaneColor color);

    ///@brief Set the Agent ID
    ///
    virtual void SetKey(AgentKey agent_key) override;

    // ------------------------------------------------------------------------

  private:
    /// @brief Find the next color in this order ...->GREEN->YELLOW->RED->GREEN->...
    ///
    LaneColor FindNextColor(LaneColor color);

    // ------------------------------------------------------------------------

    std::vector<LaneData> m_lane_data;  ///< Store all lane information

    ChSystem* m_system;                                ///< Ptr to the system for getting sim time
    std::shared_ptr<SynEnvironmentMessage> m_message;  ///< State of the environment
};

/// @} synchrono_agent

}  // namespace synchrono
}  // namespace chrono

#endif