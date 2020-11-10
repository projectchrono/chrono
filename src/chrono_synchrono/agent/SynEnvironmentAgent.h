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

#ifndef SYN_ENVIRONMENT_AGENT_H
#define SYN_ENVIRONMENT_AGENT_H

#include "chrono_synchrono/SynApi.h"
#include "chrono_synchrono/agent/SynAgent.h"

#include "chrono_synchrono/flatbuffer/message/SynMAPMessage.h"
#include "chrono_synchrono/flatbuffer/message/SynSPATMessage.h"

namespace chrono {
namespace synchrono {

/// @addtogroup synchrono_agent
/// @{

/// Agent that manages lanes and light changes for a traffic intersection. Transmits messages to receiving vehicle
/// agents about the current state of the intersection that is managed
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

    /// @brief Construct a new Syn Environment Agent object, only need one for the simulation
    ///
    /// @param rank Current rank the agent is at
    /// @param system The Chrono system the agent is in.
    SynEnvironmentAgent(int rank, ChSystem* system = 0);

    /// Destructor
    ~SynEnvironmentAgent();

    /// Advance the state of this vehicle agent until agent time syncs with passed time.
    virtual void Advance(double time_of_next_sync) override;

    /// Initialize this agents zombie representation.
    /// Will use the passed system if agent system is null.
    virtual void InitializeZombie(ChSystem* system = 0) override;

    /// Synchronize this agents zombie with the rest of the simulation.
    /// Updates agent based on specified message.
    virtual void SynchronizeZombie(SynMessage* message) override;

    /// Generate vector of SynMessages to send
    virtual void GenerateMessagesToSend(std::vector<SynMessage*>& messages) override;

    /// Environments have no state as such, they have state for SPAT + MAP
    virtual std::shared_ptr<SynMessageState> GetState() override;
    virtual std::shared_ptr<SynAgentMessage> GetMessage() override;

    ///
    /// @brief Add a lane to the environment, need to specify which intersection and which approach
    ///
    /// @param lane lane information
    /// @param color current color of the lane
    /// @param behaviour Indicates when the traffic light's color will change. If currently is RED and behaviour is
    /// {a,b,c}, it will stay RED for a seconds, switch to GREEN for b seconds, and Yellow for c seconds. Then it will
    /// switch back to RED and do the cycle again.
    /// @return The lane's position in that Approach
    ///
    int AddLane(int intersection, int approach, ApproachLane lane, LaneColor color, std::vector<double> behaviour);

    /// @brief
    ///
    /// @param lane The lane's position in that approach, returned in AddLane
    void SetColor(int intersection, int approach, int lane, LaneColor color);

  private:
    /// @brief Find the next color in this order ...->GREEN->YELLOW->RED->GREEN->...
    LaneColor FindNextColor(LaneColor color);

    // agent specific parameters
    double m_current_time;
    std::vector<LaneData> m_lane_light_data;  ///< Store all lane information

    std::shared_ptr<SynMAPMessage> m_map_msg;
    std::shared_ptr<SynSPATMessage> m_spat_msg;
};

/// @} synchrono_agent

}  // namespace synchrono
}  // namespace chrono
#endif
