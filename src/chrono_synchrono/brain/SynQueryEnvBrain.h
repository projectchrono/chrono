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
// Authors: Yanzheng Li, Jay Taves
// =============================================================================
//
// Brain that uses an ACCPathFollowerDriver in order to stay on a particular
// GPS path. Looks at state messages to determine where nearby vehicles and
// looks at traffic light data to determine the light status. Based on both light
// data and nearby vehicles, sets the CurrentDistance of the ACCPathFollower
// Compare with ACCBrain which uses the same logic but with a ChLidarSensor (and
// thus the GPU) to determine where nearby vehicles are.
//
// =============================================================================

#ifndef SYN_QUERY_ENV_BRAIN_H
#define SYN_QUERY_ENV_BRAIN_H

#include <tuple>

#include "chrono_synchrono/brain/SynVehicleBrain.h"
#include "chrono_synchrono/flatbuffer/message/SynEnvironmentMessage.h"

namespace chrono {
namespace synchrono {

/// @addtogroup synchrono_brain
/// @{

/// Vehicle Brain instance that instructs its vehicle to follow a given curve and stop based on zombie agent and traffic
/// light message data
class SYN_API SynQueryEnvBrain : public SynVehicleBrain {
  public:
    /// @brief Construct a new QueryEnvBrain object
    ///
    /// @param rank The MPI rank that this brain is associated with
    /// @param driver ACCPathFollowerDriver that will be used to steer this vehicle
    /// @param vehicle The Chrono Vehicle that is driven by this Brain
    /// @param veh_ranks A vector of the other ranks in simulation who should be checked for proximity
    SynQueryEnvBrain(int rank,
                     std::shared_ptr<vehicle::ChDriver> driver,
                     vehicle::ChVehicle& vehicle,
                     std::vector<int> veh_ranks);

    /// @brief Destroy the QueryEnvBrain object
    ~SynQueryEnvBrain();

    /// @brief Updates the CurrentDistance of the underlying ACC Driver based on traffic light and vehicle proximity
    /// data
    /// @param time The time in simulation that should be synchronized to
    virtual void Synchronize(double time);

    /// @brief Calls the advance function of the underlying driver
    /// @param step See the ACCPathFollowerDriver::Advance
    virtual void Advance(double step);

    /// @brief Updates state variables with information from the passed in message
    /// @param msg The message to be processed. If a vehicle message, the position will be set in the m_other_vehicles
    /// structure, if a MAP or SPAT message, the traffic light varibles will be updated
    virtual void ProcessMessage(SynMessage* msg);

    const double FAR_DISTANCE = 1000;  ///< Default distance to pass to ACC Driver
    const double BOX_WIDTH = 2.5;      ///< Width of the box used for detecting other vehicles
    const double BOX_LENGTH = 25;   ///< Length (ahead of the ego vehicle) of the box used for detecting other vehicles
    const double BOX_OFFSET = 2.5;  ///< Distance ahead of this vehicle to start the box

  private:
    /// The intersection, approach and lane the vehicle is in, for traffic light checking. Only meaningful when
    /// m_inside_box is true
    int m_current_intersection = 0;
    int m_current_approach = 0;
    int m_current_lane = 0;

    LaneColor m_light_color = LaneColor::RED;  ///< Starts red (0: green, 1: yellow, 2: yellow)
    bool m_inside_box = false;                 ///< Is vehicle inside stopping box area
    double m_dist = FAR_DISTANCE;  ///< Distance to nearest point (either traffic-light stopping area or a vehicle)

    std::map<int, std::tuple<bool, double>>
        m_other_vehicles;  ///< Structure indicating if nearby vehicles are directly ahead of us and if so how far away
};

/// @} synchrono_brain

}  // namespace synchrono
}  // namespace chrono

#endif  // PARK_ST_BRAIN_H
