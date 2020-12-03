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
// Authors: Aaron Young, 肖言 (Yan Xiao)
// =============================================================================
//
// Agent component that stops based on information from intelligent traffic
// lights and from lidar data.
// - Steering is provided by an ACC path follower that follows a curve
// - Throttle/braking control is provided by the ACC path follower based on
//    the distance to the closest "object"
//      - Being inside the stop box for a traffic light is an object
//      - A lidar sensor detecting something is an object
//
// =============================================================================

#include "chrono_synchrono/brain/SynACCBrain.h"
#include "chrono_synchrono/brain/SynBrainFunctions.h"
#include "chrono_synchrono/brain/driver/SynMultipathDriver.h"

#include "chrono_vehicle/driver/ChPathFollowerACCDriver.h"

using namespace chrono::vehicle;

namespace chrono {
namespace synchrono {

SynACCBrain::SynACCBrain(int rank, std::shared_ptr<ChDriver> driver, ChVehicle& vehicle, bool is_multi_path)
    : SynVehicleBrain(rank, driver, vehicle), m_nearest_vehicle(FAR_DISTANCE), m_is_multi_path(is_multi_path) {}

#ifdef SENSOR
SynACCBrain::SynACCBrain(int rank,
                         std::shared_ptr<ChDriver> driver,
                         ChVehicle& vehicle,
                         std::shared_ptr<ChLidarSensor> lidar)
    : SynACCBrain(rank, driver, vehicle), m_lidar(lidar) {}
#endif

SynACCBrain::~SynACCBrain() {}

void SynACCBrain::Synchronize(double time) {
    double temp_range = FAR_DISTANCE;
    if (m_light_color == LaneColor::RED && m_inside_box)
        temp_range = std::min(temp_range, m_dist);

#ifdef SENSOR
    temp_range = GetProximityToPointCloud(m_lidar, temp_range, m_recent_lidar_data);
#endif

    if (m_is_multi_path) {
        std::static_pointer_cast<ChMulPathFollowerACCDriver>(m_driver)->SetCurrentDistance(temp_range);
    } else {
        std::static_pointer_cast<ChPathFollowerACCDriver>(m_driver)->SetCurrentDistance(temp_range);
    }

    m_driver->Synchronize(time);
}

void SynACCBrain::Advance(double step) {
    m_driver->Advance(step);
}

// Tbrain is going to process the message to extract the location of the sender agent
// and also updates the location of the agent on the current rank
void SynACCBrain::ProcessMessage(SynMessage* synmsg) {
    switch (synmsg->GetType()) {
        case SynMessageType::MAP: {
            ChVector<> veh_pos = m_vehicle.GetChassisBody()->GetPos();
            UpdateLaneInfoFromMAP(synmsg, veh_pos, m_rank, m_inside_box, m_current_lane, m_current_approach,
                                  m_current_intersection, m_dist);
            break;
        }
        case SynMessageType::SPAT: {
            if (m_inside_box)
                m_light_color =
                    GetLaneColorFromMessage(synmsg, m_current_intersection, m_current_approach, m_current_lane);

            break;
        }
        default: {
            break;
        }
    }
}

}  // namespace synchrono
}  // namespace chrono
