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
// Authors: Yanzheng Li
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

#include "chrono_synchrono/brain/SynQueryEnvBrain.h"
#include "chrono_synchrono/brain/SynBrainFunctions.h"

#include "chrono_synchrono/brain/driver/SynMultipathDriver.h"
#include "chrono_synchrono/flatbuffer/message/SynWheeledVehicleMessage.h"

#include "chrono/core/ChLog.h"
#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_vehicle/driver/ChPathFollowerACCDriver.h"

using namespace chrono::vehicle;

namespace chrono {
namespace synchrono {

SynQueryEnvBrain::SynQueryEnvBrain(int rank,
                                   std::shared_ptr<ChDriver> driver,
                                   ChVehicle& vehicle,
                                   std::vector<int> veh_ranks)
    : SynVehicleBrain(rank, driver, vehicle) {
    for (int rank : veh_ranks)
        m_other_vehicles[rank] = std::make_tuple(false, FAR_DISTANCE);
}

SynQueryEnvBrain::~SynQueryEnvBrain() {}

void SynQueryEnvBrain::Synchronize(double time) {
    double temp_range = FAR_DISTANCE;
    if (m_light_color == LaneColor::RED && m_inside_box)
        temp_range = std::min(temp_range, m_dist);

    for (auto veh_pair : m_other_vehicles) {
        // veh_pair = (other veh in our box, distance to other veh)
        if (std::get<0>(veh_pair.second))
            temp_range = std::min(temp_range, std::get<1>(veh_pair.second));
    }

    std::static_pointer_cast<ChPathFollowerACCDriver>(m_driver)->SetCurrentDistance(temp_range);

    m_driver->Synchronize(time);
}

void SynQueryEnvBrain::Advance(double step) {
    m_driver->Advance(step);
}

void SynQueryEnvBrain::ProcessMessage(SynMessage* synmsg) {
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
        case SynMessageType::WHEELED_VEHICLE: {
            auto msg = (SynWheeledVehicleMessage*)synmsg;
            auto state = msg->GetWheeledState();

            ChVector<> zombie_pos = state->chassis.GetFrame().GetPos();
            int zombie_rank = msg->GetRank();

            // Front and back of the box ahead of the current vehicle
            ChVector<> box_front =
                m_vehicle.GetChassisBody()->TransformPointLocalToParent(ChVector<>(BOX_OFFSET + BOX_LENGTH, 0, 0.2));
            ChVector<> box_back =
                m_vehicle.GetChassisBody()->TransformPointLocalToParent(ChVector<>(BOX_OFFSET, 0, 0.2));

            if (IsInsideBox(zombie_pos, box_front, box_back, BOX_WIDTH)) {
                m_other_vehicles[zombie_rank] =
                    std::make_tuple(true, (zombie_pos - m_vehicle.GetVehiclePos()).Length());
            } else {
                m_other_vehicles[zombie_rank] = std::make_tuple(false, FAR_DISTANCE);
            }
            break;
        }
        default: {
            break;
        }
    }
}

}  // namespace synchrono
}  // namespace chrono
