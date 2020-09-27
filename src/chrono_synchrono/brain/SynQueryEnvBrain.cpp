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

namespace chrono {
namespace synchrono {

SynQueryEnvBrain::SynQueryEnvBrain(int rank,
                                   std::shared_ptr<ChDriver> driver,
                                   ChVehicle& vehicle,
                                   std::vector<int> veh_ranks)
    : SynVehicleBrain(rank, driver, vehicle) {
    for (int rank : veh_ranks)
        m_other_vehicles[rank] = std::make_tuple(false, FAR_DISTANCE);

    m_rank = rank;
}

SynQueryEnvBrain::~SynQueryEnvBrain() {}

void SynQueryEnvBrain::Synchronize(double time) {
    double temp_range = FAR_DISTANCE;

    if (m_light_color == LaneColor::RED && m_inside_box) {
        temp_range = temp_range < m_dist ? temp_range : m_dist;
    }

    for (auto veh_pair : m_other_vehicles) {
        // First item is a bool indicating whether that vehicle is in our box, second item is a double indicating the
        // distance to the vehicle if they are in our box
        if (std::get<0>(veh_pair.second)) {
            double veh_dist = std::get<1>(veh_pair.second);
            temp_range = temp_range > veh_dist ? veh_dist : temp_range;
        }
    }

    std::static_pointer_cast<ChPathFollowerACCDriver>(m_driver)->SetCurrentDistance(temp_range);

    m_driver->Synchronize(time);
}

void SynQueryEnvBrain::Advance(double step) {
    m_driver->Advance(step);
}

void SynQueryEnvBrain::ProcessMessage(SynMessage* synmsg) {
    switch (synmsg->GetType()) {
        case SynMessageType::ENVIRONMENT: {
            // Environment agents don't send messages of their own,
            std::cout << "Received unexpected environment message" << std::endl;
            break;
        }
        case SynMessageType::MAP: {
            m_inside_box = false;
            auto msg = (SynMAPMessage*)synmsg;
            auto intersections = msg->GetMAPState()->intersections;

            // iteratively search for each lane, break the loop when the vehicle found it is in a lane.
            for (int i = 0; i < intersections.size(); i++) {
                auto approaches = intersections[i].approaches;
                for (int j = 0; j < approaches.size(); j++) {
                    SynApproachMessage* app_msg = new SynApproachMessage(
                        m_rank, chrono_types::make_shared<SynApproachMessageState>(approaches[j]));
                    ProcessMessage(app_msg);
                    if (m_inside_box) {
                        m_current_approach = j;
                        break;
                    }
                }
                if (m_inside_box) {
                    m_current_intersection = i;
                    break;
                }
            }

            break;
        }
        case SynMessageType::APPROACH: {
            auto msg = (SynApproachMessage*)synmsg;

            auto map_lanes = msg->GetApproachState()->lanes;
            ChVector<> pos = m_vehicle.GetChassisBody()->GetPos();

            for (int i = 0; i < map_lanes.size(); i++) {
                if (IsInsideBox(pos, map_lanes[i].controlPoints[0], map_lanes[i].controlPoints[1],
                                map_lanes[i].width)) {
                    m_current_lane = i;
                    m_inside_box = true;
                    m_dist = (map_lanes[i].controlPoints[0] - pos).Length();
                    break;
                }
            }

            break;
        }
        case SynMessageType::SPAT: {
            // SPAT Processing
            if (!m_inside_box) {
                break;
            }

            auto msg = (SynSPATMessage*)synmsg;
            auto spat_lanes = msg->GetSPATState()->lanes;

            for (auto lane : spat_lanes) {
                if (m_current_intersection == lane.intersection && m_current_approach == lane.approach &&
                    m_current_lane == lane.lane) {
                    m_light_color = lane.color;
                    break;
                }
            }
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

double SynQueryEnvBrain::DistanceToLine(ChVector<> p, ChVector<> l1, ChVector<> l2) {
    // https://www.intmath.com/plane-analytic-geometry/perpendicular-distance-point-line.php
    double A = l1.y() - l2.y();
    double B = -(l1.x() - l2.x());
    double C = l1.x() * l2.y() - l1.y() * l2.x();
    double d = abs(A * p.x() + B * p.y() + C) / sqrt(A * A + B * B);
    return d;
}

bool SynQueryEnvBrain::IsInsideBox(ChVector<> pos, ChVector<> sp, ChVector<> op, double w) {
    double len = (sp - op).Length();
    // TODO :: Should be w / 2, but paths are too far away from the actual lanes

    return (pos - sp).Length() < len && (pos - op).Length() < len && DistanceToLine(pos, sp, op) < w;
}

}  // namespace synchrono
}  // namespace chrono
