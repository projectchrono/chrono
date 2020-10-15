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

#include "chrono_synchrono/brain/driver/SynMultipathDriver.h"

#include "chrono_vehicle/driver/ChPathFollowerACCDriver.h"
#include "chrono_thirdparty/filesystem/path.h"

#ifdef SENSOR
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/filters/ChFilterVisualizePointCloud.h"
#include "chrono_sensor/filters/ChFilterPCfromDepth.h"
#include "chrono_sensor/filters/ChFilterSave.h"
#endif

namespace chrono {
namespace synchrono {

SynACCBrain::SynACCBrain(int rank, std::shared_ptr<ChDriver> driver, ChVehicle& vehicle, bool is_multi_path)
    : SynVehicleBrain(rank, driver, vehicle) {
    m_nearest_vehicle = 1000;
    m_is_multi_path = is_multi_path;
    m_rank = rank;
}

#ifdef SENSOR
SynACCBrain::SynACCBrain(int rank,
                         std::shared_ptr<ChDriver> driver,
                         ChVehicle& vehicle,
                         std::shared_ptr<ChLidarSensor> lidar)
    : SynVehicleBrain(rank, driver, vehicle) {
    SynACCBrain(rank, driver, vehicle);
    m_lidar = lidar;
}
#endif

SynACCBrain::~SynACCBrain() {}

void SynACCBrain::Synchronize(double time) {
    double temp_range = 1000;
    if (m_light_color == LaneColor::RED && m_inside_box) {
        // std::cout << "Rank " << m_rank << " inside box with red light. Distance " << m_dist << std::endl;
        temp_range = temp_range < m_dist ? temp_range : m_dist;
    }

#ifdef SENSOR
    if (m_lidar) {
        // This only returns non-null at the lidar frequency, so only overwrite if it was non-null
        UserDIBufferPtr recent_lidar = m_lidar->GetMostRecentBuffer<UserDIBufferPtr>();

        if (recent_lidar->Buffer) {
            m_recent_lidar_data = recent_lidar;
        }

        float min_val = m_nearest_vehicle / 15.0;

        if (m_recent_lidar_data && m_recent_lidar_data->Buffer) {
            for (int i = 0; i < m_recent_lidar_data->Height; i++) {
                for (int j = 0; j < m_recent_lidar_data->Width; j++) {
                    double temp_range = m_recent_lidar_data->Buffer[i * m_recent_lidar_data->Width + j].range;
                    double temp_intensity = m_recent_lidar_data->Buffer[i * m_recent_lidar_data->Width + j].intensity;

                    if (temp_range < min_val && temp_intensity >= m_lidar_intensity_epsilon) {
                        min_val = temp_range;
                    }
                }
            }
        }

        temp_range = temp_range < min_val ? temp_range : min_val;
    }

#endif
    if (m_is_multi_path) {
        std::static_pointer_cast<ChMulPathFollowerACCDriver>(m_driver)->SetCurrentDistance(temp_range);
    } else {
        std::static_pointer_cast<ChPathFollowerACCDriver>(m_driver)->SetCurrentDistance(temp_range);
    }

    // std::cout << "Range is: " << temp_range << std::endl;

    m_driver->Synchronize(time);
}

void SynACCBrain::Advance(double step) {
    m_driver->Advance(step);
}

// Tbrain is going to process the message to extract the location of the sender agent
// and also updates the location of the agent on the current rank
void SynACCBrain::ProcessMessage(SynMessage* synmsg) {
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
        default: {
            break;
        }
    }
}

double SynACCBrain::DistanceToLine(ChVector<> p, ChVector<> l1, ChVector<> l2) {
    // https://www.intmath.com/plane-analytic-geometry/perpendicular-distance-point-line.php
    double A = l1.y() - l2.y();
    double B = -(l1.x() - l2.x());
    double C = l1.x() * l2.y() - l1.y() * l2.x();
    double d = abs(A * p.x() + B * p.y() + C) / sqrt(A * A + B * B);
    return d;
}

bool SynACCBrain::IsInsideBox(ChVector<> pos, ChVector<> sp, ChVector<> op, double w) {
    double len = (sp - op).Length();
    // TODO :: Should be w / 2, but paths are too far away from the actual lanes

    return (pos - sp).Length() < len && (pos - op).Length() < len && DistanceToLine(pos, sp, op) < w;
}

// Only works for convex box
bool SynACCBrain::IsInsideQuad(ChVector<> pos, ChVector<> sp1, ChVector<> sp2, ChVector<> cp3, ChVector<> cp4) {
    bool insideTri1;
    bool insideTri2;

    float u, v, w;
    ChVector<> posN = ChVector<>({pos.x(), pos.y(), 0});
    ChVector<> sp1N = ChVector<>({sp1.x(), sp1.y(), 0});
    ChVector<> sp2N = ChVector<>({sp2.x(), sp2.y(), 0});
    ChVector<> cp3N = ChVector<>({cp3.x(), cp3.y(), 0});
    ChVector<> cp4N = ChVector<>({cp4.x(), cp4.y(), 0});

    Barycentric(posN, sp1N, sp2N, cp3N, u, v, w);
    insideTri1 = u > 0 && v > 0 && w > 0;
    Barycentric(posN, sp1N, cp3N, cp4N, u, v, w);
    insideTri2 = u > 0 && v > 0 && w > 0;

    return insideTri1 || insideTri2;
}

// Compute barycentric coordinates (u, v, w) for
// point p with respect to triangle (a, b, c)
// https://gamedev.stackexchange.com/questions/23743/whats-the-most-efficient-way-to-find-barycentric-coordinates
void SynACCBrain::Barycentric(ChVector<> p, ChVector<> a, ChVector<> b, ChVector<> c, float& u, float& v, float& w) {
    ChVector<> v0 = b - a;
    ChVector<> v1 = c - a;
    ChVector<> v2 = p - a;
    float d00 = v0 ^ v0;
    float d01 = v0 ^ v1;
    float d11 = v1 ^ v1;
    float d20 = v2 ^ v0;
    float d21 = v2 ^ v1;
    float denom = d00 * d11 - d01 * d01;
    v = (d11 * d20 - d01 * d21) / denom;
    w = (d00 * d21 - d01 * d20) / denom;
    u = 1.0f - v - w;
}

}  // namespace synchrono
}  // namespace chrono
