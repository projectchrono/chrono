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
// Authors: Jay Taves
// =============================================================================
//
// Contains several helper functions that are useful when writing controllers. Until
// we have a more robust and flexible method for controller composition, these
// functions will help people create families of controllers with similar traits.
//
// =============================================================================

#include "chrono_synchrono/controller/SynControllerFunctions.h"

#include "chrono_synchrono/flatbuffer/message/SynApproachMessage.h"
#include "chrono_synchrono/flatbuffer/message/SynMAPMessage.h"

#include "chrono_synchrono/utils/SynLog.h"

namespace chrono {
namespace synchrono {

double DistanceToLine(ChVector3d p, ChVector3d l1, ChVector3d l2) {
    // https://www.intmath.com/plane-analytic-geometry/perpendicular-distance-point-line.php
    double A = l1.y() - l2.y();
    double B = -(l1.x() - l2.x());
    double C = l1.x() * l2.y() - l1.y() * l2.x();
    double d = abs(A * p.x() + B * p.y() + C) / sqrt(A * A + B * B);
    return d;
}

bool IsInsideBox(ChVector3d pos, ChVector3d front, ChVector3d back, double width) {
    double len = (front - back).Length();
    // TODO :: Should be width / 2, but paths are too far away from the actual lanes

    return (pos - front).Length() < len && (pos - back).Length() < len && DistanceToLine(pos, front, back) < width;
}

bool IsInsideQuad(ChVector3d pos, ChVector3d sp1, ChVector3d sp2, ChVector3d cp3, ChVector3d cp4) {
    bool insideTri1;
    bool insideTri2;

    double u, v, w;
    ChVector3d posN = ChVector3d({pos.x(), pos.y(), 0});
    ChVector3d sp1N = ChVector3d({sp1.x(), sp1.y(), 0});
    ChVector3d sp2N = ChVector3d({sp2.x(), sp2.y(), 0});
    ChVector3d cp3N = ChVector3d({cp3.x(), cp3.y(), 0});
    ChVector3d cp4N = ChVector3d({cp4.x(), cp4.y(), 0});

    Barycentric(posN, sp1N, sp2N, cp3N, u, v, w);
    insideTri1 = u > 0 && v > 0 && w > 0;
    Barycentric(posN, sp1N, cp3N, cp4N, u, v, w);
    insideTri2 = u > 0 && v > 0 && w > 0;

    return insideTri1 || insideTri2;
}

// https://gamedev.stackexchange.com/questions/23743/whats-the-most-efficient-way-to-find-barycentric-coordinates
void Barycentric(ChVector3d p, ChVector3d a, ChVector3d b, ChVector3d c, double& u, double& v, double& w) {
    ChVector3d v0 = b - a;
    ChVector3d v1 = c - a;
    ChVector3d v2 = p - a;
    double d00 = v0 ^ v0;
    double d01 = v0 ^ v1;
    double d11 = v1 ^ v1;
    double d20 = v2 ^ v0;
    double d21 = v2 ^ v1;
    double denom = d00 * d11 - d01 * d01;
    v = (d11 * d20 - d01 * d21) / denom;
    w = (d00 * d21 - d01 * d20) / denom;
    u = 1.0f - v - w;
}

void UpdateLaneInfoFromMAP(std::shared_ptr<SynMessage> synmsg,
                           ChVector3d veh_pos,
                           const int& rank,
                           bool& inside_box,
                           int& current_lane,
                           int& current_approach,
                           int& current_intersection,
                           double& dist) {
    inside_box = false;
    auto msg = std::dynamic_pointer_cast<SynMAPMessage>(synmsg);
    auto intersections = msg->intersections;

    // iteratively search for each lane, break the loop when the vehicle found it is in a lane.
    for (int i = 0; i < intersections.size(); i++) {
        auto approaches = intersections[i].approaches;
        for (int j = 0; j < approaches.size(); j++) {
            UpdateInsideBoxFromApproachMessage(approaches[j], veh_pos, current_lane, inside_box, dist);
            if (inside_box) {
                current_approach = j;
                break;
            }
        }
        if (inside_box) {
            current_intersection = i;
            break;
        }
    }
}

void UpdateInsideBoxFromMessage(std::shared_ptr<SynMessage> synmsg,
                                ChVector3d veh_pos,
                                int& current_lane,
                                bool& inside_box,
                                double& dist) {
    auto app_msg = std::dynamic_pointer_cast<SynApproachMessage>(synmsg);
    UpdateInsideBoxFromApproachMessage(app_msg, veh_pos, current_lane, inside_box, dist);
}

void UpdateInsideBoxFromApproachMessage(std::shared_ptr<SynApproachMessage> app_msg,
                                        ChVector3d veh_pos,
                                        int& current_lane,
                                        bool& inside_box,
                                        double& dist) {
    auto map_lanes = app_msg->lanes;

    for (int i = 0; i < map_lanes.size(); i++) {
        if (IsInsideBox(veh_pos, map_lanes[i].controlPoints[0], map_lanes[i].controlPoints[1], map_lanes[i].width)) {
            current_lane = i;
            inside_box = true;
            dist = (map_lanes[i].controlPoints[0] - veh_pos).Length();
            return;
        }
    }
}

LaneColor GetLaneColorFromMessage(std::shared_ptr<SynMessage> synmsg,
                                  const int intersection,
                                  const int approach,
                                  const int caller_lane) {
    auto msg = std::dynamic_pointer_cast<SynSPATMessage>(synmsg);
    auto spat_lanes = msg->lanes;

    for (auto lane : spat_lanes) {
        if (intersection == lane.intersection && approach == lane.approach && caller_lane == lane.lane)
            return lane.color;
    }

    SynLog() << "Passed intersection/approach/lane was not found in SPAT message. Returning RED."
             << "\n";
    return LaneColor::RED;
}

#ifdef CHRONO_SENSOR
double GetProximityToPointCloud(std::shared_ptr<sensor::ChLidarSensor> lidar,
                                double min_range,
                                sensor::UserDIBufferPtr& recent_lidar_data) {
    // If user gave us a bad sensor value, we don't update their previous minimum
    if (!lidar)
        return min_range;

    // For a while this was 0, if needed we could make this user-changeable
    double lidar_intensity_epsilon = 10e-2;

    // This only returns non-null at the lidar frequency, so only overwrite if it was non-null
    sensor::UserDIBufferPtr recent_lidar = lidar->GetMostRecentBuffer<sensor::UserDIBufferPtr>();

    // Check and update their lidar data
    if (recent_lidar->Buffer)
        recent_lidar_data = recent_lidar;

    if (recent_lidar_data && recent_lidar_data->Buffer) {
        for (unsigned int i = 0; i < recent_lidar_data->Height; i++) {
            for (unsigned int j = 0; j < recent_lidar_data->Width; j++) {
                double lidar_range = recent_lidar_data->Buffer[i * recent_lidar_data->Width + j].range;
                double lidar_intensity = recent_lidar_data->Buffer[i * recent_lidar_data->Width + j].intensity;

                // If we find a lidar point nearer than min_range with enough intensity, update the min_range
                if (lidar_intensity >= lidar_intensity_epsilon)
                    min_range = std::min(lidar_range, min_range);
            }
        }
    }

    return min_range;
}
#endif

}  // namespace synchrono
}  // namespace chrono