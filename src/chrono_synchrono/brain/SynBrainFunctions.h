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
// Contains several helper functions that are useful when writing brains. Until
// we have a more robust and flexible method for brain composition, these
// functions will help people create families of brains with similar traits.
//
// =============================================================================

#ifndef SYN_BRAINFUNCTIONS_H
#define SYN_BRAINFUNCTIONS_H

#include <tuple>

#include "chrono_synchrono/flatbuffer/message/SynSPATMessage.h"
#include "chrono_synchrono/flatbuffer/message/SynApproachMessage.h"

#ifdef SENSOR
#include "chrono_sensor/ChLidarSensor.h"
#endif

namespace chrono {
namespace synchrono {

/// @addtogroup synchrono_brain
/// @{

/// @param p point that the distance is computed from
/// @param l1 one end of the line
/// @param l2 other end of the line
double DistanceToLine(ChVector<> p, ChVector<> l1, ChVector<> l2);

/// @brief front, back and width define a box, check if pos is inside that box
/// @param front vector position, line between this and back divides the box into two rectangles
bool IsInsideBox(ChVector<> pos, ChVector<> front, ChVector<> back, double width);

/// @brief Checks if pos is inside the (assumed convex) quadrilateral defined by vectors for each vertex
bool IsInsideQuad(ChVector<> pos, ChVector<> sp1, ChVector<> sp2, ChVector<> cp3, ChVector<> cp4);

/// @brief Compute barycentric coordinates (u, v, w) for point p with respect to triangle (a, b, c)
void Barycentric(ChVector<> p, ChVector<> a, ChVector<> b, ChVector<> c, float& u, float& v, float& w);

/// @brief update inside_box, dist and current_* variables based on info from a MAP message
/// @param synmsg must be castable to type SynMAPMessage
/// @param dist if vehicle is in box, how far is the vehicle from the box's stopping point (front of the box)
void UpdateLaneInfoFromMAP(SynMessage* synmsg,
                           ChVector<> veh_pos,
                           const int& rank,
                           bool& inside_box,
                           int& current_lane,
                           int& current_approach,
                           int& current_intersection,
                           double& dist);

/// @brief update current_lane, inside_box and dist based on info from an Approach Message
void UpdateInsideBoxFromApproachMessage(SynApproachMessage& app_msg,
                                        ChVector<> veh_pos,
                                        int& current_lane,
                                        bool& inside_box,
                                        double& dist);

/// @brief calls UpdateInsideBoxFromApproachMessage
void UpdateInsideBoxFromMessage(SynMessage* synmsg,
                                ChVector<> veh_pos,
                                int& current_lane,
                                bool& inside_box,
                                double& dist);

/// @brief Given an intersection, approach and lane, parse a SPAT message (synmsg) and return the lane color
LaneColor GetLaneColorFromMessage(SynMessage* synmsg, const int intersection, const int approach, const int lane);

#ifdef SENSOR
/// @brief Refresh passed lidar data, and search the resulting point cloud for points closer than min_val
double GetProximityToPointCloud(std::shared_ptr<sensor::ChLidarSensor> lidar,
                                double min_val,
                                &sensor::UserDIBUfferPtr recent_lidar_data);
#endif

/// @} synchrono_brain

}  // namespace synchrono
}  // namespace chrono

#endif  // SYN_BRAINFUNCTIONS_H