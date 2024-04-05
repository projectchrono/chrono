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

#ifndef SYN_CONTROLLER_FUNCTIONS_H
#define SYN_CONTROLLER_FUNCTIONS_H

#include <tuple>

#include "chrono_synchrono/flatbuffer/message/SynSPATMessage.h"
#include "chrono_synchrono/flatbuffer/message/SynApproachMessage.h"

#ifdef SENSOR
    #include "chrono_sensor/sensors/ChLidarSensor.h"
#endif

namespace chrono {
namespace synchrono {

/// @addtogroup synchrono_controller
/// @{

/// @param p point that the distance is computed from
/// @param l1 one end of the line
/// @param l2 other end of the line
double DistanceToLine(ChVector3d p, ChVector3d l1, ChVector3d l2);

/// @brief front, back and width define a box, check if pos is inside that box
/// @param pos vector position to be checked against the box
/// @param front vector position, line between this and back divides the box into two rectangles
/// @param back vector defining the back center of the box rectangle
/// @param width i.e. box_area = width * (front - back).length
bool IsInsideBox(ChVector3d pos, ChVector3d front, ChVector3d back, double width);

/// @brief Checks if pos is inside the (assumed convex) quadrilateral defined by vectors for each vertex
bool IsInsideQuad(ChVector3d pos, ChVector3d sp1, ChVector3d sp2, ChVector3d cp3, ChVector3d cp4);

/// @brief Compute barycentric coordinates (u, v, w) for point p with respect to triangle (a, b, c)
void Barycentric(ChVector3d p, ChVector3d a, ChVector3d b, ChVector3d c, double& u, double& v, double& w);

/// @brief update inside_box, dist and current_* variables based on info from a MAP message
/// @param synmsg must be castable to type SynMAPMessage
/// @param veh_pos current GREF vector location of the vehicle
/// @param rank of the vehicle we are checking
/// @param[in,out] inside_box whether the vehicle was in the stopping box for this particular lane
/// @param[in,out] current_lane if we're in a box, which lane that belonged to
/// @param[in,out] current_approach if we're in a box, which approach that belonged to
/// @param[in,out] current_intersection if we're in a box, which intersection that belonged to
/// @param[in,out] dist if vehicle is in box, how far is the vehicle from the box's stopping point (front of the box)
void UpdateLaneInfoFromMAP(std::shared_ptr<SynMessage> synmsg,
                           ChVector3d veh_pos,
                           const int& rank,
                           bool& inside_box,
                           int& current_lane,
                           int& current_approach,
                           int& current_intersection,
                           double& dist);

/// @brief update current_lane, inside_box and dist based on info from an Approach Message
void UpdateInsideBoxFromApproachMessage(std::shared_ptr<SynApproachMessage> app_msg,
                                        ChVector3d veh_pos,
                                        int& current_lane,
                                        bool& inside_box,
                                        double& dist);

/// @brief calls UpdateInsideBoxFromApproachMessage
void UpdateInsideBoxFromMessage(std::shared_ptr<SynMessage> synmsg,
                                ChVector3d veh_pos,
                                int& current_lane,
                                bool& inside_box,
                                double& dist);

/// @brief Given an intersection, approach and lane, parse a SPAT message (synmsg) and return the lane color
LaneColor GetLaneColorFromMessage(std::shared_ptr<SynMessage> synmsg,
                                  const int intersection,
                                  const int approach,
                                  const int lane);

#ifdef SENSOR
/// @brief Refresh passed lidar data, and search the resulting point cloud for points closer than min_val
double GetProximityToPointCloud(std::shared_ptr<sensor::ChLidarSensor> lidar,
                                double min_val,
                                &sensor::UserDIBUfferPtr recent_lidar_data);
#endif

/// @} synchrono_controller

}  // namespace synchrono
}  // namespace chrono

#endif  // SYN_CONTROLLER_FUNCTIONS_H