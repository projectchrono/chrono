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

#ifndef SYN_ACC_BRAIN_H
#define SYN_ACC_BRAIN_H

#include "chrono_synchrono/brain/SynVehicleBrain.h"
#include "chrono_synchrono/flatbuffer/message/SynEnvironmentMessage.h"

#ifdef SENSOR
#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/ChLidarSensor.h"
#include "chrono_sensor/ChCameraSensor.h"
#endif

namespace chrono {
namespace synchrono {

/// @addtogroup synchrono_brain
/// @{

/// Vehicle Brain instance that instructs its vehicle to follow a given curve and stop based on lidar data and traffic
/// light message data
class SYN_API SynACCBrain : public SynVehicleBrain {
  public:
    SynACCBrain(int rank,
                std::shared_ptr<vehicle::ChDriver> driver,
                vehicle::ChVehicle& vehicle,
                bool is_multi_path = false);
#ifdef SENSOR
    SynACCBrain(int rank,
                std::shared_ptr<vehicle::ChDriver> driver,
                vehicle::ChVehicle& vehicle,
                std::shared_ptr<sensor::ChLidarSensor> lidar);
#endif
    ~SynACCBrain();

    virtual void Synchronize(double time);
    virtual void Advance(double step);
    virtual void ProcessMessage(SynMessage* msg);

    void setMultipath(bool Mul) { m_is_multi_path = Mul; }
    bool m_sen_init_called;

#ifdef SENSOR
    void SetLidar(std::shared_ptr<sensor::ChLidarSensor> lidar) { m_lidar = lidar; }
#endif

    void SetNearestVehicleDistance(float dist) { m_nearest_vehicle = dist; }

    const double FAR_DISTANCE = 1000;

  private:
    LaneColor m_light_color = LaneColor::RED;
    bool m_inside_box = false;     // Is vehicle inside stopping box area
    double m_dist = FAR_DISTANCE;  // Distance to point to stop at light

    /// The intersection, approach and lane the vehicle is in, for traffic light checking. Only meaningful when
    /// m_inside_box is true
    int m_current_intersection = 0;
    int m_current_approach = 0;
    int m_current_lane = 0;

    int m_nearest_vehicle;

    bool m_is_multi_path;  ///< the agent is using a multiPathacc driver or not.

#ifdef SENSOR
    std::shared_ptr<sensor::ChSensorManager> m_manager;
    std::shared_ptr<sensor::ChLidarSensor> m_lidar;
    std::shared_ptr<sensor::ChCameraSensor> m_camera;

    bool m_save_data;
    std::string m_cam_data_path;
    std::string m_intersection_cam_data_path;

    double m_lidar_intensity_epsilon;
    sensor::UserDIBufferPtr m_recent_lidar_data;
#endif
};

/// @} synchrono_brain

}  // namespace synchrono
}  // namespace chrono

#endif
