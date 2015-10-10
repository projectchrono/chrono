// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// A driver model that uses a path steering controller and a speed controller.
// The steering controller adjusts the steering input to follow the prescribed
// path.  The output from the speed controller is used to adjust throttle and
// braking inputs in order to maintain the prescribed constant vehicle speed.
//
// =============================================================================

#ifndef CH_PATHFOLLOWER_DRIVER_H
#define CH_PATHFOLLOWER_DRIVER_H

#include <string>

#include "chrono_vehicle/ChVehicle.h"
#include "chrono_vehicle/ChDriver.h"
#include "chrono_vehicle/utils/ChSteeringController.h"
#include "chrono_vehicle/utils/ChSpeedController.h"

namespace chrono {

class CH_VEHICLE_API ChPathFollowerDriver : public ChDriver {
  public:
    ChPathFollowerDriver(ChVehicle& vehicle,            ///< associated vehicle
                         ChBezierCurve* path,           ///< Bezier curve with target path
                         const std::string& path_name,  ///< name of the path curve
                         double target_speed            ///< constant target speed
                         );

    ChPathFollowerDriver(ChVehicle& vehicle,                    ///< associated vehicle
                         const std::string& steering_filename,  ///< JSON file with steering controller specification
                         const std::string& speed_filename,     ///< JSON file with speed controller specification
                         ChBezierCurve* path,                   ///< Bezier curve with target path
                         const std::string& path_name,          ///< name of the path curve
                         double target_speed                    ///< constant target speed
                         );

    ~ChPathFollowerDriver() {}

    void SetDesiredSpeed(double val) { m_target_speed = val; }
    void SetThreshholdThrottle(double val) { m_throttle_threshold = val; }

    ChPathSteeringController& GetSteeringController() { return m_steeringPID; }
    ChSpeedController& GetSpeedController() { return m_speedPID; }

    void Reset();
    virtual void Advance(double step) override;

    void ExportPathPovray(const std::string& out_dir);

  private:
    void Create();

    ChVehicle& m_vehicle;                    ///< reference to associated vehicle
    ChPathSteeringController m_steeringPID;  ///< steering controller
    ChSpeedController m_speedPID;            ///< speed controller
    double m_target_speed;                   ///< desired vehicle speed
    std::string m_pathName;                  ///< for path visualization
    double m_throttle_threshold;             ///< throttle value below which brakes are applied
};

}  // end namespace chrono

#endif
