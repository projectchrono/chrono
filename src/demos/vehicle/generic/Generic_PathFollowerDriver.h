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
//
// =============================================================================

#ifndef GENERIC_PATHFOLLOWER_DRIVER_H
#define GENERIC_PATHFOLLOWER_DRIVER_H

#include <string>

#include "chrono_vehicle/ChVehicle.h"
#include "chrono_vehicle/ChDriver.h"
#include "chrono_vehicle/utils/ChSteeringController.h"
#include "chrono_vehicle/utils/ChSpeedController.h"

class Generic_PathFollowerDriver : public chrono::ChDriver {
  public:
    Generic_PathFollowerDriver(chrono::ChVehicle& vehicle,   ///< associated vehicle
                               chrono::ChBezierCurve* path,  ///< Bezier curve with target path
                               const std::string& path_name  ///< name of the path curve
                               );

    Generic_PathFollowerDriver(
        chrono::ChVehicle& vehicle,            ///< associated vehicle
        const std::string& steering_filename,  ///< JSON file with steering controller specification
        const std::string& speed_filename,     ///< JSON file with speed controller specification
        chrono::ChBezierCurve* path,           ///< Bezier curve with target path
        const std::string& path_name           ///< name of the path curve
        );

    ~Generic_PathFollowerDriver() {}

    void SetDesiredSpeed(double val) { m_target_speed = val; }

    chrono::ChPathSteeringController& GetSteeringController() { return m_steeringPID; }
    chrono::ChSpeedController& GetSpeedController() { return m_speedPID; }

    void Reset();
    virtual void Advance(double step) override;

    void ExportPathPovray(const std::string& out_dir);

  private:
    void Create();

    chrono::ChVehicle& m_vehicle;                    ///< reference to associated vehicle
    chrono::ChPathSteeringController m_steeringPID;  ///< steering controller
    chrono::ChSpeedController m_speedPID;            ///< speed controller
    double m_target_speed;                           ///< desired vehicle speed
    std::string m_pathName;                          ///< for path visualization
};

#endif
