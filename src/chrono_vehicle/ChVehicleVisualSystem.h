// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2022 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Base class for a vehicle visualization system
//
// =============================================================================

#ifndef CH_VEHICLE_VISUAL_SYSTEM_H
#define CH_VEHICLE_VISUAL_SYSTEM_H

#include "chrono/assets/ChVisualSystem.h"
#include "chrono/utils/ChUtilsChaseCamera.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChVehicle.h"
#include "chrono_vehicle/ChDriver.h"

namespace chrono {
namespace vehicle {

class CH_VEHICLE_API ChVehicleVisualSystem : virtual public ChVisualSystem {
  public:
    ChVehicleVisualSystem();
    virtual ~ChVehicleVisualSystem();

    /// Set parameters for the underlying chase camera.
    void SetChaseCamera(const ChVector<>& ptOnChassis,  ///< tracked point on chassis body (in vehicle reference frame)
                        double chaseDist,               ///< chase distance (behind tracked point)
                        double chaseHeight              ///< chase height (above tracked point)
    );
    /// Set the step size for integration of the chase-cam dynamics.
    void SetStepsize(double val);
    /// Set camera state (mode).
    void SetChaseCameraState(utils::ChChaseCamera::State state);
    /// Set camera position.
    /// Note that this forces the chase-cam in Track mode.
    void SetChaseCameraPosition(const ChVector<>& pos);
    /// Set camera angle.
    void SetChaseCameraAngle(double angle);
    /// Set camera zoom multipliers.
    void SetChaseCameraMultipliers(double minMult, double maxMult);

    /// Update information related to driver inputs.
    virtual void Synchronize(const std::string& msg, const DriverInputs& driver_inputs);

  protected:
    virtual void OnAttachToVehicle();

    ChVehicle* m_vehicle;  ///< pointer to the associated vehicle system

    std::unique_ptr<utils::ChChaseCamera> m_camera;  ///< chase camera
    double m_stepsize;                               ///< integration step size for chase-cam dynamics
    ChVector<> m_camera_point;                       ///< point on tracked body
    double m_camera_dist;                            ///< camera chase distance
    double m_camera_height;                          ///< camera chase height
    utils::ChChaseCamera::State m_camera_state;      ///< initial camera state
    ChVector<> m_camera_pos;                         ///< initial camera position
    double m_camera_angle;                           ///< initial camera angle;
    double m_camera_minMult;                         ///< initial camera minimum multiplier
    double m_camera_maxMult;                         ///< initial camera maximum multiplier

    std::string m_driver_msg;  ///< HUD message from driver system
    double m_steering;         ///< driver steering input
    double m_throttle;         ///< driver throttle input
    double m_braking;          ///< driver braking input

    friend class ChVehicle;
};

}  // namespace vehicle
}  // namespace chrono

#endif
