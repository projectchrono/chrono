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
#include "chrono_vehicle/ChTerrain.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_vis
/// @{

class CH_VEHICLE_API ChVehicleVisualSystem : virtual public ChVisualSystem {
  public:
    ChVehicleVisualSystem();
    virtual ~ChVehicleVisualSystem();

    /// Attach a vehicle to this vehicle visualization system.
    virtual void AttachVehicle(ChVehicle* vehicle);

    /// Attach a driver to this vehicle visualization system.
    virtual void AttachDriver(ChDriver* driver);

    /// Attach a terrain system to this vehicle visualization system (optional).
    virtual void AttachTerrain(ChTerrain* terrain);

    /// Set parameters for the underlying chase camera.
    void SetChaseCamera(const ChVector3d& ptOnChassis,  ///< tracked point on chassis body (in vehicle reference frame)
                        double chaseDist,               ///< chase distance (behind tracked point)
                        double chaseHeight              ///< chase height (above tracked point)
    );
    /// Set the step size for integration of the chase-cam dynamics.
    void SetStepsize(double val);
    /// Set camera state (mode).
    void SetChaseCameraState(utils::ChChaseCamera::State state);
    /// Set camera position.
    /// Note that this forces the chase-cam in Track mode.
    void SetChaseCameraPosition(const ChVector3d& pos);
    /// Set camera angle.
    void SetChaseCameraAngle(double angle);
    /// Set camera zoom multipliers.
    void SetChaseCameraMultipliers(double minMult, double maxMult);

    /// Update visual system at the current time.
    virtual void Synchronize(double time, const DriverInputs& driver_inputs);

    /// Advance (optional) dynamics of the visualization system.
    virtual void Advance(double step) {}

    /// Return the step RTF calculated by the associated vehicle (step time / simulated time).
    /// See ChVehicle::GetStepRTF.
    double GetStepRTF() const;

    const ChVehicle& GetVehicle() const { return *m_vehicle; }
    const ChTerrain* GetTerrain() const { return m_terrain; }
    ChDriver* GetDriver() { return m_driver; }
    const utils::ChChaseCamera& GetChaseCamera() const { return *m_camera; }
    double GetSteering() const { return m_steering; }
    double GetThrottle() const { return m_throttle; }
    double GetBraking() const { return m_braking; }
    double GetClutch() const { return m_clutch; }

  protected:
    ChVehicle* m_vehicle;  ///< pointer to the associated vehicle system
    ChDriver* m_driver;    ///< pointer to an associated driver system (may be null)
    ChTerrain* m_terrain;  ///< pointer to an associated terrain system

    std::unique_ptr<utils::ChChaseCamera> m_camera;  ///< chase camera
    double m_stepsize;                               ///< integration step size for chase-cam dynamics
    ChVector3d m_camera_point;                       ///< point on tracked body
    double m_camera_dist;                            ///< camera chase distance
    double m_camera_height;                          ///< camera chase height
    utils::ChChaseCamera::State m_camera_state;      ///< initial camera state
    ChVector3d m_camera_pos;                         ///< initial camera position
    double m_camera_angle;                           ///< initial camera angle;
    double m_camera_minMult;                         ///< initial camera minimum multiplier
    double m_camera_maxMult;                         ///< initial camera maximum multiplier

    double m_steering;  ///< driver steering input
    double m_throttle;  ///< driver throttle input
    double m_braking;   ///< driver braking input
    double m_clutch;    ///< driver clutch input

    friend class ChVehicle;
};

/// @} vehicle_vis

}  // namespace vehicle
}  // namespace chrono

#endif
