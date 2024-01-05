// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
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
// Co-simulation FMU encapsulating a vehicle path-follower driver system.
// The driver model includes a lateral path-following PID controller and a
// longitudinal PID cruise controller.
//
// The wrapped Chrono::Vehicle driver model is defined through a data file
// specifying the path for the lateral controller.
//
// This driver FMU must be co-simulated with a vehicle system which provides
// the current vehicle reference frame (of type ChFrameMoving).
//
// This driver FMU defines continuous output variables for:
//   - sterring command (in [-1,1])
//   - throttle command (in [0,1]
//   - braking command (in [0,1])
//
// =============================================================================

#pragma once

#include <string>
#include <vector>
#include <array>

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/geometry/ChGeometry.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/utils/ChSpeedController.h"
#include "chrono_vehicle/utils/ChSteeringController.h"

#ifdef CHRONO_IRRLICHT
    #include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
#endif

// #define FMI2_FUNCTION_PREFIX MyModel_
#include "chrono_fmi/ChFmuToolsExport.h"

class FmuComponent : public chrono::FmuChronoComponentBase {
  public:
    FmuComponent(fmi2String _instanceName, fmi2Type _fmuType, fmi2String _fmuGUID);
    ~FmuComponent() {}

    /// Advance dynamics.
    virtual fmi2Status _doStep(fmi2Real currentCommunicationPoint,
                               fmi2Real communicationStepSize,
                               fmi2Boolean noSetFMUStatePriorToCurrentPoint) override;

  private:
    virtual void _enterInitializationMode() override;
    virtual void _exitInitializationMode() override;

    virtual void _preModelDescriptionExport() override;
    virtual void _postModelDescriptionExport() override;

    virtual bool is_cosimulation_available() const override { return true; }
    virtual bool is_modelexchange_available() const override { return false; }

    /// Create the driver system.
    /// This function is invoked in _exitInitializationMode(), once FMU parameters are set.
    void CreateDriver();

    /// Update driver system with current FMU continuous inputs.
    /// This function is called before advancing dynamics of the vehicle.
    void SynchronizeDriver(double time);

    /// Extract FMU continuous outputs from the driver system.
    /// This function is called after advancing dynamics of the vehicle.
    void CalculateDriverOutputs();

    std::shared_ptr<chrono::vehicle::ChPathSteeringController> steeringPID;  ///< steering controller
    std::shared_ptr<chrono::vehicle::ChSpeedController> speedPID;            ///< speed controller

    std::string path_file;   ///< name of file with path Bezier curve data
    double look_ahead_dist;  ///< look ahead distance for lateral PID controller
    double Kp_steering;      ///< steering PID proportional gain
    double Ki_steering;      ///< steering PID integral gain
    double Kd_steering;      ///< steering PID derivative gain

    double throttle_threshold;  ///< threshold throttle/braking control
    double Kp_speed;            ///< speed PID proportional gain
    double Ki_speed;            ///< speed PID integral gain
    double Kd_speed;            ///< speed PID derivative gain

    double step_size;  ///< integration step size

    double target_speed;                ///< current target speed (FMU input)
    chrono::ChFrameMoving<> ref_frame;  ///< vehicle reference frame (FMU input)

    chrono::ChVector<> init_loc;  ///< location of first path point (FMU constant output)
    double init_yaw;              ///< orientation of first path segment (FMU constant output)

    // Vehicle driver commands (FMU countinuous outputs)
    double steering;  ///< steering command, in [-1,1]
    double throttle;  ///< throttle command, in [0,1]
    double braking;   ///< braking command, in [0,1]

    fmi2Boolean vis;                     ///< enable/disable run-time visualization
    chrono::ChSystemSMC sys;             ///< containing system (visualization use only)
    chrono::geometry::ChAABB path_aabb;  ///< path axis-aligned bounding box
    int iballS;                          ///< ID for sentinel visualization shape
    int iballT;                          ///< ID for target visualization shape
#ifdef CHRONO_IRRLICHT
    std::shared_ptr<chrono::irrlicht::ChVisualSystemIrrlicht> vis_sys;
#endif
};

// Create an instance of this FMU
FmuComponentBase* fmi2Instantiate_getPointer(fmi2String instanceName, fmi2Type fmuType, fmi2String fmuGUID) {
    return new FmuComponent(instanceName, fmuType, fmuGUID);
}
