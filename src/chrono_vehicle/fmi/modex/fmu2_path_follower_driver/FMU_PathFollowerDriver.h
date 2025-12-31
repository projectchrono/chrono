// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2025 projectchrono.org
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
// Model exchange FMU with a vehicle path-follower driver model.
// The driver model includes a lateral path-following PID controller and a
// longitudinal PID cruise controller.
//
// The wrapped Chrono::Vehicle driver model is defined through a data file
// specifying the path for the lateral controller.
//
// This driver FMU expects an input that provides the current vehicle reference
// frame (of type ChFrameMoving). It generates vehicle control inputs (throttle,
// braking, and steering).
//
// =============================================================================

#pragma once

#include "chrono/core/ChBezierCurve.h"
#include "chrono/geometry/ChGeometry.h"

#include "chrono_fmi/fmi2/ChFmuToolsExport.h"

class FmuComponent : public chrono::fmi2::FmuChronoComponentBase {
  public:
    FmuComponent(fmi2String instanceName,
                 fmi2Type fmuType,
                 fmi2String fmuGUID,
                 fmi2String fmuResourceLocation,
                 const fmi2CallbackFunctions* functions,
                 fmi2Boolean visible,
                 fmi2Boolean loggingOn);
    ~FmuComponent() {}

  private:
    virtual bool is_cosimulation_available() const override { return false; }
    virtual bool is_modelexchange_available() const override { return true; }

    virtual fmi2Status enterInitializationModeIMPL() override;
    virtual fmi2Status exitInitializationModeIMPL() override;

    virtual fmi2Status getContinuousStatesIMPL(fmi2Real x[], size_t nx) override;
    virtual fmi2Status setContinuousStatesIMPL(const fmi2Real x[], size_t nx) override;
    virtual fmi2Status getDerivativesIMPL(fmi2Real derivatives[], size_t nx) override;

    virtual void preModelDescriptionExport() override;
    virtual void postModelDescriptionExport() override;

    /// Update driver system with current FMU continuous inputs.
    /// This function is called before getDerivatives.
    void SynchronizeDriver(double time);

    /// Extract FMU continuous outputs from the driver system.
    /// This function is called after getDerivatives.
    void CalculateDriverOutputs();

    void CalculateLateralControlerOutputs();
    void CalculateLongitudinalControlerOutputs();

    // FMU parameters
    std::string path_file;      ///< name of file with path Bezier curve data
    double look_ahead_dist;     ///< look ahead distance for lateral PID controller
    double Kp_steering;         ///< steering PID proportional gain
    double Ki_steering;         ///< steering PID integral gain
    double throttle_threshold;  ///< threshold throttle/braking control
    double Kp_speed;            ///< speed PID proportional gain
    double Ki_speed;            ///< speed PID integral gain

    // FMU inputs
    double target_speed;                ///< current target speed
    chrono::ChFrameMoving<> ref_frame;  ///< vehicle reference frame

    // FMU outputs
    double steering;  ///< (FMU countinuous output) steering command, in [-1,1]
    double throttle;  ///< (FMU countinuous output) throttle command, in [0,1]
    double braking;   ///< (FMU countinuous output) braking command, in [0,1]

    chrono::ChVector3d sentinel_loc;  ///< (FMU continuous output) current sentinel point location
    chrono::ChVector3d target_loc;    ///< (FMU continuous output) current target point location

    // FMU states
    chrono::ChVectorN<double, 2> q;   ///< states
    chrono::ChVectorN<double, 2> qd;  ///< state derivatives

    std::unique_ptr<chrono::ChBezierCurveTracker> tracker;  ///< path tracker

    // Steering rate limiter
    double rising_rate_steering;
    double crt_time;
    double prev_time; 
    double prev_steering_output;
};
