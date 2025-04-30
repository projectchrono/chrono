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
// The driver model includes a lateral path-following PI controller and a
// longitudinal PI cruise controller.
//
// =============================================================================

#include <cassert>
#include <algorithm>
#include <iomanip>

#include "chrono/geometry/ChLineBezier.h"
#include "chrono/utils/ChUtils.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono/utils/ChUtils.h"

#include "FMU_PathFollowerDriver.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::fmi2;

// -----------------------------------------------------------------------------

// Create an instance of this FMU
fmu_forge::fmi2::FmuComponentBase* fmu_forge::fmi2::fmi2InstantiateIMPL(fmi2String instanceName,
                                                                        fmi2Type fmuType,
                                                                        fmi2String fmuGUID,
                                                                        fmi2String fmuResourceLocation,
                                                                        const fmi2CallbackFunctions* functions,
                                                                        fmi2Boolean visible,
                                                                        fmi2Boolean loggingOn) {
    return new FmuComponent(instanceName, fmuType, fmuGUID, fmuResourceLocation, functions, visible, loggingOn);
}

// -----------------------------------------------------------------------------

FmuComponent::FmuComponent(fmi2String instanceName,
                           fmi2Type fmuType,
                           fmi2String fmuGUID,
                           fmi2String fmuResourceLocation,
                           const fmi2CallbackFunctions* functions,
                           fmi2Boolean visible,
                           fmi2Boolean loggingOn)
    : FmuChronoComponentBase(instanceName, fmuType, fmuGUID, fmuResourceLocation, functions, visible, loggingOn) {
    // Initialize FMU type
    initializeType(fmuType);

    // Set initial/default values for FMU variables
    steering = 0;
    throttle = 0;
    braking = 0;

    target_speed = 0;

    look_ahead_dist = 5.0;
    Kp_steering = 0.8;
    Ki_steering = 0.0;

    throttle_threshold = 0.2;
    Kp_speed = 0.4;
    Ki_speed = 0.0;

    // Steering rate limiter
    crt_time = -1;
    prev_time = -1;
    rising_rate_steering = 1.0;

    // Get default path file from FMU resources
    auto resources_dir = std::string(fmuResourceLocation).erase(0, 8);
    path_file = resources_dir + "/ISO_double_lane_change.txt";

    // Set initial conditions for underlying ODEs (PID integral errors)
    q = {0.0, 0.0};

    // Set FIXED PARAMETERS for this FMU
    //// TODO: units for gains
    AddFmuVariable(&path_file, "path_file", FmuVariable::Type::String, "1", "path specififcation file",  //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);          //

    AddFmuVariable(&look_ahead_dist, "look_ahead_dist", FmuVariable::Type::Real, "1",            //
                   "lookahead distance, steering controller",                                    //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);  //
    AddFmuVariable(&Kp_steering, "Kp_steering", FmuVariable::Type::Real, "1",                    //
                   "proportional gain, steering controller",                                     //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);  //
    AddFmuVariable(&Ki_steering, "Ki_steering", FmuVariable::Type::Real, "1",                    //
                   "integral gain, steering controller",                                         //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);  //

    AddFmuVariable(&throttle_threshold, "throttle_threshold", FmuVariable::Type::Real, "1",      //
                   "threshold for throttle/brake control, speed controller",                     //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);  //
    AddFmuVariable(&Kp_speed, "Kp_speed", FmuVariable::Type::Real, "1",                          //
                   "proportional gain, speed controller",                                        //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);  //
    AddFmuVariable(&Ki_speed, "Ki_speed", FmuVariable::Type::Real, "1",                          //
                   "integral gain, speed controller",                                            //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);  //

    // Set CONTINOUS INPUTS for this FMU
    AddFmuFrameMovingVariable(ref_frame, "ref_frame", "m", "m/s", "reference frame",                         //
                              FmuVariable::CausalityType::input, FmuVariable::VariabilityType::continuous);  //
    AddFmuVariable(&target_speed, "target_speed", FmuVariable::Type::Real, "m/s", "target speed",            //
                   FmuVariable::CausalityType::input, FmuVariable::VariabilityType::continuous);             //

    // Set CONTINOUS OUTPUTS for this FMU
    AddFmuVariable(&steering, "steering", FmuVariable::Type::Real, "1", "steering command",       //
                   FmuVariable::CausalityType::output, FmuVariable::VariabilityType::continuous,  //
                   FmuVariable::InitialType::exact);                                              //
    AddFmuVariable(&throttle, "throttle", FmuVariable::Type::Real, "1", "throttle command",       //
                   FmuVariable::CausalityType::output, FmuVariable::VariabilityType::continuous,  //
                   FmuVariable::InitialType::exact);                                              //
    AddFmuVariable(&braking, "braking", FmuVariable::Type::Real, "1", "braking command",          //
                   FmuVariable::CausalityType::output, FmuVariable::VariabilityType::continuous,  //
                   FmuVariable::InitialType::exact);                                              //

    AddFmuVecVariable(sentinel_loc, "sentinel_loc", "m", "sentinel location",                         //
                      FmuVariable::CausalityType::output, FmuVariable::VariabilityType::continuous,   //
                      FmuVariable::InitialType::exact);                                               //
    AddFmuVecVariable(target_loc, "target_loc", "m", "target location",                               //
                      FmuVariable::CausalityType::output, FmuVariable::VariabilityType::continuous,   //
                      FmuVariable::InitialType::exact);                                               //

    // Set states and derivatives
    AddFmuVariable(&q[0], "err_lat", FmuVariable::Type::Real, "1", "lateral integral error",        //
                   FmuVariable::CausalityType::output, FmuVariable::VariabilityType::continuous,    //
                   FmuVariable::InitialType::exact);                                                //
    AddFmuVariable(&q[0], "err_long", FmuVariable::Type::Real, "1", "longitudinal integral error",  //
                   FmuVariable::CausalityType::output, FmuVariable::VariabilityType::continuous,    //
                   FmuVariable::InitialType::exact);                                                //

    AddFmuVariable(&qd[0], "der(err_lat)", FmuVariable::Type::Real, "1",                         //
                   "derivative of lateral integral error",                                       //
                   FmuVariable::CausalityType::local, FmuVariable::VariabilityType::continuous,  //
                   FmuVariable::InitialType::calculated);                                        //
    AddFmuVariable(&qd[0], "der(err_long)", FmuVariable::Type::Real, "1",                        //
                   "derivative of longitudinal integral error",                                  //
                   FmuVariable::CausalityType::local, FmuVariable::VariabilityType::continuous,  //
                   FmuVariable::InitialType::calculated);                                        //

    // Specify state derivatives
    DeclareStateDerivative("der(err_lat)", "err_lat", {"err_lat"});
    DeclareStateDerivative("der(err_long)", "err_long", {"err_long"});

    // Specify functions to process input variables (called before getDerivatives)
    AddPreStepFunction([this]() { this->SynchronizeDriver(this->GetTime()); });

    // Specify functions to calculate FMU outputs (called after getDerivatives)
    AddPostStepFunction([this]() { this->CalculateDriverOutputs(); });
}

void FmuComponent::preModelDescriptionExport() {}

void FmuComponent::postModelDescriptionExport() {}

fmi2Status FmuComponent::enterInitializationModeIMPL() {
    return fmi2Status::fmi2OK;
}

fmi2Status FmuComponent::exitInitializationModeIMPL() {
    // Create the path and tracker
    auto path = ChBezierCurve::Read(path_file, false);
    tracker = std::unique_ptr<ChBezierCurveTracker>(new ChBezierCurveTracker(path));

    return fmi2Status::fmi2OK;
}

fmi2Status FmuComponent::getContinuousStatesIMPL(fmi2Real x[], size_t nx) {
    for (size_t i = 0; i < nx; i++) {
        x[i] = q[i];
    }

    return fmi2Status::fmi2OK;
}

fmi2Status FmuComponent::setContinuousStatesIMPL(const fmi2Real x[], size_t nx) {
    for (size_t i = 0; i < nx; i++) {
        q[i] = x[i];
    }

    return fmi2Status::fmi2OK;
}

fmi2Status FmuComponent::getDerivativesIMPL(fmi2Real derivatives[], size_t nx) {
    qd[0] = q[0];
    qd[1] = q[1];

    derivatives[0] = qd[0];
    derivatives[1] = q[1];

    return fmi2Status::fmi2OK;
}

void FmuComponent::SynchronizeDriver(double time) {
    crt_time = time;

    // Force a re-evaluation of the rotation matrix of the reference frame
    ref_frame.SetRot(ref_frame.GetRot());
}

void FmuComponent::CalculateDriverOutputs() {
    CalculateLateralControlerOutputs();
    CalculateLongitudinalControlerOutputs();
}

void FmuComponent::CalculateLateralControlerOutputs() {
    // Calculate current sentinel location.  This is a point at the look-ahead distance in front of the vehicle.
    sentinel_loc = ref_frame.TransformPointLocalToParent(ChVector3d(look_ahead_dist, 0, 0));

    // Calculate target location
    tracker->CalcClosestPoint(sentinel_loc, target_loc);

    // Project error onto the (x,y) horizontal plane.
    ChVector3d err_vec = target_loc - sentinel_loc;
    err_vec.z() = 0;

    // Calculate the sign of the angle between the projections of the sentinel
    // vector and the target vector (with origin at vehicle location).
    ChVector3d sentinel_vec = sentinel_loc - ref_frame.GetPos();
    sentinel_vec.z() = 0;
    ChVector3d target_vec = target_loc - ref_frame.GetPos();
    target_vec.z() = 0;

    double temp = Vcross(sentinel_vec, target_vec).z();

    // Calculate current proportional error
    double err_p = ChSignum(temp) * err_vec.Length();

    // Extract current integral error from states
    double err_i = q[0];

    // PI controller output
    double out = Kp_steering * err_p + Ki_steering * err_i;

    /*
    // Rate limiter
    if (prev_time > 0) {
        double rate = (out - prev_steering_output) / (crt_time - prev_time);
        if (rate > rising_rate_steering)
            out = prev_steering_output + rising_rate_steering * (crt_time - prev_time);
        if (rate < -rising_rate_steering)
            out = prev_steering_output - rising_rate_steering * (crt_time - prev_time);
    }
    prev_time = crt_time;
    prev_steering_output = out;
    */

    // Convert to steering command
    steering = ChClamp(out, -1.0, +1.0);
}

void FmuComponent::CalculateLongitudinalControlerOutputs() {
    // Current vehicle speed.
    double speed = Vdot(ref_frame.GetPosDt(), ref_frame.GetRotMat().GetAxisX());

    // Calculate current proportional error
    double err_p = target_speed - speed;

    // Extract current integral error from states
    double err_i = q[1];

    // PI controller output
    double out = Kp_speed * err_p + Ki_speed * err_i;
    ChClampValue(out, -1.0, +1.0);

    // Convert to throttle/braking commands
    if (out > 0) {
        // Vehicle moving too slow
        braking = 0;
        throttle = out;
    } else if (throttle > throttle_threshold) {
        // Vehicle moving too fast: reduce throttle
        braking = 0;
        throttle = 1 + out;
    } else {
        // Vehicle moving too fast: apply brakes
        braking = -out;
        throttle = 0;
    }
}
