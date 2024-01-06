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
// =============================================================================

#include <cassert>
#include <algorithm>

#include "chrono/geometry/ChLineBezier.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

// #define FMI2_FUNCTION_PREFIX MyModel_
#include "FMU_PathFollowerDriver.h"

using namespace chrono;
using namespace chrono::vehicle;

FmuComponent::FmuComponent(fmi2String _instanceName, fmi2Type _fmuType, fmi2String _fmuGUID)
    : FmuChronoComponentBase(_instanceName, _fmuType, _fmuGUID) {
    // Initialize FMU type
    initializeType(_fmuType);

    // Set initial/default values for FMU variables
    steering = 0;
    throttle = 0;
    braking = 0;

    target_speed = 0;

    step_size = 1e-3;
    vis = 0;

    look_ahead_dist = 5.0;
    Kp_steering = 0.8;
    Ki_steering = 0.0;
    Kd_steering = 0.0;

    throttle_threshold = 0.2;
    Kp_speed = 0.4;
    Ki_speed = 0.0;
    Kd_speed = 0.0;

    init_loc = VNULL;
    init_yaw = 0;

    // Set configuration flags for this FMU
    AddFmuVariable(&vis, "vis", FmuVariable::Type::Boolean, "1", "enable visualization",         //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);  //

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
    AddFmuVariable(&Kd_steering, "Kd_steering", FmuVariable::Type::Real, "1",                    //
                   "derivative gain, steering controller",                                       //
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
    AddFmuVariable(&Kd_speed, "Kd_speed", FmuVariable::Type::Real, "1",                          //
                   "derivative gain, speed controller",                                          //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);  //

    AddFmuVariable(&step_size, "step_size", FmuVariable::Type::Real, "s", "integration step size",  //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);     //

    // Set CONSTANT OUTPUT for this FMU
    AddFmuVecVariable(init_loc, "init_loc", "m", "location of first path point",                                //
                      FmuVariable::CausalityType::output, FmuVariable::VariabilityType::constant);              //
    AddFmuVariable(&init_yaw, "init_yaw", FmuVariable::Type::Real, "rad", "orientation of first path segment",  //
                   FmuVariable::CausalityType::output, FmuVariable::VariabilityType::constant);                 //

    // Set CONTINOUS INPUTS for this FMU
    AddFmuFrameMovingVariable(ref_frame, "ref_frame", "m", "m/s", "reference frame",                         //
                              FmuVariable::CausalityType::input, FmuVariable::VariabilityType::continuous);  //
    AddFmuVariable(&target_speed, "target_speed", FmuVariable::Type::Real, "m/s", "target speed",            //
                   FmuVariable::CausalityType::input, FmuVariable::VariabilityType::continuous);             //

    // Set CONTINOUS OUTPUTS for this FMU
    AddFmuVariable(&steering, "steering", FmuVariable::Type::Real, "1", "steering command",        //
                   FmuVariable::CausalityType::output, FmuVariable::VariabilityType::continuous);  //
    AddFmuVariable(&throttle, "throttle", FmuVariable::Type::Real, "1", "throttle command",        //
                   FmuVariable::CausalityType::output, FmuVariable::VariabilityType::continuous);  //
    AddFmuVariable(&braking, "braking", FmuVariable::Type::Real, "1", "braking command",           //
                   FmuVariable::CausalityType::output, FmuVariable::VariabilityType::continuous);  //

    // Specify functions to process input variables (at beginning of step)
    preStepCallbacks.push_back([this]() { this->SynchronizeDriver(this->GetTime()); });

    // Specify functions to calculate FMU outputs (at end of step)
    postStepCallbacks.push_back([this]() { this->CalculateDriverOutputs(); });
}

void FmuComponent::CreateDriver() {
    std::cout << "Create driver FMU" << std::endl;
    std::cout << " Path file: " << path_file << std::endl;

    auto path = ChBezierCurve::read(path_file, false);

    speedPID = chrono_types::make_shared<ChSpeedController>();
    steeringPID = chrono_types::make_shared<ChPathSteeringController>(path);

    steeringPID->SetLookAheadDistance(look_ahead_dist);
    steeringPID->SetGains(Kp_steering, Ki_steering, Kd_steering);
    speedPID->SetGains(Kp_speed, Ki_speed, Kd_speed);

    auto point0 = path->getPoint(0);
    auto point1 = path->getPoint(1);
    init_loc = point0;
    init_yaw = std::atan2(point1.y() - point0.y(), point1.x() - point0.x());

    ref_frame.SetPos(init_loc);
    ref_frame.SetRot(Q_from_AngZ(init_yaw));

#ifdef CHRONO_IRRLICHT
    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetBodyFixed(true);
    sys.AddBody(ground);

    auto num_points = static_cast<unsigned int>(path->getNumPoints());
    auto path_asset = chrono_types::make_shared<ChVisualShapeLine>();
    path_asset->SetLineGeometry(chrono_types::make_shared<geometry::ChLineBezier>(path));
    path_asset->SetColor(ChColor(0.8f, 0.8f, 0.0f));
    path_asset->SetName("path");
    path_asset->SetNumRenderPoints(std::max<unsigned int>(2 * num_points, 400));
    ground->AddVisualShape(path_asset);

    path_aabb = path_asset->GetLineGeometry()->GetBoundingBox();
#endif
}

void FmuComponent::SynchronizeDriver(double time) {
    // Set the rotation matrix of the reference frame
    ref_frame.GetA().Set_A_quaternion(ref_frame.GetRot());
}

void FmuComponent::CalculateDriverOutputs() {
    //// TODO
}

void FmuComponent::_preModelDescriptionExport() {}

void FmuComponent::_postModelDescriptionExport() {}

void FmuComponent::_enterInitializationMode() {}

void FmuComponent::_exitInitializationMode() {
    // Create the driver system
    CreateDriver();

    // Initialize runtime visualization (if requested and if available)
    if (vis) {
#ifdef CHRONO_IRRLICHT
        std::cout << " Enable run-time visualization" << std::endl;

        // Calculate grid dimensions based on path AABB
        double spacing = 0.5;
        int grid_x = (int)std::ceil(path_aabb.Size().x() / spacing);
        int grid_y = 2 * (int)std::ceil(path_aabb.Size().y() / spacing);
        auto grid_pos = path_aabb.Center() - ChVector<>(0, 0, 0.05);
        auto grid_rot = Q_from_AngZ(init_yaw);

        // Create run-time visualization system
        vis_sys = chrono_types::make_shared<irrlicht::ChVisualSystemIrrlicht>();
        vis_sys->SetLogLevel(irr::ELL_NONE);
        vis_sys->AttachSystem(&sys);
        vis_sys->SetWindowSize(800, 600);
        vis_sys->SetWindowTitle("Path-follower Driver FMU");
        vis_sys->SetCameraVertical(CameraVerticalDir::Z);
        vis_sys->AddGrid(spacing, spacing, grid_x, grid_y, ChCoordsys<>(grid_pos, grid_rot),
                         ChColor(0.31f, 0.43f, 0.43f));
        vis_sys->Initialize();
        vis_sys->AddCamera(ChVector<>(-4, 0, 0.5), ChVector<>(0, 0, 0));
        vis_sys->AddTypicalLights();

        // Create visualization objects for steering controller (sentinel and target points)
        auto sentinel_shape = chrono_types::make_shared<ChVisualShapeSphere>(0.1);
        auto target_shape = chrono_types::make_shared<ChVisualShapeSphere>(0.1);
        sentinel_shape->SetColor(ChColor(1, 0, 0));
        target_shape->SetColor(ChColor(0, 1, 0));
        iballS = vis_sys->AddVisualModel(sentinel_shape, ChFrame<>());
        iballT = vis_sys->AddVisualModel(target_shape, ChFrame<>());
#else
        std::cout << " Run-time visualization not available" << std::endl;
#endif
    }
}

fmi2Status FmuComponent::_doStep(fmi2Real currentCommunicationPoint,
                                 fmi2Real communicationStepSize,
                                 fmi2Boolean noSetFMUStatePriorToCurrentPoint) {
    while (time < currentCommunicationPoint + communicationStepSize) {
        fmi2Real h = std::min((currentCommunicationPoint + communicationStepSize - time),
                              std::min(communicationStepSize, step_size));

        // Set the throttle and braking values based on the output from the speed controller.
        double out_speed = speedPID->Advance(ref_frame, target_speed, time, h);
        ChClampValue(out_speed, -1.0, 1.0);

        if (out_speed > 0) {
            // Vehicle moving too slow
            braking = 0;
            throttle = out_speed;
        } else if (throttle > throttle_threshold) {
            // Vehicle moving too fast: reduce throttle
            braking = 0;
            throttle = 1 + out_speed;
        } else {
            // Vehicle moving too fast: apply brakes
            braking = -out_speed;
            throttle = 0;
        }

        // Set the steering value based on the output from the steering controller.
        double out_steering = steeringPID->Advance(ref_frame, time, h);
        ChClampValue(out_steering, -1.0, 1.0);
        steering = out_steering;

        if (vis) {
#ifdef CHRONO_IRRLICHT
            // Update system and all visual assets
            sys.Update(true);

            // Update camera position
            auto x_dir = ref_frame.GetA().Get_A_Xaxis();
            auto camera_target = ref_frame.GetPos();
            auto camera_pos = camera_target - 2.0 * x_dir + ChVector<>(0, 0, 0.5);
            vis_sys->UpdateCamera(camera_pos, camera_target);

            // Update sentinel and target location markers for the path-follower controller.
            vis_sys->UpdateVisualModel(iballS, ChFrame<>(steeringPID->GetSentinelLocation()));
            vis_sys->UpdateVisualModel(iballT, ChFrame<>(steeringPID->GetTargetLocation()));

            auto status = vis_sys->Run();
            if (!status)
                return fmi2Discard;
            vis_sys->BeginScene(true, true, ChColor(0.33f, 0.6f, 0.78f));
            vis_sys->Render();
            vis_sys->RenderFrame(ref_frame);
            vis_sys->EndScene();
#endif
        }
        ////sendToLog("time: " + std::to_string(time) + "\n", fmi2Status::fmi2OK, "logAll");

        time = time + h;
    }

    return fmi2Status::fmi2OK;
}
