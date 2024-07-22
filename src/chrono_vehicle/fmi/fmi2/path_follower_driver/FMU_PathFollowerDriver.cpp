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
fmu_tools::fmi2::FmuComponentBase* fmu_tools::fmi2::fmi2InstantiateIMPL(fmi2String instanceName,
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
    : FmuChronoComponentBase(instanceName, fmuType, fmuGUID, fmuResourceLocation, functions, visible, loggingOn),
      render_frame(0) {
    // Initialize FMU type
    initializeType(fmuType);

    // Set initial/default values for FMU variables
    steering = 0;
    throttle = 0;
    braking = 0;

    target_speed = 0;

    step_size = 1e-3;

    out_path = ".";
    save_img = false;
    fps = 60;

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

    // Get default path file from FMU resources
    auto resources_dir = std::string(fmuResourceLocation).erase(0, 8);
    path_file = resources_dir + "/ISO_double_lane_change.txt";

#ifdef CHRONO_IRRLICHT
    if (visible)
        vis_sys = chrono_types::make_shared<irrlicht::ChVisualSystemIrrlicht>();
#else
    if (visible)
        std::cout << "The FMU was not built with run-time visualization support. Visualization disabled." << std::endl;
#endif

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

    // Set FIXED PARAMETERS for this FMU (I/O)
    AddFmuVariable(&out_path, "out_path", FmuVariable::Type::String, "1", "output directory",    //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);  //
    AddFmuVariable(&fps, "fps", FmuVariable::Type::Real, "1", "rendering frequency",             //
                   FmuVariable::CausalityType::parameter, FmuVariable::VariabilityType::fixed);  //

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

    // Set DISCRETE INPUTS for this FMU (I/O)
    AddFmuVariable((int*)(&save_img), "save_img", FmuVariable::Type::Boolean, "1", "trigger saving images",  //
                   FmuVariable::CausalityType::input, FmuVariable::VariabilityType::discrete);               //

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

    // Specify functions to process input variables (at beginning of step)
    AddPreStepFunction([this]() { this->SynchronizeDriver(this->GetTime()); });

    // Specify functions to calculate FMU outputs (at end of step)
    AddPostStepFunction([this]() { this->CalculateDriverOutputs(); });
}

void FmuComponent::CreateDriver() {
    std::cout << "Create driver FMU" << std::endl;
    std::cout << " Path file: " << path_file << std::endl;

    auto path = ChBezierCurve::Read(path_file, false);

    speedPID = chrono_types::make_shared<ChSpeedController>();
    steeringPID = chrono_types::make_shared<ChPathSteeringController>(path);

    steeringPID->SetLookAheadDistance(look_ahead_dist);
    steeringPID->SetGains(Kp_steering, Ki_steering, Kd_steering);
    speedPID->SetGains(Kp_speed, Ki_speed, Kd_speed);

    auto point0 = path->GetPoint(0);
    auto point1 = path->GetPoint(1);
    init_loc = point0;
    init_yaw = std::atan2(point1.y() - point0.y(), point1.x() - point0.x());

    ref_frame.SetPos(init_loc);
    ref_frame.SetRot(QuatFromAngleZ(init_yaw));

#ifdef CHRONO_IRRLICHT
    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetFixed(true);
    sys.AddBody(ground);

    auto num_points = static_cast<unsigned int>(path->GetNumPoints());
    auto path_asset = chrono_types::make_shared<ChVisualShapeLine>();
    path_asset->SetLineGeometry(chrono_types::make_shared<ChLineBezier>(path));
    path_asset->SetColor(ChColor(0.8f, 0.8f, 0.0f));
    path_asset->SetName("path");
    path_asset->SetNumRenderPoints(std::max<unsigned int>(2 * num_points, 400));
    ground->AddVisualShape(path_asset);

    path_aabb = path_asset->GetLineGeometry()->GetBoundingBox();
#endif
}

void FmuComponent::SynchronizeDriver(double time) {
    // Force a re-evaluation of the rotation matrix of the reference frame
    ref_frame.SetRot(ref_frame.GetRot());
}

void FmuComponent::CalculateDriverOutputs() {
    //// TODO
}

void FmuComponent::preModelDescriptionExport() {}

void FmuComponent::postModelDescriptionExport() {}

fmi2Status FmuComponent::enterInitializationModeIMPL() {
    return fmi2Status::fmi2OK;
}

fmi2Status FmuComponent::exitInitializationModeIMPL() {
    // Create the driver system
    CreateDriver();

    // Initialize runtime visualization (if requested and if available)
#ifdef CHRONO_IRRLICHT
    if (vis_sys) {
        std::cout << " Enable run-time visualization" << std::endl;

        // Calculate grid dimensions based on path AABB
        double spacing = 0.5;
        int grid_x = (int)std::ceil(path_aabb.Size().x() / spacing);
        int grid_y = 2 * (int)std::ceil(path_aabb.Size().y() / spacing);
        auto grid_pos = path_aabb.Center() - ChVector3d(0, 0, 0.05);
        auto grid_rot = QuatFromAngleZ(init_yaw);

        // Create run-time visualization system
        vis_sys->SetLogLevel(irr::ELL_NONE);
        vis_sys->SetJPEGQuality(100);
        vis_sys->AttachSystem(&sys);
        vis_sys->SetWindowSize(800, 800);
        vis_sys->SetWindowTitle("Path-follower Driver FMU (FMI 2.0)");
        vis_sys->SetCameraVertical(CameraVerticalDir::Z);
        vis_sys->AddGrid(spacing, spacing, grid_x, grid_y, ChCoordsys<>(grid_pos, grid_rot),
                         ChColor(0.31f, 0.43f, 0.43f));
        vis_sys->Initialize();
        vis_sys->AddCamera(ChVector3d(-4, 0, 0.5), ChVector3d(0, 0, 0));
        vis_sys->AddTypicalLights();

        // Create visualization objects for steering controller (sentinel and target points)
        auto sentinel_shape = chrono_types::make_shared<ChVisualShapeSphere>(0.1);
        auto target_shape = chrono_types::make_shared<ChVisualShapeSphere>(0.1);
        sentinel_shape->SetColor(ChColor(1, 0, 0));
        target_shape->SetColor(ChColor(0, 1, 0));
        iballS = vis_sys->AddVisualModel(sentinel_shape, ChFrame<>());
        iballT = vis_sys->AddVisualModel(target_shape, ChFrame<>());
    }
#endif
    return fmi2Status::fmi2OK;
}

fmi2Status FmuComponent::doStepIMPL(fmi2Real currentCommunicationPoint,
                                    fmi2Real communicationStepSize,
                                    fmi2Boolean noSetFMUStatePriorToCurrentPoint) {
    while (m_time < currentCommunicationPoint + communicationStepSize) {
        fmi2Real h = std::min((currentCommunicationPoint + communicationStepSize - m_time),
                              std::min(communicationStepSize, step_size));

        // Set the throttle and braking values based on the output from the speed controller.
        double out_speed = speedPID->Advance(ref_frame, target_speed, m_time, h);
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
        double out_steering = steeringPID->Advance(ref_frame, m_time, h);
        ChClampValue(out_steering, -1.0, 1.0);
        steering = out_steering;

#ifdef CHRONO_IRRLICHT
        if (vis_sys) {
            // Update system and all visual assets
            sys.Update(true);
            sys.SetChTime(m_time);

            // Update camera position
            auto x_dir = ref_frame.GetRotMat().GetAxisX();
            auto camera_target = ref_frame.GetPos();
            auto camera_pos = camera_target - 2.0 * x_dir + ChVector3d(0, 0, 0.5);
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

            if (save_img && m_time >= render_frame / fps) {
                std::ostringstream filename;
                filename << out_path << "/img_" << std::setw(4) << std::setfill('0') << render_frame + 1 << ".bmp";
                vis_sys->WriteImageToFile(filename.str());
                render_frame++;
            }
        }
#endif
        ////sendToLog("time: " + std::to_string(m_time) + "\n", fmi2Status::fmi2OK, "logAll");

        m_time += h;
    }

    return fmi2Status::fmi2OK;
}
