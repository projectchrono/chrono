// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Kyle Sha
// =============================================================================
// Platform- and backend-independent orbit-camera controller. Holds an orbit
// state (yaw / pitch / distance around a target) and turns pointer input deltas
// into a Chrono sensor offset pose. Any windowing layer (Cocoa, GLFW, X11, ...)
// can feed it input; it depends only on chrono core math, so the same controller
// drives the Metal visualizer today and could drive the OpenGL/Vulkan one.
//
// The produced ChFrame is a *sensor offset pose* (relative to the sensor's
// parent body), using the Chrono::Sensor camera convention: forward = +X_local,
// up = +Z_local, right = -Y_local, horizontal FOV.
// =============================================================================

#ifndef CH_ORBIT_CAMERA_CONTROL_H
#define CH_ORBIT_CAMERA_CONTROL_H

#include <cmath>
#include "chrono/core/ChFrame.h"
#include "chrono/core/ChRotation.h"

namespace chrono {
namespace sensor {

/// @addtogroup sensor_metal
/// @{

class ChOrbitCameraControl {
  public:
    // Orbit state (radians / meters). Defaults place the camera behind and
    // slightly above a target ~1 m up (a natural chase/inspection pose).
    double yaw = CH_PI;   // azimuth around +Z; CH_PI = behind a +X-facing body
    double pitch = 0.22;  // elevation above the horizon
    double dist = 8.0;    // distance from the target
    ChVector3d target = ChVector3d(0, 0, 1.0);

    // Limits
    double min_pitch = -1.30, max_pitch = 1.45;
    double min_dist = 1.5, max_dist = 120.0;

    // Sensitivities (per input unit)
    double yaw_gain = 0.010, pitch_gain = 0.010, zoom_gain = 0.05;

    /// Apply pointer input: horizontal/vertical drag deltas (pixels) and a
    /// scroll delta (wheel notches; positive zooms in).
    void ApplyInput(double drag_dx, double drag_dy, double scroll) {
        yaw += drag_dx * yaw_gain;
        pitch += drag_dy * pitch_gain;
        if (pitch < min_pitch)
            pitch = min_pitch;
        if (pitch > max_pitch)
            pitch = max_pitch;
        dist *= std::pow(0.9, scroll * zoom_gain);
        if (dist < min_dist)
            dist = min_dist;
        if (dist > max_dist)
            dist = max_dist;
    }

    /// Current offset pose (camera orbiting `target`, looking at it).
    ChFrame<double> OffsetPose() const {
        // unit vector from target out to the camera
        ChVector3d off(std::cos(pitch) * std::cos(yaw), std::cos(pitch) * std::sin(yaw), std::sin(pitch));
        ChVector3d pos = target + off * dist;
        ChVector3d fwd = -off;  // camera looks back at the target
        double fy = std::atan2(fwd.y(), fwd.x());
        double fp = std::asin(fwd.z());
        return ChFrame<double>(pos, QuatFromAngleZ(fy) * QuatFromAngleY(-fp));
    }
};

/// @} sensor_metal
}  // namespace sensor
}  // namespace chrono

#endif
