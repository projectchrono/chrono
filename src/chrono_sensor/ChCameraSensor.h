// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2019 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Asher Elmquist
// =============================================================================
//
// Container class for a camera sensor. This specifies a default ray tracing
// for cameras.
//
// =============================================================================

#ifndef CHCAMERASENSOR_H
#define CHCAMERASENSOR_H

#include "chrono_sensor/ChOptixSensor.h"

namespace chrono {
namespace sensor {

/// @addtogroup sensor_sensors
/// @{

/// The type of lens model that camera can use for rendering
enum CameraLensModelType {
    PINHOLE,   ///< traditional computer graphics ideal camera model.
    SPHERICAL  ///< Wide angle lens model based on single spherical lens.
};

/// Camera class
class CH_SENSOR_API ChCameraSensor : public ChOptixSensor {
  public:
    /// @brief Constructor for the base camera class that defaults to a pinhole lens model
    /// @param parent A shared pointer to a body on which the sensor should be attached.
    /// @param updateRate The desired update rate of the sensor in Hz.
    /// @param offsetPose The desired relative position and orientation of the sensor on the body.
    /// @param w The width of the image the camera should generate.
    /// @param h The height of the image the camera should generate.
    /// @param hFOV The horizontal field of view of the camera lens.
    /// @param lag The lag between when data collection is stopped and when data should be available to the user.
    /// @param exposure_time The time the camera should be collecting data for each frame.
    /// @param supersample_factor The number of rays that should be sampled per pixel for antialiasing.
    /// @param lens_model A enum specifying the desired lens model.
    ChCameraSensor(std::shared_ptr<chrono::ChBody> parent,     // object to which the sensor is attached
                   float updateRate,                           // rate at which the sensor updates
                   chrono::ChFrame<double> offsetPose,         // position of sensor relative to parent object
                   unsigned int w,                             // image width
                   unsigned int h,                             // image height
                   float hFOV,                                 // horizontal field of view
                   unsigned int supersample_factor = 1,        // number of samples per pixel for antialiasing
                   CameraLensModelType lens_model = PINHOLE);  // camera model to use for rendering

    /// camera class destructor
    ~ChCameraSensor();

    /// returns the camera's horizontal field of view. Vertical field of view is determined by the image aspect
    /// ratio and the lens model
    /// @return The horizontal field of view of the camera lens
    float GetHFOV() const { return m_hFOV; }

    /// returns the lens model type used for rendering
    /// @return An enum specifying which lens model is being used. (0: PINHOLE, 1: SPHERICAL)
    CameraLensModelType GetLensModelType() const { return m_lens_model_type; }

  private:
    float m_hFOV;                           ///< the horizontal field of view of the sensor
    unsigned int m_supersample_factor;      ///< super sampling factor for antialiasing
    CameraLensModelType m_lens_model_type;  ///< lens model used by the camera
};

/// @} sensor_sensors

}  // namespace sensor
}  // namespace chrono

#endif
