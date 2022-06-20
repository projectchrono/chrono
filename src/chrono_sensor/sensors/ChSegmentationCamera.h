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
//
// =============================================================================

#ifndef CHSEGMENTATIONCAMERA_H
#define CHSEGMENTATIONCAMERA_H

// will use same parameters as camera
#include "chrono_sensor/sensors/ChCameraSensor.h"

namespace chrono {
namespace sensor {

/// @addtogroup sensor_sensors
/// @{

/// Camera class
class CH_SENSOR_API ChSegmentationCamera : public ChOptixSensor {
  public:
    /// @brief Constructor for a segmentation camera that defaults to a pinhole lens model
    /// @param parent A shared pointer to a body on which the sensor should be attached.
    /// @param updateRate The desired update rate of the sensor in Hz.
    /// @param offsetPose The desired relative position and orientation of the sensor on the body.
    /// @param w The width of the image the camera should generate.
    /// @param h The height of the image the camera should generate.
    /// @param hFOV The horizontal field of view of the camera lens.
    /// @param lens_model A enum specifying the desired lens model.
    ChSegmentationCamera(std::shared_ptr<chrono::ChBody> parent,  // object to which the sensor is attached
                         float updateRate,                        // rate at which the sensor updates
                         chrono::ChFrame<double> offsetPose,      // position of sensor relative to parent object
                         unsigned int w,                          // image width
                         unsigned int h,                          // image height
                         float hFOV,                              // horizontal field of view
                         CameraLensModelType lens_model = CameraLensModelType::PINHOLE);  // lens model type

    /// camera class destructor
    ~ChSegmentationCamera();

    /// returns the camera's horizontal field of view. Vertical field of view is determined by the image aspect
    /// ratio and the lens model
    /// @return The horizontal field of view of the camera lens
    float GetHFOV() const { return m_hFOV; }

    /// returns the lens model type used for rendering
    /// @return An enum specifying which lens model is being used. (0: PINHOLE, 1: SPHERICAL)
    CameraLensModelType GetLensModelType() const { return m_lens_model_type; }

    /// returns the lens model parameters
    /// @return LensParams of lens parameters. Will default to zeros for any terms not used. These are coverted for the inverse model
    LensParams GetLensParameters() const { return m_lens_parameters; }

    /// Sets the parameters for a radial lens distortion model
    /// Parameters should be given for the forward model
    /// The backward distortion model will the used and calculated from the forward parameters given
    /// @param params the set of 3 radial parameters (k1,k2,k3)
    void SetRadialLensParameters(ChVector<float> params);

  private:
    float m_hFOV;                           ///< the horizontal field of view of the sensor
    CameraLensModelType m_lens_model_type;  ///< lens model used by the camera
    LensParams m_lens_parameters;      ///< lens parameters when applicable
};

/// @} sensor_sensors

}  // namespace sensor
}  // namespace chrono

#endif
