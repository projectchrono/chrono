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
// Authors: Asher Elmquist, Eric Brandt
// =============================================================================
//
// =============================================================================

#ifndef CHFILTERIMUUPDATE_H
#define CHFILTERIMUUPDATE_H

#include <memory>
#include <random>
#include <queue>
#include "chrono_sensor/filters/ChFilter.h"
#include "chrono/core/ChVector.h"
#include "chrono/core/ChMathematics.h"

namespace chrono {
namespace sensor {

// forward declaration
class ChSensor;
class ChNoiseModel;
class ChAccelerometerSensor;
class ChGyroscopeSensor;
class ChMagnetometerSensor;

/// @addtogroup sensor_filters
/// @{

/// Class for generating IMU data
class CH_SENSOR_API ChFilterAccelerometerUpdate : public ChFilter {
  public:
    /// Class constructor
    /// @param noise_model The noise model to use when augmenting the IMU data
    ChFilterAccelerometerUpdate(std::shared_ptr<ChNoiseModel> noise_model);

    /// Apply function. Generates IMU data.
    /// @param pSensor A pointer to the sensor on which the filter is attached.
    /// @param bufferInOut A buffer that is passed into the filter.
    virtual void Apply();

    /// Initializes all data needed by the filter access apply function.
    /// @param pSensor A pointer to the sensor.
    /// @param bufferInOut pointer to the process buffer
    virtual void Initialize(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut);

  private:
    std::shared_ptr<ChAccelerometerSensor> m_accSensor;
    std::shared_ptr<SensorHostAccelBuffer> m_bufferOut;  ///< For holding generated IMU data
    std::shared_ptr<ChNoiseModel> m_noise_model;      ///< The noise model for augmenting data
};

/// Class for generating IMU data
class CH_SENSOR_API ChFilterGyroscopeUpdate : public ChFilter {
  public:
    /// Class constructor
    /// @param noise_model The noise model to use when augmenting the IMU data
    ChFilterGyroscopeUpdate(std::shared_ptr<ChNoiseModel> noise_model);

    /// Apply function. Generates IMU data.
    /// @param pSensor A pointer to the sensor on which the filter is attached.
    /// @param bufferInOut A buffer that is passed into the filter.
    virtual void Apply();

    /// Initializes all data needed by the filter access apply function.
    /// @param pSensor A pointer to the sensor.
    /// @param bufferInOut pointer to the process buffer
    virtual void Initialize(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut);

  private:
    std::shared_ptr<ChGyroscopeSensor> m_gyroSensor;
    std::shared_ptr<SensorHostGyroBuffer> m_bufferOut;  ///< For holding generated IMU data
    std::shared_ptr<ChNoiseModel> m_noise_model;     ///< The noise model for augmenting data
};

/// Class for generating IMU data
class CH_SENSOR_API ChFilterMagnetometerUpdate : public ChFilter {
  public:
    /// Class constructor
    /// @param noise_model The noise model to use when augmenting the IMU data
    /// @param gps_reference The GPS reference location for the simulation origin
    ChFilterMagnetometerUpdate(std::shared_ptr<ChNoiseModel> noise_model, ChVector<double> gps_reference);

    /// Apply function. Generates IMU data.
    /// @param pSensor A pointer to the sensor on which the filter is attached.
    /// @param bufferInOut A buffer that is passed into the filter.
    virtual void Apply();

    /// Initializes all data needed by the filter access apply function.
    /// @param pSensor A pointer to the sensor.
    /// @param bufferInOut pointer to the process buffer
    virtual void Initialize(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut);

  private:
    std::shared_ptr<ChMagnetometerSensor> m_magSensor;
    std::shared_ptr<SensorHostMagnetBuffer> m_bufferOut;  ///< For holding generated IMU data
    std::shared_ptr<ChNoiseModel> m_noise_model;          ///< The noise model for augmenting data
    ChVector<double> m_gps_reference;                     ///< gps reference location

    const double theta_0 = 80.65 * CH_C_DEG_TO_RAD;  // latitude of magnetic pole
    const double phi_0 = -72.68 * CH_C_DEG_TO_RAD;   // longitude of magnetic pole
    const double B_0 = 0.305;                        // mean magnetic field at magnetic equator (in Gauss)
};

/// @}

}  // namespace sensor
}  // namespace chrono

#endif
