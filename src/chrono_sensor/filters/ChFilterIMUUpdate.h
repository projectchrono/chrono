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

namespace chrono {
namespace sensor {

// forward declaration
class ChSensor;

/// @addtogroup sensor_filters
/// @{

/// IMU Noise Model base class
class CH_SENSOR_API ChIMUNoiseModel {
  public:
    /// Class constructor
    ChIMUNoiseModel(){};

    /// Class destructor
    ~ChIMUNoiseModel(){};

    /// Function for adding noise to the IMU
    /// @param gyro Gyroscope data to augment
    /// @param acc Acceleratometer data to augment
    virtual void AddNoise(chrono::ChVector<float>& gyro, chrono::ChVector<float>& acc) = 0;
};

/// IMU Noise model: no noise
class CH_SENSOR_API ChIMUNoiseNone : public ChIMUNoiseModel {
  public:
    /// Class constructor
    ChIMUNoiseNone() {}
    /// Class destructor
    ~ChIMUNoiseNone() {}

    /// Function for adding noise to the IMU. This is empty and adds no noise.
    /// @param gyro Gyroscope data to augment
    /// @param acc Acceleratometer data to augment
    virtual void AddNoise(chrono::ChVector<float>& gyro, chrono::ChVector<float>& acc) {}
};

/// IMU Noise model: gaussian drifting noise with noncorrelated equal distributions
class CH_SENSOR_API ChIMUNoiseNormalDrift : public ChIMUNoiseModel {
  public:
    /// Class constructor
    ChIMUNoiseNormalDrift(float updateRate,
                          float g_mean,
                          float g_stdev,
                          float g_bias_drift,
                          float g_tau_drift,
                          float a_mean,
                          float a_stdev,
                          float a_bias_drift,
                          float a_tau_drift);

    /// Class destructor
    ~ChIMUNoiseNormalDrift(){};

    /// Function for adding drifting noise to the IMU
    /// @param gyro Gyroscope data to augment
    /// @param acc Acceleratometer data to augment
    virtual void AddNoise(chrono::ChVector<float>& gyro, chrono::ChVector<float>& acc);

  private:
    std::minstd_rand m_generator;                        ///< random number generator
    ChVector<float> m_gyro_bias_prev = {0.f, 0.f, 0.f};  ///< for holding the previous step data
    ChVector<float> m_acc_bias_prev = {0.f, 0.f, 0.f};   ///< for hold the previous step data

    float m_updateRate;    ///< holding the sensor update rate for use in drift
    float m_g_mean;        ///< mean of normal distribution for gyroscope
    float m_g_stdev;       ///< standard deviation of normal distribution for gyroscope
    float m_g_bias_drift;  ///< bias component of gyroscope drift
    float m_g_tau_drift;   ///< time constant for gyroscope drift
    float m_a_mean;        ///< mean of normal distribution for accelerometer
    float m_a_stdev;       ///< standard deviation of normal distribution for accelerometer
    float m_a_bias_drift;  ///< bias component of accelerometer drift
    float m_a_tau_drift;   ///< time constant for accelerometer drift
};

/// Class for generating IMU data
class CH_SENSOR_API ChFilterIMUUpdate : public ChFilter {
  public:
    /// Class constructor
    /// @param noise_model The noise model to use when augmenting the IMU data
    ChFilterIMUUpdate(std::shared_ptr<ChIMUNoiseModel> noise_model);

    /// Apply function. Generates IMU data.
    /// @param pSensor A pointer to the sensor on which the filter is attached.
    /// @param bufferInOut A buffer that is passed into the filter.
    virtual void Apply(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut);

    /// Initializes all data needed by the filter access apply function.
    /// @param pSensor A pointer to the sensor.
    virtual void Initialize(std::shared_ptr<ChSensor> pSensor) {}

  private:
    std::shared_ptr<SensorHostIMUBuffer> m_buffer;   ///< For holding generated IMU data
    std::shared_ptr<ChIMUNoiseModel> m_noise_model;  ///< The noise model for augmenting data
};

/// @}

}  // namespace sensor
}  // namespace chrono

#endif
