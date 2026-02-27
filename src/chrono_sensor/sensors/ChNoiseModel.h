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

#ifndef CH_NOISE_MODEL_H
#define CH_NOISE_MODEL_H

#include <random>

#include "chrono_sensor/ChApiSensor.h"
#include "chrono/core/ChVector3.h"

namespace chrono {
namespace sensor {

/// @addtogroup sensor_sensors
/// @{

/// Noise model base class.
class CH_SENSOR_API ChNoiseModel {
  public:
    virtual ~ChNoiseModel() {}

    /// Function for adding noise to data.
    /// @param data data to augment
    virtual void AddNoise(ChVector3d& data) = 0;

    /// Function for adding noise over a time interval.
    /// @param data data to augment
    /// @param last_ch_time the last time the data was updated
    /// @param ch_time the current time
    virtual void AddNoise(ChVector3d& data, float last_ch_time, float ch_time) = 0;

  protected:
    ChNoiseModel() {}
};

/// Class implementing no noise model.
class CH_SENSOR_API ChNoiseNone : public ChNoiseModel {
  public:
    ChNoiseNone() {}
    ~ChNoiseNone() {}

    /// Function for adding noise to a set of values.
    virtual void AddNoise(ChVector3d& data) override {}

    /// Function for adding noise over a time interval.
    virtual void AddNoise(ChVector3d& data, float last_ch_time, float ch_time) override {}
};

/// Noise model based on a normal distribution.
class CH_SENSOR_API ChNoiseNormal : public ChNoiseModel {
  public:
    /// Class constructor.
    /// @param mean The mean of the normal distribution
    /// @param stdev The standard deviation of the normal distribution
    ChNoiseNormal(ChVector3d mean, ChVector3d stdev);

    ~ChNoiseNormal() {}

    /// Noise addition function.
    virtual void AddNoise(ChVector3d& data) override;

    /// Function for adding noise over a time interval.
    virtual void AddNoise(ChVector3d& data, float last_ch_time, float ch_time) override;

  private:
    std::minstd_rand m_generator;  ///< random number generator
    ChVector3d m_mean;             ///< mean of the normal distribution
    ChVector3d m_stdev;            ///< standard deviation of the normal distribution
};

/// IMU Noise model: Gaussian drifting noise with noncorrelated equal distributions.
class CH_SENSOR_API ChNoiseNormalDrift : public ChNoiseModel {
  public:
    ChNoiseNormalDrift(double updateRate, ChVector3d mean, ChVector3d stdev, double drift_bias, double tau_drift);

    ~ChNoiseNormalDrift() {}

    /// Function for adding noise to data.
    virtual void AddNoise(ChVector3d& data) override;

    /// Function for adding noise over a time interval.
    virtual void AddNoise(ChVector3d& data, float last_ch_time, float ch_time) override;

  private:
    std::minstd_rand m_generator;  ///< random number generator
    ChVector3d m_bias_prev;

    double m_updateRate;  ///< holding the sensor update rate for use in drift
    ChVector3d m_mean;    ///< mean of normal distribution for gyroscope
    ChVector3d m_stdev;   ///< standard deviation of normal distribution for gyroscope
    double m_drift_bias;  ///< bias component of gyroscope drift
    double m_tau_drift;   ///< time constant for gyroscope drift
};

/// GPS Noise model based on Random Walks
class CH_SENSOR_API ChNoiseRandomWalks : public ChNoiseModel {
  public:
    /// Class constructor with default tuning parameters.
    /// @param mean The mean of the normal distribution of the acceleration that is integrated twice to provide the
    /// random walk
    /// @param sigma The standard deviation of the normal distribution of the acceleration that is integrated twice to
    /// provide the random walk
    /// @param noise_model_update_rate The update rate of the noise model which defines the integration step size. Note:
    /// This is different from the sensor update rate.
    /// @param gps_reference The reference position of the GPS
    ChNoiseRandomWalks(float mean, float sigma, float noise_model_update_rate, ChVector3d gps_reference);

    /// Class constructor with custom tuning parameters.
    /// @param mean The mean of the normal distribution of the acceleration that is integrated twice to provide the
    /// random walk
    /// @param sigma The standard deviation of the normal distribution of the acceleration that is integrated twice to
    /// provide the random walk
    /// @param noise_model_update_rate The update rate of the noise model which defines the integration step size. Note:
    /// This is different from the sensor update rate.
    /// @param max_velocity The maximum allowable velocity for the random walk
    /// @param max_acceleration The maximum allowable acceleration for the random walk
    /// @param gps_reference The reference position of the GPS
    ChNoiseRandomWalks(float mean,
                       float sigma,
                       float noise_model_update_rate,
                       double max_velocity,
                       double max_acceleration,
                       ChVector3d gps_reference);

    ~ChNoiseRandomWalks() {}

    /// Function for adding noise to data.
    virtual void AddNoise(ChVector3d& data) override {}

    /// Function for adding noise over a time interval.
    virtual void AddNoise(ChVector3d& data, float last_ch_time, float ch_time) override;

  private:
    std::minstd_rand m_generator;  ///< random number generator
    float m_mean;                  ///< mean of the normal distribution
    float m_sigma;                 ///< standard deviation of the normal distribution
    double m_step_size;            ///< current step size for the random walk
    double m_max_velocity;         ///< maximum first derivative value for the random walk
    double m_max_acceleration;     ///< maximum second derivative value for the random walk
    ChVector3d m_gps_reference;    ///< reference position for the GPS to augment
    float m_last_updated_ch_time;  ///< last time the noise model was updated
    ChVector3d m_prev_error_p;     ///< previous error in position
    ChVector3d m_prev_error_v;     ///< previous first derivative value for the noise model
};

/// @} sensor_sensors

}  // namespace sensor
}  // namespace chrono

#endif