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
// =============================================================================

#ifndef CHFILTERGPSUPDATE_H
#define CHFILTERGPSUPDATE_H

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

/// GPS Noise Model base class. All GPS noise models should inherit from here.
class CH_SENSOR_API ChGPSNoiseModel {
  public:
    /// Class constructor
    ChGPSNoiseModel(){};
    /// Class destructor
    ~ChGPSNoiseModel(){};

    /// Virtual function that every GPS noise model must implement. Will be called when noise should be added to the
    /// data.
    /// @param The Cartesian coordinates to be augmented with data.
    virtual void AddNoise(ChVector<double>& coords) = 0;
};

/// GPS Noise model: individually parameterized independent gaussian distribution
class CH_SENSOR_API ChGPSNoiseNone : public ChGPSNoiseModel {
  public:
    /// Class constructor
    ChGPSNoiseNone() {}
    /// Class destructor
    ~ChGPSNoiseNone() {}

    /// Noise addition function. Adds no noise for this model.
    /// @param The Cartesian coordinates to be augmented with data.
    virtual void AddNoise(ChVector<double>& coords) {}
};

/// GPS Noise model: individually parameterized independent gaussian distribution
class CH_SENSOR_API ChGPSNoiseNormal : public ChGPSNoiseModel {
  public:
    /// Class constructor
    /// @param mean The mean of the normal distribution
    /// @param stdev The standard deviation of the normal distribution
    ChGPSNoiseNormal(ChVector<float> mean, ChVector<float> stdev);
    /// Class destructor
    ~ChGPSNoiseNormal();

    /// Noise addition function. Adds no noise for this model.
    /// @param The Cartesian coordinates to be augmented with data.
    virtual void AddNoise(ChVector<double>& coords);

  private:
    std::minstd_rand m_generator;  ///< random number generator
    ChVector<float> m_mean;        ///< mean of the normal distribution
    ChVector<float> m_stdev;       ///< standard deviation of the normal distribution
};

/// Class for generating GPS data for a GPS sensor
class CH_SENSOR_API ChFilterGPSUpdate : public ChFilter {
  public:
    /// Class constructor
    /// @param gps_reference The GPS location of the simulation origin
    /// @param noise_model The noise model for augmenting the GPS data
    ChFilterGPSUpdate(ChVector<double> gps_reference, std::shared_ptr<ChGPSNoiseModel> noise_model);

    /// Apply function. Generates GPS data.
    /// @param pSensor A pointer to the sensor on which the filter is attached.
    /// @param bufferInOut A buffer that is passed into the filter.
    virtual void Apply(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut);

    /// Initializes all data needed by the filter access apply function.
    /// @param pSensor A pointer to the sensor.
    virtual void Initialize(std::shared_ptr<ChSensor> pSensor) {}

  private:
    std::shared_ptr<SensorHostGPSBuffer> m_buffer;   ///< buffer that will be used for passing to the next filter
    std::shared_ptr<ChGPSNoiseModel> m_noise_model;  ///< pointer to the noise model for augmenting GPS data
    ChVector<double> m_ref;                          ///< for holding the reference location
};

/// @}

}  // namespace sensor
}  // namespace chrono

#endif
