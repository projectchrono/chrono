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
class ChNoiseModel;
class ChGPSSensor;

/// @addtogroup sensor_filters
/// @{

/// Class for generating GPS data for a GPS sensor
class CH_SENSOR_API ChFilterGPSUpdate : public ChFilter {
  public:
    /// Class constructor
    /// @param gps_reference The GPS location of the simulation origin
    /// @param noise_model The noise model for augmenting the GPS data
    ChFilterGPSUpdate(ChVector<double> gps_reference, std::shared_ptr<ChNoiseModel> noise_model);

    /// Apply function. Generates GPS data.
    virtual void Apply();

    /// Initializes all data needed by the filter access apply function.
    /// @param pSensor A pointer to the sensor on which the filter is attached.
    /// @param bufferInOut A buffer that is passed into the filter.
    virtual void Initialize(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut);

  private:
    std::shared_ptr<SensorHostGPSBuffer> m_bufferOut;  ///< buffer that will be used for passing to the next filter
    std::shared_ptr<ChGPSSensor> m_GPSSensor;
    std::shared_ptr<ChNoiseModel> m_noise_model;  ///< pointer to the noise model for augmenting GPS data
    ChVector<double> m_ref;                       ///< for holding the reference location
};

/// @}

}  // namespace sensor
}  // namespace chrono

#endif
