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
// Authors: Eric Brandt, Asher Elmquist
// =============================================================================
//
// =============================================================================

#ifndef CHFILTER_H
#define CHFILTER_H

#include <memory>
#include <string>

#include "chrono/core/ChTypes.h"

#include "chrono_sensor/ChApiSensor.h"
#include "chrono_sensor/ChConfigSensor.h"
#include "chrono_sensor/sensors/ChSensorBuffer.h"

namespace chrono {
namespace sensor {

// forward declaration
class ChSensor;

/// @addtogroup sensor_filters
/// @{

/// Sentinel for a filter that has not been attached to a sensor, and for a sensor that has not been
/// registered with a ChSensorManager. Seeding from a sentinel would silently correlate streams, so
/// ChSensorManager::GetDeterministicSeed throws on it instead.
static const unsigned int CH_SENSOR_UNASSIGNED_RNG_ID = 0xFFFFFFFFu;

/// Base class for all filters that can be applied to a sensor after initial rendering. Any filters that will be added
/// to a sensor must inherit from here.
class CH_SENSOR_API ChFilter {
  public:
    /// Virtual class destructor
    virtual ~ChFilter() {}

    /// Virtual apply function. This will be called sequentially for all filters in a filter list. This should contain
    /// the necessary processing during the simulation. This must be threadsafe when applied to a ChOptixSensor.
    virtual void Apply() = 0;

    /// Virtual initialize function. This will be called once when added to the ChSensorManager. It should create all
    /// the necessary memory space and setup any data that will be needed in the apply function. This function does not
    /// need to be threadsafe so can be used to access and store any sensor specific information that is needed later.
    /// @param pSensor A pointer to the sensor to which it is attached
    /// @param bufferInOut The pointer that is passed from one filter to the next. Can be changed by the filter when
    /// augmentation does not happen in place.
    virtual void Initialize(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut) = 0;

    /// Accesses the name of the filter. Name not used for any critical processes. Optional use for clarity.
    /// A string reference to the filter's name.
    std::string& Name() { return m_name; }

  protected:
    /// protected constructor for the filter which requires a name as input.
    /// @param name A string name of the filter.
    ChFilter(std::string name) { Name() = name; }

    /// This filter's RNG stream index within its sensor, for filters that own a cuRAND buffer.
    ///
    /// Needed because (sensor, purpose) is NOT a unique stream key: PushFilter applies no type or
    /// duplicate check, so two filters of the same kind can sit on one sensor, each with its own
    /// RNG buffer, and both would answer to the same (sensor, purpose) pair. This index separates
    /// them. It is an attach-order counter, not a list position, because ChOptixEngine prepends its
    /// render filter with PushFilterFront after the user's filters are already in place, which would
    /// renumber a positional index.
    /// @return The index assigned when this filter was attached, or CH_SENSOR_UNASSIGNED_RNG_ID if
    /// it never was.
    unsigned int GetRngStreamIndex() const { return m_rng_stream_index; }
    /// Error function for invalid filter graph: null buffer found
    void InvalidFilterGraphNullBuffer(std::shared_ptr<ChSensor> pSensor);
    /// Error function for invalid filter graph: type mismatch in graph
    void InvalidFilterGraphBufferTypeMismatch(std::shared_ptr<ChSensor> pSensor);
    /// Error function for invalid filter graph: type mismatch in graph
    void InvalidFilterGraphSensorTypeMismatch(std::shared_ptr<ChSensor> pSensor);

  private:
    std::string m_name;  ///< stores the name of the filter.

    /// Assigned by ChSensor when the filter is attached. Deliberately private with no public
    /// setter: a user who could change it could silently collide two RNG streams.
    unsigned int m_rng_stream_index = CH_SENSOR_UNASSIGNED_RNG_ID;

    friend class ChSensor;  ///< assigns m_rng_stream_index in PushFilter / PushFilterFront
};

/// @}

}  // namespace sensor
}  // namespace chrono

#endif
