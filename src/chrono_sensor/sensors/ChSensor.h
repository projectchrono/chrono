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
// Base class for all sensors
//
// =============================================================================

#ifndef CHSENSOR_H
#define CHSENSOR_H

#include "chrono_sensor/ChApiSensor.h"

#include <list>
#include <mutex>

#include "chrono_sensor/sensors/ChSensorBuffer.h"
#include "chrono/physics/ChBody.h"
#include "chrono_sensor/filters/ChFilter.h"
#include "chrono_sensor/optix/ChOptixUtils.h"

namespace chrono {
namespace sensor {

/// @addtogroup sensor_sensors
/// @{

/// global constanst for use in template parameters
const char ChFilterR8AccessName[] = "ChFilterR8Access";          /// single channel 8 bit array
const char ChFilterRGBA8AccessName[] = "ChFilterRGBA8Access";    /// 4 channel 8 bit array
const char ChFilterDIAccessName[] = "ChFilterDIAccess";          /// 2 channel float array (Depth+Intenisty)
const char ChFilterXYZIAccessName[] = "ChFilterXYZIAccess";      /// 4 channel float array (XYZ positions+Intensity)
const char ChFilterAccelAccessName[] = "ChFilterAccelAccess";    /// Accelerometer data format (3 doubles total)
const char ChFilterGyroAccessName[] = "ChFilterGyroAccess";      /// Gyroscope data format (3 doubles total)
const char ChFilterMagnetAccessName[] = "ChFilterMagnetAccess";  /// Magnetometer data format (3 doubles total)
const char ChFilterGPSAccessName[] = "ChFilterGPSAccess";        /// GPS data format (4 doubles total)
const char ChFilterRadarAccessName[] = "ChFilterRadarAccess";
const char ChFilterRadarXYZAccessName[] = "ChFilterRadarXYZAccess";
const char ChFilterTachometerAccessName[] = "ChFilterTachometerAccess";

const char ChFilterDepthAccessName[] = "ChFilterDepthAccess";

/// Base class for a chrono sensor. A specific sensor can inherit from here
class CH_SENSOR_API ChSensor {
  public:
    ///@brief Constructor for the base sensor class
    ///@param parent Body to which the sensor is attached.
    ///@param updateRate Rate at which the sensor should update.
    ///@param offsetPose Relative position and orientation of the sensor with respect to its parent object.
    //@param lag Lag time between end of data collection and when data becomes available to the user.
    //@param collection_window Collection time over which the sensor should collect data from the simulation.
    ChSensor(std::shared_ptr<chrono::ChBody> parent, float updateRate, chrono::ChFrame<double> offsetPose);

    /// Class destructor
    virtual ~ChSensor();

    /// @brief Set the sensor's relative position and orientation
    /// @param pose The relative position and orientation with respect to the parent body
    void SetOffsetPose(chrono::ChFrame<double> pose) { m_offsetPose = pose; }

    /// Get the sensor's relative position and orientation
    /// @return The frame that specifies the offset pose
    ChFrame<double> GetOffsetPose() { return m_offsetPose; }

    /// Get the object to which the sensor is attached
    /// @return A shared pointer to the body on which the sensor is attached
    std::shared_ptr<ChBody> GetParent() const { return m_parent; }

    /// @brief Set the sensor's name
    /// @param name Name of the sensor -> not used for any internal critical mechanisms
    void SetName(std::string name) { m_name = name; }

    /// Get the name of the sensor
    /// @return The string name of the sensor
    std::string GetName() const { return m_name; }

    /// Get the sensor update rate (Hz)
    /// @returns The update rate in Hz
    float GetUpdateRate() const { return m_updateRate; }

    /// Set the lag parameter
    /// @param t The lag time
    void SetLag(float t);

    /// Get the sensor lag (seconds)
    /// @return The lag of the sensor
    float GetLag() const { return m_lag; }

    /// Set the collection window
    /// @param t The collection time of the sensor
    void SetCollectionWindow(float t);

    /// Get the sensor data collection window (seconds)
    /// @return The duration of simulation time over which the sensor generates its data
    float GetCollectionWindow() const { return m_collection_window; }

    ///@brief Set the sensor update rate (Hz)
    ///@param updateRate Desired update rate in Hz
    void SetUpdateRate(float updateRate) { m_updateRate = updateRate; }

    /// Get the number of times the sensor has been updated
    /// @return The number of times an update for the sensor has been started
    unsigned int GetNumLaunches() {
        std::lock_guard<std::mutex> lck(m_dataAccess);
        return m_num_launches;
    }

    /// Increments the count of number of updates
    void IncrementNumLaunches() {
        std::lock_guard<std::mutex> lck(m_dataAccess);
        m_num_launches++;
    }

    /// Get the sensor's list of filters
    /// @return An immutable list of filters that are used to generate and augment the sensor's data
    std::list<std::shared_ptr<ChFilter>> GetFilterList() const { return m_filters; }

    /// Add a filter to the sensor
    /// @param filter A filter that should be added to the filter list if the filter list is not yet locked. If the
    /// filter list has been locked (i.e. the sensor has started generating data) the new filter will be ignored.
    void PushFilter(std::shared_ptr<ChFilter> filter);

    /// Add a filter to the front of the list on a sensor
    /// @param filter A filter that should be added to the filter list if the filter list is not yet locked. If the
    /// filter list has been locked (i.e. the sensor has started generating data) the new filter will be ignored.
    void PushFilterFront(std::shared_ptr<ChFilter> filter);

    /// Gives ability to lock the filter list to prevent race conditions. This is called automatically when the sensor
    /// is added to the ChSensor manager. This is needed to prevent changing of filters while a worker thread is
    /// processing the filters. WARNING: this operation cannot be undone.
    void LockFilterList() { m_filter_list_locked = true; }

    /// Get the last filter in the list that matches the template type
    /// @return A shared pointer to a ChSensorBuffer of the templated type.
    template <class UserBufferType>
    UserBufferType GetMostRecentBuffer();  // explicit specializations exist for each buffer type avaiable

  protected:
    float m_updateRate;  ///< sensor update rate
    float m_lag;         ///< sensor lag from the time all scene information is available (sensor processing time)
    float m_collection_window;  ///< time over which data is collected. (lag+shutter = time from when data collection
                                ///< is started to when it is available to the user)
    float m_timeLastUpdated;    ///< time since previous update
    std::shared_ptr<chrono::ChBody> m_parent;        ///< object to which the sensor is attached
    chrono::ChFrame<double> m_offsetPose;            ///< position and orientation of the sensor relative to its parent
    std::string m_name;                              ///< name of the sensor
    unsigned int m_num_launches;                     ///< number of times the sensor has been updated
    std::list<std::shared_ptr<ChFilter>> m_filters;  ///< filter list for post-processing sensor data
    bool m_filter_list_locked = false;  ///< gives ability to lock the filter list to prevent race conditions

  private:
    template <class UserBufferType, class FilterType, const char* FilterName>
    UserBufferType GetMostRecentBufferHelper();  ///< explicit specializations exist for each buffer type avaiable
    std::mutex m_dataAccess;                     ///< data access mutex to prevent data race in the sensor class

};  // class ChSensor

class CH_SENSOR_API ChDynamicSensor : public ChSensor {
  public:
    ChDynamicSensor(std::shared_ptr<chrono::ChBody> parent, float updateRate, chrono::ChFrame<double> offsetPose)
        : ChSensor(parent, updateRate, offsetPose) {}
    ~ChDynamicSensor() {}
    virtual void PushKeyFrame() = 0;
    virtual void ClearKeyFrames() = 0;
};

/// @} sensor_sensors

}  // namespace sensor
}  // namespace chrono

#endif
