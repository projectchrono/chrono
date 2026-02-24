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
// Base class for all sensor
//
// =============================================================================

#include <iostream>

#include "chrono/physics/ChSystem.h"
#include "chrono/utils/ChUtils.h"

// NOTE: order is important! ChSensor.h must be included *before* ChFilterAccess.h
#include "chrono_sensor/sensors/ChSensor.h"
#include "chrono_sensor/filters/ChFilterAccess.h"

namespace chrono {
namespace sensor {

ChSensor::ChSensor(std::shared_ptr<ChBody> parent, float updateRate, ChFrame<double> offsetPose)
    : m_updateRate(updateRate),
      m_offsetPose(offsetPose),
      m_parent(parent),
      m_lag(0),
      m_collection_window(0),
      m_timeLastUpdated(0),
      m_name("") {
    m_num_launches = (int)(parent->GetSystem()->GetChTime() * updateRate);
}

ChSensor::~ChSensor() {}

void ChSensor::SetLag(float t) {
    m_lag = std::max(0.f, t);
}

void ChSensor::SetCollectionWindow(float t) {
    m_collection_window = ChClamp(t, 0.f, 1 / m_updateRate);
}

void ChSensor::PushFilter(std::shared_ptr<ChFilter> filter) {
    if (!m_filter_list_locked) {
        m_filters.push_back(filter);
    } else {
        std::cerr << "WARNING: Filter list has been locked for safety. All filters should be added to "
                     "sensor before sensor is added to ChSensorManager\n";
    }
}

void ChSensor::PushFilterFront(std::shared_ptr<ChFilter> filter) {
    if (!m_filter_list_locked) {
        m_filters.push_front(filter);
    } else {
        std::cerr << "WARNING: Filter list has been locked for safety. All filters should be added to "
                     "sensor before sensor is added to ChSensorManager\n";
    }
}

// -----------------------------------------------------------------------------

#ifdef CHRONO_HAS_OPTIX

// retriever function for image data in greyscale 8-bit format
template <>
CH_SENSOR_API UserR8BufferPtr ChSensor::GetMostRecentBuffer() {
    return GetMostRecentBufferHelper<UserR8BufferPtr, ChFilterR8Access, ChFilterR8AccessName>();
}

// retriever function for image data in RGBA 8-bit format
template <>
CH_SENSOR_API UserRGBA8BufferPtr ChSensor::GetMostRecentBuffer() {
    return GetMostRecentBufferHelper<UserRGBA8BufferPtr, ChFilterRGBA8Access, ChFilterRGBA8AccessName>();
}

// -----------------------------------------------------------------------------
// retriever function for image data in RGBA 16-bit format
// -----------------------------------------------------------------------------
template <>
CH_SENSOR_API UserRGBA16BufferPtr ChSensor::GetMostRecentBuffer() {
    // call the templated helper function
    return GetMostRecentBufferHelper<UserRGBA16BufferPtr, ChFilterRGBA16Access, ChFilterRGBA16AccessName>();
}

// -----------------------------------------------------------------------------
// retriever function for image data in RGBD half4 format
// -----------------------------------------------------------------------------
template <>
CH_SENSOR_API UserRGBDHalf4BufferPtr ChSensor::GetMostRecentBuffer() {
    // call the templated helper function
    return GetMostRecentBufferHelper<UserRGBDHalf4BufferPtr, ChFilterRGBDHalf4Access, ChFilterRGBDHalf4AccessName>();
}

// -----------------------------------------------------------------------------
// retriever function for image semantic data in semantic format
// -----------------------------------------------------------------------------
template <>
CH_SENSOR_API UserSemanticBufferPtr ChSensor::GetMostRecentBuffer() {
    return GetMostRecentBufferHelper<UserSemanticBufferPtr, ChFilterSemanticAccess, ChFilterSemanticAccessName>();
}

// retriever function for image depth data as float values
template <>
CH_SENSOR_API UserDepthBufferPtr ChSensor::GetMostRecentBuffer() {
    return GetMostRecentBufferHelper<UserDepthBufferPtr, ChFilterDepthAccess, ChFilterDepthAccessName>();
}

// -----------------------------------------------------------------------------
// retriever function for image normal data as float3 values
// -----------------------------------------------------------------------------
template <>
CH_SENSOR_API UserNormalBufferPtr ChSensor::GetMostRecentBuffer() {
    // call the templated helper function
    return GetMostRecentBufferHelper<UserNormalBufferPtr, ChFilterNormalAccess, ChFilterNormalAccessName>();
}

// retriever function for lidar data in range/depth,intensity format
template <>
CH_SENSOR_API UserDIBufferPtr ChSensor::GetMostRecentBuffer() {
    return GetMostRecentBufferHelper<UserDIBufferPtr, ChFilterDIAccess, ChFilterDIAccessName>();
}

// retriever function for point cloud data in XYZ-Intensity format
template <>
CH_SENSOR_API UserXYZIBufferPtr ChSensor::GetMostRecentBuffer() {
    return GetMostRecentBufferHelper<UserXYZIBufferPtr, ChFilterXYZIAccess, ChFilterXYZIAccessName>();
}

// retriever function for radar data
template <>
CH_SENSOR_API UserRadarBufferPtr ChSensor::GetMostRecentBuffer() {
    return GetMostRecentBufferHelper<UserRadarBufferPtr, ChFilterRadarAccess, ChFilterRadarAccessName>();
}

template <>
CH_SENSOR_API UserRadarXYZBufferPtr ChSensor::GetMostRecentBuffer() {
    return GetMostRecentBufferHelper<UserRadarXYZBufferPtr, ChFilterRadarXYZAccess, ChFilterRadarXYZAccessName>();
}

#endif

// -----------------------------------------------------------------------------

// retriever function for accelerometer data
template <>
CH_SENSOR_API UserAccelBufferPtr ChSensor::GetMostRecentBuffer() {
    return GetMostRecentBufferHelper<UserAccelBufferPtr, ChFilterAccelAccess, ChFilterAccelAccessName>();
}

// retriever function for gro data
template <>
CH_SENSOR_API UserGyroBufferPtr ChSensor::GetMostRecentBuffer() {
    return GetMostRecentBufferHelper<UserGyroBufferPtr, ChFilterGyroAccess, ChFilterGyroAccessName>();
}

// retriever function for magnetometer data
template <>
CH_SENSOR_API UserMagnetBufferPtr ChSensor::GetMostRecentBuffer() {
    return GetMostRecentBufferHelper<UserMagnetBufferPtr, ChFilterMagnetAccess, ChFilterMagnetAccessName>();
}

// retriever function for GPS data
template <>
CH_SENSOR_API UserGPSBufferPtr ChSensor::GetMostRecentBuffer() {
    return GetMostRecentBufferHelper<UserGPSBufferPtr, ChFilterGPSAccess, ChFilterGPSAccessName>();
}

// retriever function for tachometer data
template <>
CH_SENSOR_API UserTachometerBufferPtr ChSensor::GetMostRecentBuffer() {
    return GetMostRecentBufferHelper<UserTachometerBufferPtr, ChFilterTachometerAccess, ChFilterTachometerAccessName>();
}

// -----------------------------------------------------------------------------
// Helper function for retrieving the last buffer of given type
// -----------------------------------------------------------------------------

template <class UserBufferType, class FilterType, const char* FilterName>
UserBufferType ChSensor::GetMostRecentBufferHelper() {
    // find the last filter in the filter list that is a 'FilterType'
    // (note using reverse iterator)
    auto it = std::find_if(m_filters.rbegin(), m_filters.rend(), [](std::shared_ptr<ChFilter> p) {
        return std::dynamic_pointer_cast<FilterType>(p) != nullptr;
    });
    // std::cout<<"CastName"<<typeid(FilterType).name()<<std::endl;
    if (it == m_filters.rend()) {
        std::stringstream s;
        s << "Cannot return device buffer: Filter List does not contain an entry of type " << FilterName;

        throw std::runtime_error(s.str());
    }
    // std::cout<<"AfterCast \n";
    return (std::dynamic_pointer_cast<FilterType>(*it))->GetBuffer();
}

}  // namespace sensor
}  // namespace chrono
