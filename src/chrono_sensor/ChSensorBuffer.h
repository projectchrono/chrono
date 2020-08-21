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

#ifndef CHSENSORBUFFER_H
#define CHSENSORBUFFER_H


#ifdef _WIN32
 #define NOMINMAX
#endif

#include <optix.h>
#include <optixu/optixpp.h>  //needed to make sure things are in the right namespace. Must be done before optixpp_namespace.h
#include <optixu/optixpp_namespace.h>  //is covered by optixpp.h but will be removed from optixpp.h in the future
#include <functional>
#include <memory>

namespace chrono {
namespace sensor {

/// @addtogroup sensor_buffers
/// @{

/// The base buffer class that contains sensor data (contains meta data of the buffer and pointer to raw data)
struct SensorBuffer {
    /// Default constructor that intializes all zero values
    SensorBuffer() : Width(0), Height(0), LaunchedCount(0), TimeStamp(0) {}
    /// Constructor based on height, width, and time
    SensorBuffer(unsigned int w, unsigned int h, float t) : Width(w), Height(h), LaunchedCount(0), TimeStamp(t) {}

    /// virtual destructor so class is virtual so it can participate in dynamic_pointer_cast<>'s
    virtual ~SensorBuffer() {}
    float TimeStamp;      ///< The time stamp on the buffer (simulation time when data collection stopped)
    unsigned int Width;   ///< The width of the data (image width when data is an image)
    unsigned int Height;  ///< The height of the data (image height when data is an image)
    unsigned int
        LaunchedCount;  ///<  number of times updates have been launched. This may not reflect how many have been
                        // completed.
};

/// Base class of 2D buffers. This holds the raw sensor data.
/// (Do not use this class directly, instead use the typedefs below)
template <class B>
struct SensorBufferT : public SensorBuffer {
    SensorBufferT() {}
    B Buffer;
};

//============================================================================
// Buffer of Optix memory (contents described by members inside optix::Buffer)
//============================================================================
/// Wrapper of an optix buffer as a sensor buffer for homogeneous use in sensor filters.
using SensorOptixBuffer = SensorBufferT<optix::Buffer>;

//================================
// RGBA8 Camera Format and Buffers
//================================

/// A pixel as defined by RGBA 8bpp format
struct PixelRGBA8 {
    uint8_t R;  ///< Red value
    uint8_t G;  ///< Green value
    uint8_t B;  ///< Blue value
    uint8_t A;  ///< Transparency value
};
/// RGBA host buffer to be used for managing data on the host
using SensorHostRGBA8Buffer = SensorBufferT<std::shared_ptr<PixelRGBA8[]>>;
/// RGBA device buffer to be used by camera filters in the graph
using DeviceRGBA8BufferPtr = std::shared_ptr<PixelRGBA8[]>;
/// Sensor buffer wrapper of a DeviceRGBA8BufferPtr
using SensorDeviceRGBA8Buffer = SensorBufferT<DeviceRGBA8BufferPtr>;
/// pointer to an RGBA image on the host that has been moved for safety and can be given to the user
using UserRGBA8BufferPtr = std::shared_ptr<SensorHostRGBA8Buffer>;

//===============================================
// R8 (8-bit Grayscale) Camera Format and Buffers
//===============================================

/// Greyscale host buffer to be used by camera filters in the graph
using SensorHostR8Buffer = SensorBufferT<std::shared_ptr<char[]>>;
/// Greyscale device buffer to be used by camera filters in the graph
using DeviceR8BufferPtr = std::shared_ptr<char[]>;
/// Sensor buffer wrapper of a DeviceR8BufferPtr
using SensorDeviceR8Buffer = SensorBufferT<DeviceR8BufferPtr>;
/// pointer to a greyscale image on the host that has been moved for safety and can be given to the user
using UserR8BufferPtr = std::shared_ptr<SensorHostR8Buffer>;

//=====================================
// Depth Lidar Data Formats and Buffers
//=====================================

/// Depth and intensity data in generic format
struct PixelDI {
    float range;      ///< Distance measurement of the lidar beam
    float intensity;  ///< Relative intensity of returned laser pulse
};
/// Depth-intensity host buffer to be used by lidar filters in the graph
using SensorHostDIBuffer = SensorBufferT<std::shared_ptr<PixelDI[]>>;
/// Depth-intensity device buffer to be used by lidar filters in the graph
using DeviceDIBufferPtr = std::shared_ptr<PixelDI[]>;
/// Sensor buffer wrapper of a DeviceDIBufferPtr
using SensorDeviceDIBuffer = SensorBufferT<DeviceDIBufferPtr>;
/// pointer to a depth-intensity buffer on the host that has been moved for safety and can be given to the user
using UserDIBufferPtr = std::shared_ptr<SensorHostDIBuffer>;

//===========================================
// Point Cloud Lidar Data Formats and Buffers
//===========================================

/// Point cloud and intensity data in generic format
struct PixelXYZI {
    float x;          ///< x location of the point in space
    float y;          ///< y location of the point in space
    float z;          ///< z location of the point in space
    float intensity;  ///< intensity of the reflection at the corresponding point
};
/// Point cloud host buffer to be used by lidar filters in the graph
using SensorHostXYZIBuffer = SensorBufferT<std::shared_ptr<PixelXYZI[]>>;
/// Point cloud device buffer to be used by lidar filters in the graph
using DeviceXYZIBufferPtr = std::shared_ptr<PixelXYZI[]>;
/// Sensor buffer wrapper of a DeviceXYZIBufferPtr
using SensorDeviceXYZIBuffer = SensorBufferT<DeviceXYZIBufferPtr>;
/// pointer to a point cloud buffer on the host that has been moved for safety and can be given to the user
using UserXYZIBufferPtr = std::shared_ptr<SensorHostXYZIBuffer>;

//=============================
// IMU Data Format and Buffers
//=============================

/// IMU data in generic format
struct IMUData {
    double Accel[3];  ///< Translational acceleration (x,y,z)
    double Roll;      ///< Roll rate (angular velocity (rad/s))
    double Pitch;     ///< Pitch rate (angular velocity (rad/s))
    double Yaw;       ///< Yaw rate (angular velocity (rad/s))
};
/// IMU host buffer to be used by IMU filters in the graph
using SensorHostIMUBuffer = SensorBufferT<std::shared_ptr<IMUData[]>>;
/// pointer to an IMU buffer on the host that has been moved for safety and can be given to the user
using UserIMUBufferPtr = std::shared_ptr<SensorHostIMUBuffer>;

//============================
// GPS Data Format and Buffers
//============================

/// GPS data in generic format
struct GPSData {
    double Latitude;   ///< Latitudinal coordinate of the sensor
    double Longitude;  ///< Longitudinal coordinate of the sensor
    double Altitude;   ///< Altitude of the sensor
    double Time;       ///< Time from the sensor
};
/// GPS host buffer to be used by GPS filters in the graph
using SensorHostGPSBuffer = SensorBufferT<std::shared_ptr<GPSData[]>>;
/// pointer to a GPS buffer on the host that has been moved for safety and can be given to the user
using UserGPSBufferPtr = std::shared_ptr<SensorHostGPSBuffer>;

/// @} sensor_buffers

}  // namespace sensor
}  // namespace chrono

#endif
