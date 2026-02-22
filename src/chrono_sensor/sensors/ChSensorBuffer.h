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

#ifndef CHSENSORBUFFER_H
#define CHSENSORBUFFER_H

#ifdef _WIN32
    #ifndef NOMINMAX
        #define NOMINMAX
    #endif
#endif

#include <functional>
#include <memory>
#include <vector>

#ifdef CHRONO_HAS_OPTIX
    #include <cuda_fp16.h>
#endif

namespace chrono {
namespace sensor {

/// @addtogroup sensor_buffers
/// @{

/// Base buffer class that contains sensor data (contains meta data of the buffer and pointer to raw data).
struct SensorBuffer {
    /// Default constructor that intializes all zero values
    SensorBuffer() : TimeStamp(0), Width(0), Height(0), LaunchedCount(0) {}
    /// Constructor based on height, width, and time
    SensorBuffer(unsigned int w, unsigned int h, float t) : TimeStamp(t), Width(w), Height(h), LaunchedCount(0) {}

    virtual ~SensorBuffer() {}

    float TimeStamp;             ///< The time stamp on the buffer (simulation time when data collection stopped)
    unsigned int Width;          ///< The width of the data (image width when data is an image)
    unsigned int Height;         ///< The height of the data (image height when data is an image)
    unsigned int LaunchedCount;  ///<  number of times updates have been launched (may not reflect how many completed)
    ////unsigned int Beam_return_count;  ///< number of beam returns for lidar model
    ////bool Dual_return;                ///< true if dual return mode, false otherwise
};

/// Base class of 2D buffers.
/// This holds the raw sensor data.
template <class B>
struct SensorBufferT : public SensorBuffer {
    SensorBufferT() {}
    B Buffer;
};

template <class B>
struct LidarBufferT : public SensorBufferT<B> {
    LidarBufferT() : Dual_return(false), Beam_return_count(0) {}
    unsigned int Beam_return_count;
    bool Dual_return;
};

template <class B>
struct RadarBufferT : public SensorBufferT<B> {
    RadarBufferT() : Beam_return_count(0), Num_clusters(0) {}
    int Beam_return_count;
    int invalid_returns;
    int Num_clusters;
    std::vector<std::array<float, 3>> avg_velocity;
    std::vector<std::array<float, 3>> centroids;
    std::vector<float> amplitudes;
};

//================================
// RGBA8 Camera Format and Buffers
//================================

/// A pixel as defined by RGBA float4 format.
struct PixelFloat4 {
    float R;  ///< Red value
    float G;  ///< Green value
    float B;  ///< Blue value
    float A;  ///< Transparency value
};
/// RGBA host buffer to be used for managing data on the host.
using SensorHostFloat4Buffer = SensorBufferT<std::shared_ptr<PixelFloat4[]>>;

/// RGBA device buffer to be used by camera filters in the graph.
using DeviceFloat4BufferPtr = std::shared_ptr<PixelFloat4[]>;

/// Sensor buffer wrapper of a DeviceFloat4BufferPtr.
using SensorDeviceFloat4Buffer = SensorBufferT<DeviceFloat4BufferPtr>;

/// Pointer to an RGBA image on the host that has been moved for safety and can be given to the user.
using UserFloat4BufferPtr = std::shared_ptr<SensorHostFloat4Buffer>;

#ifdef CHRONO_HAS_OPTIX

/// A pixel as defined by RGBA float4 format.
struct PixelHalf4 {
    __half R;  ///< Red value
    __half G;  ///< Green value
    __half B;  ///< Blue value
    __half A;  ///< Transparency value
};

/// RGBA host buffer to be used for managing data on the host
using SensorHostHalf4Buffer = SensorBufferT<std::shared_ptr<PixelHalf4[]>>;

/// RGBA device buffer to be used by camera filters in the graph
using DeviceHalf4BufferPtr = std::shared_ptr<PixelHalf4[]>;

/// Sensor buffer wrapper of a DeviceHalf4BufferPtr
using SensorDeviceHalf4Buffer = SensorBufferT<DeviceHalf4BufferPtr>;

/// pointer to an RGBA image on the host that has been moved for safety and can be given to the user
using UserHalf4BufferPtr = std::shared_ptr<SensorHostHalf4Buffer>;

#endif

//================================
// RGBA8 Camera Format and Buffers
//================================

/// A pixel as defined by RGBA 8bpp format.
struct PixelRGBA8 {
    uint8_t R;  ///< Red value
    uint8_t G;  ///< Green value
    uint8_t B;  ///< Blue value
    uint8_t A;  ///< Transparency value
};

/// RGBA host buffer to be used for managing data on the host.
using SensorHostRGBA8Buffer = SensorBufferT<std::shared_ptr<PixelRGBA8[]>>;

/// RGBA device buffer to be used by camera filters in the graph.
using DeviceRGBA8BufferPtr = std::shared_ptr<PixelRGBA8[]>;

/// Sensor buffer wrapper of a DeviceRGBA8BufferPtr.
using SensorDeviceRGBA8Buffer = SensorBufferT<DeviceRGBA8BufferPtr>;

/// Pointer to an RGBA image on the host that has been moved for safety and can be given to the user.
using UserRGBA8BufferPtr = std::shared_ptr<SensorHostRGBA8Buffer>;

//===============================================
// R8 (8-bit Grayscale) Camera Format and Buffers
//===============================================

/// Greyscale host buffer to be used by camera filters in the graph.
using SensorHostR8Buffer = SensorBufferT<std::shared_ptr<char[]>>;

/// Greyscale device buffer to be used by camera filters in the graph.
using DeviceR8BufferPtr = std::shared_ptr<char[]>;

/// Sensor buffer wrapper of a DeviceR8BufferPtr.
using SensorDeviceR8Buffer = SensorBufferT<DeviceR8BufferPtr>;

/// Pointer to a greyscale image on the host that has been moved for safety and can be given to the user.
using UserR8BufferPtr = std::shared_ptr<SensorHostR8Buffer>;

/// A pixel as defined for semantic segmentation.
struct PixelSemantic {
    unsigned short int class_id;     ///< class id
    unsigned short int instance_id;  ///< instance id
};

/// Semantic host buffer to be used for managing data on the host.
using SensorHostSemanticBuffer = SensorBufferT<std::shared_ptr<PixelSemantic[]>>;

/// Semantic device buffer to be used by segmenation camera.
using DeviceSemanticBufferPtr = std::shared_ptr<PixelSemantic[]>;

/// Sensor buffer wrapper of a DeviceSemanticBufferPtr.
using SensorDeviceSemanticBuffer = SensorBufferT<DeviceSemanticBufferPtr>;

/// Pointer to an semantic image on the host that has been moved for safety and can be given to the user.
using UserSemanticBufferPtr = std::shared_ptr<SensorHostSemanticBuffer>;

struct PixelDepth {
    float depth;
};

using SensorHostDepthBuffer = SensorBufferT<std::shared_ptr<PixelDepth[]>>;

using DeviceDepthBufferPtr = std::shared_ptr<PixelDepth[]>;

using SensorDeviceDepthBuffer = SensorBufferT<DeviceDepthBufferPtr>;

using UserDepthBufferPtr = std::shared_ptr<SensorHostDepthBuffer>;

//=====================================
// Range Radar Data Formats and Buffers
//=====================================

struct RadarReturn {
    float range;
    float azimuth;
    float elevation;
    float doppler_velocity[3];
    float amplitude;
    float objectId;
};

/// Host buffer to be used by radar filters in the graph.
using SensorHostRadarBuffer = RadarBufferT<std::shared_ptr<RadarReturn[]>>;

/// Device buffer to be used by radar filters in the graph.
using DeviceRadarBufferPtr = std::shared_ptr<RadarReturn[]>;

/// Sensor buffer wrapper of a DeviceRadarBufferPtr.
using SensorDeviceRadarBuffer = RadarBufferT<DeviceRadarBufferPtr>;

/// Pointer to a radar buffer on the host that has been moved for safety and can be given to the user.
using UserRadarBufferPtr = std::shared_ptr<SensorHostRadarBuffer>;

struct RadarXYZReturn {
    float x;
    float y;
    float z;
    float vel_x;
    float vel_y;
    float vel_z;
    float amplitude;
    float objectId;
};

using SensorHostRadarXYZBuffer = RadarBufferT<std::shared_ptr<RadarXYZReturn[]>>;
using DeviceRadarXYZBufferPtr = std::shared_ptr<RadarXYZReturn[]>;
using SensorDeviceRadarXYZBuffer = RadarBufferT<DeviceRadarXYZBufferPtr>;
using UserRadarXYZBufferPtr = std::shared_ptr<SensorHostRadarXYZBuffer>;

//=====================================
// Depth Lidar Data Formats and Buffers
//=====================================

/// Depth and intensity data in generic format.
struct PixelDI {
    float range;      ///< Distance measurement of the lidar beam
    float intensity;  ///< Relative intensity of returned laser pulse
};

/// Depth-intensity host buffer to be used by lidar filters in the graph.
using SensorHostDIBuffer = LidarBufferT<std::shared_ptr<PixelDI[]>>;

/// Depth-intensity device buffer to be used by lidar filters in the graph.
using DeviceDIBufferPtr = std::shared_ptr<PixelDI[]>;

/// Sensor buffer wrapper of a DeviceDIBufferPtr.
using SensorDeviceDIBuffer = LidarBufferT<DeviceDIBufferPtr>;

/// Pointer to a depth-intensity buffer on the host that has been moved for safety and can be given to the user.
using UserDIBufferPtr = std::shared_ptr<SensorHostDIBuffer>;

//===========================================
// Point Cloud Lidar Data Formats and Buffers
//===========================================

/// Point cloud and intensity data in generic format.
struct PixelXYZI {
    float x;          ///< x location of the point in space
    float y;          ///< y location of the point in space
    float z;          ///< z location of the point in space
    float intensity;  ///< intensity of the reflection at the corresponding point
};

/// Point cloud host buffer to be used by lidar filters in the graph.
using SensorHostXYZIBuffer = LidarBufferT<std::shared_ptr<PixelXYZI[]>>;

/// Point cloud device buffer to be used by lidar filters in the graph.
using DeviceXYZIBufferPtr = std::shared_ptr<PixelXYZI[]>;

/// Sensor buffer wrapper of a DeviceXYZIBufferPtr.
using SensorDeviceXYZIBuffer = LidarBufferT<DeviceXYZIBufferPtr>;

/// Pointer to a point cloud buffer on the host that has been moved for safety and can be given to the user.
using UserXYZIBufferPtr = std::shared_ptr<SensorHostXYZIBuffer>;

//=============================
// IMU Data Format and Buffers
//=============================

/// Accelerometer data.
struct AccelData {
    double X;  ///< translational acceleration in local x-direction
    double Y;  ///< translational acceleration in local y-direction
    double Z;  ///< translational acceleration in local z-direction
};

/// Acclerometer host buffer to be used by acclerometer filters in the graph.
using SensorHostAccelBuffer = SensorBufferT<std::shared_ptr<AccelData[]>>;

/// Pointer to an acclerometer buffer on the host that has been moved for safety and can be given to the user.
using UserAccelBufferPtr = std::shared_ptr<SensorHostAccelBuffer>;

/// Gyroscope data.
struct GyroData {
    double Roll;   ///< angular velocity in local x-direction
    double Pitch;  ///< angular velocity in local y-direction
    double Yaw;    ///< angular velocity in local z-direction
};

/// Acclerometer host buffer to be used by acclerometer filters in the graph.
using SensorHostGyroBuffer = SensorBufferT<std::shared_ptr<GyroData[]>>;

/// Pointer to an acclerometer buffer on the host that has been moved for safety and can be given to the user.
using UserGyroBufferPtr = std::shared_ptr<SensorHostGyroBuffer>;

/// Magnetometer data.
struct MagnetData {
    double X;  ///< x component of magnetic field
    double Y;  ///< y component of magnetic field
    double Z;  ///< z component of magnetic field
};

/// acclerometer host buffer to be used by acclerometer filters in the graph.
using SensorHostMagnetBuffer = SensorBufferT<std::shared_ptr<MagnetData[]>>;

/// Pointer to an acclerometer buffer on the host that has been moved for safety and can be given to the user.
using UserMagnetBufferPtr = std::shared_ptr<SensorHostMagnetBuffer>;

//===================================
// Tachometer Data Format and Buffers
//===================================

struct TachometerData {
    float rpm;  ///< RPM of motor shaft
};

/// Tachometer host buffer to be used by tachometer filters in the graph.
using SensorHostTachometerBuffer = SensorBufferT<std::shared_ptr<TachometerData[]>>;

/// Pointer to a tachometer buffer on the host that has been moved for safety and can be given to the user.
using UserTachometerBufferPtr = std::shared_ptr<SensorHostTachometerBuffer>;

/*
//================================
// Speedometer Data Format and Buffers
//================================

struct EncoderData {
    float speed;  ///< speed of object
};

// Speedometer host buffer to be used by speedometer filters in the graph.
using SensorHostEncoderBuffer = SensorBufferT<std::shared_ptr<EncoderData[]>>;

// Pointer to a speedometer buffer on the host that has been moved to safety and can be given to the user.
using UserEncoderBufferPtr = std::shared_ptr<SensorHostEncoderBuffer>;
*/

//============================
// GPS Data Format and Buffers
//============================

/// GPS data in generic format.
struct GPSData {
    double Latitude;   ///< Latitudinal coordinate of the sensor
    double Longitude;  ///< Longitudinal coordinate of the sensor
    double Altitude;   ///< Altitude of the sensor
    double Time;       ///< Time from the sensor
};

/// GPS host buffer to be used by GPS filters in the graph.
using SensorHostGPSBuffer = SensorBufferT<std::shared_ptr<GPSData[]>>;

/// Pointer to a GPS buffer on the host that has been moved for safety and can be given to the user.
using UserGPSBufferPtr = std::shared_ptr<SensorHostGPSBuffer>;

/// @} sensor_buffers

}  // namespace sensor
}  // namespace chrono

#endif
