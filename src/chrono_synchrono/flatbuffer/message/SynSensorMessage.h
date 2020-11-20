// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2020 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Aaron Young
// =============================================================================
//
// Wraps data received from a flatbuffer sensor message into a corresponding C++
// class.
// See also flatbuffer/fbs/Sensor.fbs
//
// =============================================================================

#ifndef SYN_SENSOR_MESSAGE_H
#define SYN_SENSOR_MESSAGE_H

#include "chrono_synchrono/flatbuffer/message/SynMessage.h"

namespace chrono {
namespace synchrono {

/// @addtogroup synchrono_flatbuffer
/// @{

// a pixel in RGBA 8bpp format
struct PixelRGBA8 {
    uint8_t R;
    uint8_t G;
    uint8_t B;
    uint8_t A;
};

// a pixel in R8 (8-bit grayscale) format
struct PixelR8 {
    uint8_t R;
};

// depth and intensity data in generic format
struct PixelDI {
    float range;
    float intensity;
};

// point cloud and intensity data in generic format
struct PixelXYZI {
    float x;
    float y;
    float z;
    float intensity;
};

// IMU data in generic format
struct IMUData {
    double Accel[3];
    double Roll;
    double Pitch;
    double Yaw;
};

// GPS data in generic format
struct GPSData {
    double Latitude;
    double Longitude;
    double Altitude;
    double Time;
};

struct SynSensorMessageState : public SynMessageState {
    // Base class of 2D buffers
    struct SensorBuffer {
        SensorBuffer() : Width(0), Height(0), LaunchedCount(0) {}
        SensorBuffer(unsigned int w, unsigned int h) : Width(w), Height(h), LaunchedCount(0) {}
        SensorBuffer(unsigned int w, unsigned int h, void* b, int size_in_bytes)
            : Width(w), Height(h), LaunchedCount(0) {
            Buffer.resize(size_in_bytes);
            memcpy(Buffer.data(), b, size_in_bytes);
        }
        virtual ~SensorBuffer() {
            // delete Buffer;
        }  // virtual destructor so class is virtual so it can participate in dynamic_pointer_cast<>'s
        unsigned int Width;
        unsigned int Height;
        unsigned int LaunchedCount;  // number of times updates have been launched.
        std::vector<uint8_t> Buffer;
    };

    enum Type {
        NONE = 0,   ///< Default Type
        RGBA8 = 1,  ///< RGBA 8BPP Format
        R8 = 2,     ///< 8-bit Grayscale
        DI = 3,     ///< Depth and Intensity
        XYZI = 4,   ///< Point Cloud and Intensity
        IMU = 5,    ///< Accel (x,y,z), Roll, Pitch, Yaw
        GPS = 6     ///< Latitude, Longitude, Altitude, Time
    } type;

    std::shared_ptr<SensorBuffer> buffer;  ///< Handle to the SensorBuffer

    /// Default Constructor
    SynSensorMessageState(double time = 0, Type type = NONE, std::shared_ptr<SensorBuffer> buffer = nullptr)
        : SynMessageState(time), type(type), buffer(buffer) {}

    /// Casts underlying Buffer to the specified type
    template <class T>
    T* GetBufferAsType() {
        return (T*)buffer->Buffer.data();
    }
};

/// Wraps data received from a flatbuffer sensor message into a corresponding C++ class.
class SYN_API SynSensorMessage : public SynMessage {
  public:
    ///@brief Construct a new SensorMessageState object
    ///
    ///@param rank the rank of this message
    SynSensorMessage(int rank);

    ///@brief Generates and sets the state of this message from flatbuffer message
    ///
    ///@param message the flatbuffer message to convert to a MessageState object
    virtual void StateFromMessage(const SynFlatBuffers::Message* message) override;

    ///@brief Generates a SynFlatBuffers::Message from the message state
    ///
    ///@param builder the flatbuffer builder used to construct messages
    ///@return flatbuffers::Offset<SynFlatBuffers::Message> the generated message
    virtual FlatBufferMessage MessageFromState(flatbuffers::FlatBufferBuilder& builder) override;

    ///@brief Get the SensorMessageState object
    ///
    ///@return std::shared_ptr<SensorMessageState> the state associated with this message
    virtual std::shared_ptr<SynMessageState> GetState() override { return m_state; }

  private:
    std::shared_ptr<SynSensorMessageState> m_state;  ///< handle to the message state
};

/// @} synchrono_flatbuffer

}  // namespace synchrono
}  // namespace chrono

#endif
