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

#ifndef CHFILTERACCESS_H
#define CHFILTERACCESS_H

#include <functional>
#include <memory>
#include <queue>
#include <stack>
#include <mutex>
#include "chrono_sensor/sensors/ChSensorBuffer.h"
#include "chrono_sensor/filters/ChFilter.h"
#include "chrono_sensor/sensors/ChOptixSensor.h"
#include "chrono/physics/ChSystem.h"

#include <typeinfo>

namespace chrono {
namespace sensor {

// forward declaration
class ChSensor;

/// @addtogroup sensor_filters
/// @{

/// Filter for accessing data from the sensor.
template <class BufferType, class UserBufferType>
class CH_SENSOR_API ChFilterAccess : public ChFilter {
  public:
    /// Class constructor
    /// @param name String name of the filter. Defaults to empty.
    ChFilterAccess(std::string name = {}) : ChFilter(name.length() > 0 ? name : "CopyToFilter"){};

    /// Virtual class destructor
    virtual ~ChFilterAccess() {}

    /// Apply function. Moves data from the device into the lag buffer and presents data to the user if the data is
    /// ready based on the time.
    /// @param pSensor A pointer to the sensor on which the filter is attached.
    /// @param bufferInOut A buffer that is passed into the filter. This data is what will be made available for the
    /// user.
    virtual void Apply();

    /// Initializes all data needed by the filter access apply function.
    /// @param pSensor A pointer to the sensor.
    /// @param bufferInOut the incoming process buffer
    virtual void Initialize(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut) {
        // check the buffer is the correct type
        if (!bufferInOut) {
            InvalidFilterGraphNullBuffer(pSensor);
        }
        if (auto pBuf = std::dynamic_pointer_cast<BufferType>(bufferInOut)) {
            m_bufferIn = pBuf;  // save handle to the incoming buffer
        } else {
            std::cout<<typeid(pBuf).name()<<std::endl;
            std::cout<<typeid(bufferInOut).name()<<std::endl;
            InvalidFilterGraphBufferTypeMismatch(pSensor);
        }

        if (auto pOpx = std::dynamic_pointer_cast<ChOptixSensor>(pSensor)) {
            m_cuda_stream = pOpx->GetCudaStream();
        }

        m_sensor = pSensor;  // save handle to the parent sensor (weak ptr to not cause loop dependency)
        m_max_lag_buffers = 1 + (unsigned int)std::ceil((pSensor->GetLag() + pSensor->GetCollectionWindow()) *
                                                        pSensor->GetUpdateRate());
        m_user_buffer = chrono_types::make_shared<BufferType>();
    }

    /// User calls this to get access and ownership of the buffer memory on the host.
    /// user can store the returned pointer or modify as desired,
    /// user has the ownership of the memory. Underlying framework has released all ownership.
    /// Filter graph will create new memory during the next run.
    /// Returns a user buffer that is safe for the user to access.
    UserBufferType GetBuffer() {
        // lock the mutex before shifting ownership of the data
        std::lock_guard<std::mutex> lck(m_mutexBufferAccess);

        // give the user the most recent buffer that qualifies based on time
        // current time:
        auto pSensor = m_sensor.lock();
        float ch_time = (float)pSensor->GetParent()->GetSystem()->GetChTime();

        // create a buffer even if it will have null data. Copy to get the right metadata
        // UserBufferType return_buffer = chrono_types::make_shared<BufferType>(m_buffer);

        // pop all the old buffers
        while (m_lag_buffers.size() > 0 && ch_time > m_lag_buffers.front()->TimeStamp + pSensor->GetLag() - 1e-7) {
            auto buf = m_lag_buffers.front();

            m_lag_buffers.pop();

            // move the data into our return buffer for the user
            m_user_buffer->Buffer = std::move(buf->Buffer);
            m_user_buffer->Width = buf->Width;
            m_user_buffer->Height = buf->Height;
            m_user_buffer->TimeStamp = buf->TimeStamp;
            m_user_buffer->LaunchedCount = buf->LaunchedCount;
        }

        // return the copied class that now has ownership of the buffer memory
        // new memory will be created when it is needed. This transfers ownership to the user as we do not have a
        // reference to the copied class
        return m_user_buffer;
    }

  private:
    std::mutex m_mutexBufferAccess;          ///< mutex that is locked when the lag buffer is touched
    UserBufferType m_user_buffer;            ///< buffer that can be returned
    std::weak_ptr<ChSensor> m_sensor;        ///< pointer to the sensor to which this filter is attached
    std::shared_ptr<BufferType> m_bufferIn;  ///< shared pointer to the buffer coming in
    CUstream m_cuda_stream;                  ///< reference to the cuda stream for device-side buffers

    std::queue<std::shared_ptr<BufferType>>
        m_lag_buffers;  ///< buffers that are time stamped and held until past their lag time
    std::stack<std::shared_ptr<BufferType>>
        m_empty_lag_buffers;         ///< buffers that can be reused rather than allocating new memory each time
    unsigned int m_max_lag_buffers;  ///< maximum number of buffers that could be needed
};

// Typedefs for explicit Filters
/// Access to greyscale data
using ChFilterR8Access = ChFilterAccess<SensorHostR8Buffer, UserR8BufferPtr>;
/// Access to RGBA8 data
using ChFilterRGBA8Access = ChFilterAccess<SensorHostRGBA8Buffer, UserRGBA8BufferPtr>;
/// Access to semantic image
using ChFilterSemanticAccess = ChFilterAccess<SensorHostSemanticBuffer, UserSemanticBufferPtr>;
/// Access to point cloud data
using ChFilterXYZIAccess = ChFilterAccess<SensorHostXYZIBuffer, UserXYZIBufferPtr>;
/// Access to depth/intensity data
using ChFilterDIAccess = ChFilterAccess<SensorHostDIBuffer, UserDIBufferPtr>;
/// Access to accelerometer data
using ChFilterAccelAccess = ChFilterAccess<SensorHostAccelBuffer, UserAccelBufferPtr>;
/// Access to gyroscope data
using ChFilterGyroAccess = ChFilterAccess<SensorHostGyroBuffer, UserGyroBufferPtr>;
/// Access to magnetometer data
using ChFilterMagnetAccess = ChFilterAccess<SensorHostMagnetBuffer, UserMagnetBufferPtr>;
/// Access to GPS data
using ChFilterGPSAccess = ChFilterAccess<SensorHostGPSBuffer, UserGPSBufferPtr>;
/// Access to Radar data
using ChFilterRadarAccess = ChFilterAccess<SensorHostRadarBuffer, UserRadarBufferPtr>;
/// Access to Processed Radar data
using ChFilterRadarXYZAccess = ChFilterAccess<SensorHostRadarXYZBuffer, UserRadarXYZBufferPtr>;
/// Access to Tachoemter data
using ChFilterTachometerAccess = ChFilterAccess<SensorHostTachometerBuffer, UserTachometerBufferPtr>;
/// Access to Encoder data
// using ChFilterEncoderAccess = ChFilterAccess<SensorHostEncoderBuffer, UserEncoderBufferPtr>;

/// @}

}  // namespace sensor
}  // namespace chrono

#endif
