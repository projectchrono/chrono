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

#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/sensors/ChSensor.h"
#include "chrono_sensor/utils/CudaMallocHelper.h"

#include <cuda.h>

namespace chrono {
namespace sensor {

template <>
CH_SENSOR_API void ChFilterAccess<SensorHostR8Buffer, UserR8BufferPtr>::Apply() {
    // create a new buffer to push to the lag buffer list
    std::shared_ptr<SensorHostR8Buffer> tmp_buffer;
    if (m_empty_lag_buffers.size() > 0) {
        tmp_buffer = m_empty_lag_buffers.top();
        m_empty_lag_buffers.pop();
    } else {
        tmp_buffer = chrono_types::make_shared<SensorHostR8Buffer>();
        std::shared_ptr<char[]> b(cudaHostMallocHelper<char>(m_bufferIn->Width * m_bufferIn->Height),
                                  cudaHostFreeHelper<char>);
        tmp_buffer->Buffer = std::move(b);
    }

    tmp_buffer->Width = m_bufferIn->Width;
    tmp_buffer->Height = m_bufferIn->Height;
    tmp_buffer->LaunchedCount = m_bufferIn->LaunchedCount;
    tmp_buffer->TimeStamp = m_bufferIn->TimeStamp;

    cudaMemcpyAsync(tmp_buffer->Buffer.get(), m_bufferIn->Buffer.get(), m_bufferIn->Width * m_bufferIn->Height,
                    cudaMemcpyDeviceToHost, m_cuda_stream);

    {  // lock in this scope before pushing to lag buffer queue
        std::lock_guard<std::mutex> lck(m_mutexBufferAccess);
        // push our buffer into the lag queue
        m_lag_buffers.push(tmp_buffer);
        // prevent lag buffer overflow - remove any old buffers that have expired. We don't want the lag_buffer to
        // grow unbounded
        while (m_lag_buffers.size() > m_max_lag_buffers) {
            m_empty_lag_buffers.push(
                m_lag_buffers.front());  // push the buffer back for efficiency if it wasn't given to the user
            m_lag_buffers.pop();
        }
        // synchronize the cuda stream since we moved data to the host
        cudaStreamSynchronize(m_cuda_stream);
    }
}

template <>
CH_SENSOR_API void ChFilterAccess<SensorHostRGBA8Buffer, UserRGBA8BufferPtr>::Apply() {
    // create a new buffer to push to the lag buffer list
    std::shared_ptr<SensorHostRGBA8Buffer> tmp_buffer;
    if (m_empty_lag_buffers.size() > 0) {
        tmp_buffer = m_empty_lag_buffers.top();
        m_empty_lag_buffers.pop();
    } else {
        tmp_buffer = chrono_types::make_shared<SensorHostRGBA8Buffer>();
        std::shared_ptr<PixelRGBA8[]> b(cudaHostMallocHelper<PixelRGBA8>(m_bufferIn->Width * m_bufferIn->Height),
                                        cudaHostFreeHelper<PixelRGBA8>);
        tmp_buffer->Buffer = std::move(b);
    }

    tmp_buffer->Width = m_bufferIn->Width;
    tmp_buffer->Height = m_bufferIn->Height;
    tmp_buffer->LaunchedCount = m_bufferIn->LaunchedCount;
    tmp_buffer->TimeStamp = m_bufferIn->TimeStamp;

    cudaMemcpyAsync(tmp_buffer->Buffer.get(), m_bufferIn->Buffer.get(),
                    m_bufferIn->Width * m_bufferIn->Height * sizeof(PixelRGBA8), cudaMemcpyDeviceToHost, m_cuda_stream);

    {  // lock in this scope before pushing to lag buffer queue
        std::lock_guard<std::mutex> lck(m_mutexBufferAccess);
        // push our buffer into the lag queue
        m_lag_buffers.push(tmp_buffer);
        // prevent lag buffer overflow - remove any old buffers that have expired. We don't want the lag_buffer to
        // grow unbounded
        while (m_lag_buffers.size() > m_max_lag_buffers) {
            m_empty_lag_buffers.push(
                m_lag_buffers.front());  // push the buffer back for efficiency if it wasn't given to the user
            m_lag_buffers.pop();
        }
        // synchronize the cuda stream since we moved data to the host
        cudaStreamSynchronize(m_cuda_stream);
    }
}

template <>
CH_SENSOR_API void ChFilterAccess<SensorHostSemanticBuffer, UserSemanticBufferPtr>::Apply() {
    // create a new buffer to push to the lag buffer list
    std::shared_ptr<SensorHostSemanticBuffer> tmp_buffer;
    if (m_empty_lag_buffers.size() > 0) {
        tmp_buffer = m_empty_lag_buffers.top();
        m_empty_lag_buffers.pop();
    } else {
        tmp_buffer = chrono_types::make_shared<SensorHostSemanticBuffer>();
        std::shared_ptr<PixelSemantic[]> b(cudaHostMallocHelper<PixelSemantic>(m_bufferIn->Width * m_bufferIn->Height),
                                        cudaHostFreeHelper<PixelSemantic>);
        tmp_buffer->Buffer = std::move(b);
    }

    tmp_buffer->Width = m_bufferIn->Width;
    tmp_buffer->Height = m_bufferIn->Height;
    tmp_buffer->LaunchedCount = m_bufferIn->LaunchedCount;
    tmp_buffer->TimeStamp = m_bufferIn->TimeStamp;

    cudaMemcpyAsync(tmp_buffer->Buffer.get(), m_bufferIn->Buffer.get(),
                    m_bufferIn->Width * m_bufferIn->Height * sizeof(PixelSemantic), cudaMemcpyDeviceToHost, m_cuda_stream);

    {  // lock in this scope before pushing to lag buffer queue
        std::lock_guard<std::mutex> lck(m_mutexBufferAccess);
        // push our buffer into the lag queue
        m_lag_buffers.push(tmp_buffer);
        // prevent lag buffer overflow - remove any old buffers that have expired. We don't want the lag_buffer to
        // grow unbounded
        while (m_lag_buffers.size() > m_max_lag_buffers) {
            m_empty_lag_buffers.push(
                m_lag_buffers.front());  // push the buffer back for efficiency if it wasn't given to the user
            m_lag_buffers.pop();
        }
        // synchronize the cuda stream since we moved data to the host
        cudaStreamSynchronize(m_cuda_stream);
    }
}


template <>
CH_SENSOR_API void ChFilterAccess<SensorHostXYZIBuffer, UserXYZIBufferPtr>::Apply() {
    // create a new buffer to push to the lag buffer list
    std::shared_ptr<SensorHostXYZIBuffer> tmp_buffer;
    if (m_empty_lag_buffers.size() > 0) {
        tmp_buffer = m_empty_lag_buffers.top();
        m_empty_lag_buffers.pop();
    } else {
        tmp_buffer = chrono_types::make_shared<SensorHostXYZIBuffer>();
        std::shared_ptr<PixelXYZI[]> b(cudaHostMallocHelper<PixelXYZI>(m_bufferIn->Width * m_bufferIn->Height),
                                       cudaHostFreeHelper<PixelXYZI>);
        tmp_buffer->Buffer = std::move(b);
    }

    tmp_buffer->Width = m_bufferIn->Beam_return_count;
    tmp_buffer->Height = 1;
    tmp_buffer->LaunchedCount = m_bufferIn->LaunchedCount;
    tmp_buffer->TimeStamp = m_bufferIn->TimeStamp;

    cudaMemcpyAsync(tmp_buffer->Buffer.get(), m_bufferIn->Buffer.get(),
                    m_bufferIn->Width * m_bufferIn->Height * sizeof(PixelXYZI), cudaMemcpyDeviceToHost, m_cuda_stream);

    {  // lock in this scope before pushing to lag buffer queue
        std::lock_guard<std::mutex> lck(m_mutexBufferAccess);
        // push our buffer into the lag queue
        m_lag_buffers.push(tmp_buffer);
        // prevent lag buffer overflow - remove any old buffers that have expired. We don't want the lag_buffer to
        // grow unbounded
        while (m_lag_buffers.size() > m_max_lag_buffers) {
            m_empty_lag_buffers.push(
                m_lag_buffers.front());  // push the buffer back for efficiency if it wasn't given to the user
            m_lag_buffers.pop();
        }
        // synchronize the cuda stream since we moved data to the host
        cudaStreamSynchronize(m_cuda_stream);
    }
}

template <>
CH_SENSOR_API void ChFilterAccess<SensorHostDIBuffer, UserDIBufferPtr>::Apply() {
    // create a new buffer to push to the lag buffer list
    std::shared_ptr<SensorHostDIBuffer> tmp_buffer;
    if (m_empty_lag_buffers.size() > 0) {
        tmp_buffer = m_empty_lag_buffers.top();
        m_empty_lag_buffers.pop();
    } else {
        tmp_buffer = chrono_types::make_shared<SensorHostDIBuffer>();
        std::shared_ptr<PixelDI[]> b(cudaHostMallocHelper<PixelDI>(m_bufferIn->Width * m_bufferIn->Height),
                                     cudaHostFreeHelper<PixelDI>);
        tmp_buffer->Buffer = std::move(b);
    }

    tmp_buffer->Width = m_bufferIn->Width;
    tmp_buffer->Height = m_bufferIn->Height;
    tmp_buffer->LaunchedCount = m_bufferIn->LaunchedCount;
    tmp_buffer->TimeStamp = m_bufferIn->TimeStamp;

    cudaMemcpyAsync(tmp_buffer->Buffer.get(), m_bufferIn->Buffer.get(),
                    m_bufferIn->Width * m_bufferIn->Height * sizeof(PixelDI), cudaMemcpyDeviceToHost, m_cuda_stream);

    {  // lock in this scope before pushing to lag buffer queue
        std::lock_guard<std::mutex> lck(m_mutexBufferAccess);
        // push our buffer into the lag queue
        m_lag_buffers.push(tmp_buffer);
        // prevent lag buffer overflow - remove any old buffers that have expired. We don't want the lag_buffer to
        // grow unbounded
        while (m_lag_buffers.size() > m_max_lag_buffers) {
            m_empty_lag_buffers.push(
                m_lag_buffers.front());  // push the buffer back for efficiency if it wasn't given to the user
            m_lag_buffers.pop();
        }
        // synchronize the cuda stream since we moved data to the host
        cudaStreamSynchronize(m_cuda_stream);
    }
}

template <>
CH_SENSOR_API void ChFilterAccess<SensorHostRadarBuffer, UserRadarBufferPtr>::Apply() {
    // create a new buffer to push to the lag buffer list
    std::shared_ptr<SensorHostRadarBuffer> tmp_buffer;
    if (m_empty_lag_buffers.size() > 0) {
        tmp_buffer = m_empty_lag_buffers.top();
        m_empty_lag_buffers.pop();
    } else {
        tmp_buffer = chrono_types::make_shared<SensorHostRadarBuffer>();
        std::shared_ptr<RadarReturn[]> b(cudaHostMallocHelper<RadarReturn>(m_bufferIn->Width * m_bufferIn->Height),
                                        cudaHostFreeHelper<RadarReturn>);
        tmp_buffer->Buffer = std::move(b);
    }

    tmp_buffer->Width = m_bufferIn->Width;
    tmp_buffer->Height = m_bufferIn->Height;
    tmp_buffer->LaunchedCount = m_bufferIn->LaunchedCount;
    tmp_buffer->TimeStamp = m_bufferIn->TimeStamp;

    cudaMemcpyAsync(tmp_buffer->Buffer.get(), m_bufferIn->Buffer.get(),
                    m_bufferIn->Width * m_bufferIn->Height * sizeof(RadarReturn), cudaMemcpyDeviceToHost, m_cuda_stream);

    {  // lock in this scope before pushing to lag buffer queue
        std::lock_guard<std::mutex> lck(m_mutexBufferAccess);
        // push our buffer into the lag queue
        m_lag_buffers.push(tmp_buffer);
        // prevent lag buffer overflow - remove any old buffers that have expired. We don't want the lag_buffer to
        // grow unbounded
        while (m_lag_buffers.size() > m_max_lag_buffers) {
            m_empty_lag_buffers.push(
                m_lag_buffers.front());  // push the buffer back for efficiency if it wasn't given to the user
            m_lag_buffers.pop();
        }
        // synchronize the cuda stream since we moved data to the host
        cudaStreamSynchronize(m_cuda_stream);
    }
}

template <>
CH_SENSOR_API void ChFilterAccess<SensorHostRadarXYZBuffer, UserRadarXYZBufferPtr>::Apply() {
    // create a new buffer to push to the lag buffer list
    std::shared_ptr<SensorHostRadarXYZBuffer> tmp_buffer;
    if (m_empty_lag_buffers.size() > 0) {
        tmp_buffer = m_empty_lag_buffers.top();
        m_empty_lag_buffers.pop();
    } else {
        tmp_buffer = chrono_types::make_shared<SensorHostRadarXYZBuffer>();
        std::shared_ptr<RadarXYZReturn[]> b(
            cudaHostMallocHelper<RadarXYZReturn>(m_bufferIn->Width * m_bufferIn->Height),
            cudaHostFreeHelper<RadarXYZReturn>);
        tmp_buffer->Buffer = std::move(b);
    }

    tmp_buffer->Width = m_bufferIn->Beam_return_count;
    tmp_buffer->Height = 1;
    tmp_buffer->LaunchedCount = m_bufferIn->LaunchedCount;
    tmp_buffer->TimeStamp = m_bufferIn->TimeStamp;

    cudaMemcpyAsync(tmp_buffer->Buffer.get(), m_bufferIn->Buffer.get(),
                    m_bufferIn->Width * m_bufferIn->Height * sizeof(RadarXYZReturn), cudaMemcpyDeviceToHost,
                    m_cuda_stream);

    {  // lock in this scope before pushing to lag buffer queue
        std::lock_guard<std::mutex> lck(m_mutexBufferAccess);
        // push our buffer into the lag queue
        m_lag_buffers.push(tmp_buffer);
        // prevent lag buffer overflow - remove any old buffers that have expired. We don't want the lag_buffer to
        // grow unbounded
        while (m_lag_buffers.size() > m_max_lag_buffers) {
            m_empty_lag_buffers.push(
                m_lag_buffers.front());  // push the buffer back for efficiency if it wasn't given to the user
            m_lag_buffers.pop();
        }
        // synchronize the cuda stream since we moved data to the host
        cudaStreamSynchronize(m_cuda_stream);
    }
}

template <>
CH_SENSOR_API void ChFilterAccess<SensorHostAccelBuffer, UserAccelBufferPtr>::Apply() {
    // create a new buffer to push to the lag buffer list
    std::shared_ptr<SensorHostAccelBuffer> tmp_buffer;
    if (m_empty_lag_buffers.size() > 0) {
        tmp_buffer = m_empty_lag_buffers.top();
        m_empty_lag_buffers.pop();
    } else {
        tmp_buffer = chrono_types::make_shared<SensorHostAccelBuffer>();
        tmp_buffer->Buffer = std::make_unique<AccelData[]>(m_bufferIn->Width * m_bufferIn->Height);
    }

    tmp_buffer->Width = m_bufferIn->Width;
    tmp_buffer->Height = m_bufferIn->Height;
    tmp_buffer->LaunchedCount = m_bufferIn->LaunchedCount;
    tmp_buffer->TimeStamp = m_bufferIn->TimeStamp;

    // copy the data into our new buffer
    memcpy(tmp_buffer->Buffer.get(), m_bufferIn->Buffer.get(), sizeof(AccelData));

    {  // lock in this scope before pushing to lag buffer queue
        std::lock_guard<std::mutex> lck(m_mutexBufferAccess);
        // push our buffer into the lag queue
        m_lag_buffers.push(tmp_buffer);
        // prevent lag buffer overflow - remove any super old buffers that have expired. We don't want the lag_buffer to
        // grow unbounded
        while (m_lag_buffers.size() > m_max_lag_buffers) {
            m_empty_lag_buffers.push(
                m_lag_buffers.front());  // push the buffer back for efficiency if it wasn't given to the user
            m_lag_buffers.pop();
        }
    }
}

template <>
CH_SENSOR_API void ChFilterAccess<SensorHostGyroBuffer, UserGyroBufferPtr>::Apply() {
    // create a new buffer to push to the lag buffer list
    std::shared_ptr<SensorHostGyroBuffer> tmp_buffer;
    if (m_empty_lag_buffers.size() > 0) {
        tmp_buffer = m_empty_lag_buffers.top();
        m_empty_lag_buffers.pop();
    } else {
        tmp_buffer = chrono_types::make_shared<SensorHostGyroBuffer>();
        tmp_buffer->Buffer = std::make_unique<GyroData[]>(m_bufferIn->Width * m_bufferIn->Height);
    }

    tmp_buffer->Width = m_bufferIn->Width;
    tmp_buffer->Height = m_bufferIn->Height;
    tmp_buffer->LaunchedCount = m_bufferIn->LaunchedCount;
    tmp_buffer->TimeStamp = m_bufferIn->TimeStamp;

    // copy the data into our new buffer
    memcpy(tmp_buffer->Buffer.get(), m_bufferIn->Buffer.get(), sizeof(GyroData));

    {  // lock in this scope before pushing to lag buffer queue
        std::lock_guard<std::mutex> lck(m_mutexBufferAccess);
        // push our buffer into the lag queue
        m_lag_buffers.push(tmp_buffer);
        // prevent lag buffer overflow - remove any super old buffers that have expired. We don't want the lag_buffer to
        // grow unbounded
        while (m_lag_buffers.size() > m_max_lag_buffers) {
            m_empty_lag_buffers.push(
                m_lag_buffers.front());  // push the buffer back for efficiency if it wasn't given to the user
            m_lag_buffers.pop();
        }
    }
}

template <>
CH_SENSOR_API void ChFilterAccess<SensorHostMagnetBuffer, UserMagnetBufferPtr>::Apply() {
    // create a new buffer to push to the lag buffer list
    std::shared_ptr<SensorHostMagnetBuffer> tmp_buffer;
    if (m_empty_lag_buffers.size() > 0) {
        tmp_buffer = m_empty_lag_buffers.top();
        m_empty_lag_buffers.pop();
    } else {
        tmp_buffer = chrono_types::make_shared<SensorHostMagnetBuffer>();
        tmp_buffer->Buffer = std::make_unique<MagnetData[]>(m_bufferIn->Width * m_bufferIn->Height);
    }

    tmp_buffer->Width = m_bufferIn->Width;
    tmp_buffer->Height = m_bufferIn->Height;
    tmp_buffer->LaunchedCount = m_bufferIn->LaunchedCount;
    tmp_buffer->TimeStamp = m_bufferIn->TimeStamp;

    // copy the data into our new buffer
    memcpy(tmp_buffer->Buffer.get(), m_bufferIn->Buffer.get(), sizeof(MagnetData));

    {  // lock in this scope before pushing to lag buffer queue
        std::lock_guard<std::mutex> lck(m_mutexBufferAccess);
        // push our buffer into the lag queue
        m_lag_buffers.push(tmp_buffer);
        // prevent lag buffer overflow - remove any super old buffers that have expired. We don't want the lag_buffer to
        // grow unbounded
        while (m_lag_buffers.size() > m_max_lag_buffers) {
            m_empty_lag_buffers.push(
                m_lag_buffers.front());  // push the buffer back for efficiency if it wasn't given to the user
            m_lag_buffers.pop();
        }
    }
}

template <>
CH_SENSOR_API void ChFilterAccess<SensorHostGPSBuffer, UserGPSBufferPtr>::Apply() {
    // create a new buffer to push to the lag buffer list
    std::shared_ptr<SensorHostGPSBuffer> tmp_buffer;
    if (m_empty_lag_buffers.size() > 0) {
        tmp_buffer = m_empty_lag_buffers.top();
        m_empty_lag_buffers.pop();
    } else {
        tmp_buffer = chrono_types::make_shared<SensorHostGPSBuffer>();
        tmp_buffer->Buffer = std::make_unique<GPSData[]>(m_bufferIn->Width * m_bufferIn->Height);
    }

    tmp_buffer->Width = m_bufferIn->Width;
    tmp_buffer->Height = m_bufferIn->Height;
    tmp_buffer->LaunchedCount = m_bufferIn->LaunchedCount;
    tmp_buffer->TimeStamp = m_bufferIn->TimeStamp;

    // copy the data into our new buffer
    memcpy(tmp_buffer->Buffer.get(), m_bufferIn->Buffer.get(), sizeof(GPSData));

    {  // lock in this scope before pushing to lag buffer queue
        std::lock_guard<std::mutex> lck(m_mutexBufferAccess);
        // push our buffer into the lag queue
        m_lag_buffers.push(tmp_buffer);
        // prevent lag buffer overflow - remove any super old buffers that have expired. We don't want the lag_buffer to
        // grow unbounded
        while (m_lag_buffers.size() > m_max_lag_buffers) {
            m_empty_lag_buffers.push(
                m_lag_buffers.front());  // push the buffer back for efficiency if it wasn't given to the user
            m_lag_buffers.pop();
        }
    }
}

template <>
CH_SENSOR_API void ChFilterAccess<SensorHostTachometerBuffer, UserTachometerBufferPtr>::Apply() {
    // create a new buffer to push to the lag buffer list
    std::shared_ptr<SensorHostTachometerBuffer> tmp_buffer;
    if (m_empty_lag_buffers.size() > 0) {
        tmp_buffer = m_empty_lag_buffers.top();
        m_empty_lag_buffers.pop();
    } else {
        tmp_buffer = chrono_types::make_shared<SensorHostTachometerBuffer>();
        tmp_buffer->Buffer = std::make_unique<TachometerData[]>(m_bufferIn->Width * m_bufferIn->Height);
    }

    tmp_buffer->Width = m_bufferIn->Width;
    tmp_buffer->Height = m_bufferIn->Height;
    tmp_buffer->LaunchedCount = m_bufferIn->LaunchedCount;
    tmp_buffer->TimeStamp = m_bufferIn->TimeStamp;

    // copy the data into our new buffer
    memcpy(tmp_buffer->Buffer.get(), m_bufferIn->Buffer.get(), sizeof(TachometerData));

    {  // lock in this scope before pushing to lag buffer queue
        std::lock_guard<std::mutex> lck(m_mutexBufferAccess);
        // push our buffer into the lag queue
        m_lag_buffers.push(tmp_buffer);
        // prevent lag buffer overflow - remove any super old buffers that have expired. We don't want the lag_buffer to
        // grow unbounded
        while (m_lag_buffers.size() > m_max_lag_buffers) {
            m_empty_lag_buffers.push(
                m_lag_buffers.front());  // push the buffer back for efficiency if it wasn't given to the user
            m_lag_buffers.pop();
        }
    }
}


// template <>
// CH_SENSOR_API void ChFilterAccess<SensorHostEncoderBuffer, UserEncoderBufferPtr>::Apply() {
//     // create a new buffer to push to the lag buffer list
//     std::shared_ptr<SensorHostEncoderBuffer> tmp_buffer;
//     if (m_empty_lag_buffers.size() > 0) {
//         tmp_buffer = m_empty_lag_buffers.top();
//         m_empty_lag_buffers.pop();
//     } else {
//         tmp_buffer = chrono_types::make_shared<SensorHostEncoderBuffer>();
//         tmp_buffer->Buffer = std::make_unique<EncoderData[]>(m_bufferIn->Width * m_bufferIn->Height);
//     }
// 
//     tmp_buffer->Width = m_bufferIn->Width;
//     tmp_buffer->Height = m_bufferIn->Height;
//     tmp_buffer->LaunchedCount = m_bufferIn->LaunchedCount;
//     tmp_buffer->TimeStamp = m_bufferIn->TimeStamp;
// 
//     // copy the data into our new buffer
//     memcpy(tmp_buffer->Buffer.get(), m_bufferIn->Buffer.get(), sizeof(EncoderData));
// 
//     {  // lock in this scope before pushing to lag buffer queue
//         std::lock_guard<std::mutex> lck(m_mutexBufferAccess);
//         // push our buffer into the lag queue
//         m_lag_buffers.push(tmp_buffer);
//         // prevent lag buffer overflow - remove any super old buffers that have expired. We don't want the lag_buffer to
//         // grow unbounded
//         while (m_lag_buffers.size() > m_max_lag_buffers) {
//             m_empty_lag_buffers.push(
//                 m_lag_buffers.front());  // push the buffer back for efficiency if it wasn't given to the user
//             m_lag_buffers.pop();
//         }
//     }
// }

}  // namespace sensor
}  // namespace chrono
