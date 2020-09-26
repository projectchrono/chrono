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
#include "chrono_sensor/ChSensor.h"
#include "chrono_sensor/utils/CudaMallocHelper.h"

#include <cuda.h>

namespace chrono {
namespace sensor {

template <>
CH_SENSOR_API void ChFilterAccess<SensorHostR8Buffer, UserR8BufferPtr>::Apply(
    std::shared_ptr<ChSensor> pSensor,
    std::shared_ptr<SensorBuffer>& bufferInOut) {
    // to copy to a host buffer, we need to know what buffer to copy.
    // for now, that means this filter can only with buffers that are of type R8 Device (GPU).
    std::shared_ptr<SensorDeviceR8Buffer> pDev = std::dynamic_pointer_cast<SensorDeviceR8Buffer>(bufferInOut);
    if (!pDev) {
        throw std::runtime_error("cannot copy supplied buffer type to a Host R8 buffer.");
    }

    unsigned int sz = bufferInOut->Width * bufferInOut->Height;

    // create a new buffer to push to the lag buffer list
    std::shared_ptr<SensorHostR8Buffer> tmp_buffer;
    if (m_empty_lag_buffers.size() > 0) {
        tmp_buffer = m_empty_lag_buffers.top();
        m_empty_lag_buffers.pop();
    } else {
        tmp_buffer = chrono_types::make_shared<SensorHostR8Buffer>();
        tmp_buffer->Buffer = std::make_unique<char[]>(sz);
    }

    tmp_buffer->Width = bufferInOut->Width;
    tmp_buffer->Height = bufferInOut->Height;
    tmp_buffer->LaunchedCount = bufferInOut->LaunchedCount;
    tmp_buffer->TimeStamp = bufferInOut->TimeStamp;

    cudaMemcpy(tmp_buffer->Buffer.get(), pDev->Buffer.get(), sz, cudaMemcpyDeviceToHost);

    {  // lock in this scope before pushing to lag buffer queue
        std::lock_guard<std::mutex> lck(m_mutexBufferAccess);

        // push our buffer into the lag queue
        m_lag_buffers.push(tmp_buffer);

        // std::cout << "Pushed filter with timestamp= " << m_buffer.TimeStamp << std::endl;

        // prevent lag buffer overflow - remove any old buffers that have expired. We don't want the lag_buffer to
        // grow unbounded
        while (m_lag_buffers.size() > m_max_lag_buffers) {
            m_empty_lag_buffers.push(
                m_lag_buffers.front());  // push the buffer back for efficiency if it wasn't given to the user
            m_lag_buffers.pop();
        }
    }
}

template <>
CH_SENSOR_API void ChFilterAccess<SensorHostRGBA8Buffer, UserRGBA8BufferPtr>::Apply(
    std::shared_ptr<ChSensor> pSensor,
    std::shared_ptr<SensorBuffer>& bufferInOut) {
    // to copy to a host buffer, we need to know what buffer to copy.
    // for now, that means this filter can only work with sensor that use an Optix buffer.
    std::shared_ptr<SensorOptixBuffer> pOpx = std::dynamic_pointer_cast<SensorOptixBuffer>(bufferInOut);
    std::shared_ptr<SensorDeviceRGBA8Buffer> pRGBA8 = std::dynamic_pointer_cast<SensorDeviceRGBA8Buffer>(bufferInOut);
    if (!pOpx && !pRGBA8) {
        throw std::runtime_error("cannot copy supplied buffer type to a Host RGBA8 buffer.");
    }

    void* dev_buffer_ptr;

    // if we have an optix buffer
    if (pOpx) {
        int device_id = pOpx->Buffer->getContext()->getEnabledDevices()[0];  // TODO: issue with mulitple GPUs
        dev_buffer_ptr = pOpx->Buffer->getDevicePointer(device_id);
    }
    // if we have a device buffer
    else if (pRGBA8) {
        dev_buffer_ptr = (void*)(pRGBA8->Buffer.get());
    }

    unsigned int sz = bufferInOut->Width * bufferInOut->Height;

    // create a new buffer to push to the lag buffer list
    std::shared_ptr<SensorHostRGBA8Buffer> tmp_buffer;
    if (m_empty_lag_buffers.size() > 0) {
        tmp_buffer = m_empty_lag_buffers.top();
        m_empty_lag_buffers.pop();
    } else {
        tmp_buffer = chrono_types::make_shared<SensorHostRGBA8Buffer>();
        tmp_buffer->Buffer = std::make_unique<PixelRGBA8[]>(sz);
    }

    tmp_buffer->Width = bufferInOut->Width;
    tmp_buffer->Height = bufferInOut->Height;
    tmp_buffer->LaunchedCount = bufferInOut->LaunchedCount;
    tmp_buffer->TimeStamp = bufferInOut->TimeStamp;

    cudaMemcpy(tmp_buffer->Buffer.get(), dev_buffer_ptr, sz * sizeof(PixelRGBA8), cudaMemcpyDeviceToHost);

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
    }
}

template <>
CH_SENSOR_API void ChFilterAccess<SensorHostXYZIBuffer, UserXYZIBufferPtr>::Apply(
    std::shared_ptr<ChSensor> pSensor,
    std::shared_ptr<SensorBuffer>& bufferInOut) {
    // to copy to a host buffer, we need to know what buffer to copy.
    // for now, that means this filter can only with buffers that are of type R8 Device (GPU).
    std::shared_ptr<SensorDeviceXYZIBuffer> pDev = std::dynamic_pointer_cast<SensorDeviceXYZIBuffer>(bufferInOut);
    if (!pDev) {
        throw std::runtime_error("cannot copy supplied buffer type to a Host XYZI buffer.");
    }

    // if we've never allocated the buffer, or if the user 'took' the buffer last time
    // s/he retrieved it, we need to allocate it.
    unsigned int sz = bufferInOut->Width * bufferInOut->Height;

    // create a new buffer to push to the lag buffer list
    std::shared_ptr<SensorHostXYZIBuffer> tmp_buffer;
    if (m_empty_lag_buffers.size() > 0) {
        tmp_buffer = m_empty_lag_buffers.top();
        m_empty_lag_buffers.pop();
    } else {
        tmp_buffer = chrono_types::make_shared<SensorHostXYZIBuffer>();
        tmp_buffer->Buffer = std::make_unique<PixelXYZI[]>(sz);
    }

    tmp_buffer->Width = bufferInOut->Width;
    tmp_buffer->Height = bufferInOut->Height;
    tmp_buffer->LaunchedCount = bufferInOut->LaunchedCount;
    tmp_buffer->TimeStamp = bufferInOut->TimeStamp;

    cudaMemcpy(tmp_buffer->Buffer.get(), pDev->Buffer.get(), sz * sizeof(PixelXYZI), cudaMemcpyDeviceToHost);

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
    }
}

template <>
CH_SENSOR_API void ChFilterAccess<SensorHostDIBuffer, UserDIBufferPtr>::Apply(
    std::shared_ptr<ChSensor> pSensor,
    std::shared_ptr<SensorBuffer>& bufferInOut) {
    // to copy to a host buffer, we need to know what buffer to copy.
    // for now, that means this filter can only with buffers that are of type R8 Device (GPU).
    std::shared_ptr<SensorOptixBuffer> pOpx = std::dynamic_pointer_cast<SensorOptixBuffer>(bufferInOut);
    std::shared_ptr<SensorDeviceDIBuffer> pDev = std::dynamic_pointer_cast<SensorDeviceDIBuffer>(bufferInOut);
    if (!pOpx && !pDev) {
        throw std::runtime_error("cannot copy supplied buffer type to a Host DI buffer.");
    }

    void* dev_buffer_ptr;

    // if we have an optix buffer
    if (pOpx) {
        int device_id = pOpx->Buffer->getContext()->getEnabledDevices()[0];  // TODO: issue with mulitple GPUs
        dev_buffer_ptr = pOpx->Buffer->getDevicePointer(device_id);
    }
    // if we have a device buffer
    else if (pDev) {
        dev_buffer_ptr = (void*)(pDev->Buffer.get());
    }

    unsigned int sz = bufferInOut->Width * bufferInOut->Height;

    // create a new buffer to push to the lag buffer list
    std::shared_ptr<SensorHostDIBuffer> tmp_buffer;
    if (m_empty_lag_buffers.size() > 0) {
        tmp_buffer = m_empty_lag_buffers.top();
        m_empty_lag_buffers.pop();
    } else {
        tmp_buffer = chrono_types::make_shared<SensorHostDIBuffer>();
        tmp_buffer->Buffer = std::make_unique<PixelDI[]>(sz);
    }

    tmp_buffer->Width = bufferInOut->Width;
    tmp_buffer->Height = bufferInOut->Height;
    tmp_buffer->LaunchedCount = bufferInOut->LaunchedCount;
    tmp_buffer->TimeStamp = bufferInOut->TimeStamp;

    cudaMemcpy(tmp_buffer->Buffer.get(), dev_buffer_ptr, sz * sizeof(PixelDI), cudaMemcpyDeviceToHost);

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
    }
}

template <>
CH_SENSOR_API void ChFilterAccess<SensorHostIMUBuffer, UserIMUBufferPtr>::Apply(
    std::shared_ptr<ChSensor> pSensor,
    std::shared_ptr<SensorBuffer>& bufferInOut) {
    // to copy to a host buffer, we need to know what buffer to copy.
    // for now, that means this filter can only work with sensor that use an Optix buffer.
    // std::cout<<"Apply Method Called \n";
    std::shared_ptr<SensorHostIMUBuffer> pIMU = std::dynamic_pointer_cast<SensorHostIMUBuffer>(bufferInOut);
    if (!pIMU) {
        throw std::runtime_error("cannot copy supplied buffer type to a Host IMU buffer.");
    }

    // create a new buffer to push to the lag buffer list
    std::shared_ptr<SensorHostIMUBuffer> tmp_buffer;
    if (m_empty_lag_buffers.size() > 0) {
        tmp_buffer = m_empty_lag_buffers.top();
        m_empty_lag_buffers.pop();
    } else {
        tmp_buffer = chrono_types::make_shared<SensorHostIMUBuffer>();
        tmp_buffer->Buffer = std::make_unique<IMUData[]>(1);
    }

    tmp_buffer->Width = bufferInOut->Width;
    tmp_buffer->Height = bufferInOut->Height;
    tmp_buffer->LaunchedCount = bufferInOut->LaunchedCount;
    tmp_buffer->TimeStamp = bufferInOut->TimeStamp;

    // copy the data into our new buffer
    memcpy(tmp_buffer->Buffer.get(), pIMU->Buffer.get(), sizeof(IMUData));

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
CH_SENSOR_API void ChFilterAccess<SensorHostGPSBuffer, UserGPSBufferPtr>::Apply(
    std::shared_ptr<ChSensor> pSensor,
    std::shared_ptr<SensorBuffer>& bufferInOut) {
    // to copy to a host buffer, we need to know what buffer to copy.
    // for now, that means this filter can only work with sensor that use an Optix buffer.
    std::shared_ptr<SensorHostGPSBuffer> pGPS = std::dynamic_pointer_cast<SensorHostGPSBuffer>(bufferInOut);
    if (!pGPS) {
        throw std::runtime_error("cannot copy supplied buffer type to a Host GPS buffer.");
    }

    // create a new buffer to push to the lag buffer list
    std::shared_ptr<SensorHostGPSBuffer> tmp_buffer;
    if (m_empty_lag_buffers.size() > 0) {
        tmp_buffer = m_empty_lag_buffers.top();
        m_empty_lag_buffers.pop();
    } else {
        tmp_buffer = chrono_types::make_shared<SensorHostGPSBuffer>();
        tmp_buffer->Buffer = std::make_unique<GPSData[]>(1);
    }

    tmp_buffer->Width = bufferInOut->Width;
    tmp_buffer->Height = bufferInOut->Height;
    tmp_buffer->LaunchedCount = bufferInOut->LaunchedCount;
    tmp_buffer->TimeStamp = bufferInOut->TimeStamp;

    // copy the data into our new buffer
    memcpy(tmp_buffer->Buffer.get(), pGPS->Buffer.get(), sizeof(GPSData));

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

}  // namespace sensor
}  // namespace chrono
