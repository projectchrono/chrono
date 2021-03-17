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



#include "chrono_sensor/filters/ChFilterRadarPCfromRange.h"
#include "chrono_sensor/ChRadarSensor.h"
#include "chrono_sensor/utils/CudaMallocHelper.h"
#include "chrono_sensor/cuda/pointcloud.cuh"

namespace chrono {
namespace sensor {

    ChFilterRadarPCfromRange::ChFilterRadarPCfromRange(std::string name) : ChFilter(name) {}

    CH_SENSOR_API void ChFilterRadarPCfromRange::Apply(std::shared_ptr<ChSensor> pSensor,
                                                  std::shared_ptr<SensorBuffer>& bufferInOut){
        assert(bufferInOut != nullptr);
        if(!bufferInOut)
            throw std::runtime_error("The Radar Pointcloud fiter was not supplied an input buffer");

        // The sensor must be a radar 
        std::shared_ptr<ChRadarSensor> pRadar = std::dynamic_pointer_cast<ChRadarSensor>(pSensor);
        if (!pRadar){
            throw std::runtime_error("This sensor must be a radar");
        }

        // get the pointer to the memory either from optix or from our device buffer
        void* ptr;
        if (auto pOpx = std::dynamic_pointer_cast<SensorOptixBuffer>(bufferInOut)){
            if (pOpx->Buffer->getFormat() != RT_FORMAT_FLOAT2){
                std::cout<<"Incoming Optix format: "<<pOpx->Buffer->getFormat()<<std::endl;
                throw std::runtime_error(
                    "The only Optix format that can be converted to pointcloud is FLOAT2 (Range and RCS)"
                );
            }
            // we need id of first device for this context (should only have 1 anyway2)
            int device_id = pOpx->Buffer->getContext()->getEnabledDevices()[0];
            ptr = pOpx->Buffer->getDevicePointer(device_id); //hard coded to grab from device 0
        } else if (auto pRI = std::dynamic_pointer_cast<SensorDeviceRangeRcsBuffer>(bufferInOut)){
            ptr = (void*)pRI->Buffer.get();
        } else {
            throw std::runtime_error("The pointcloud filter cannot be run on the requested input buffer type");
        }

        if (!m_buffer){
            m_buffer = chrono_types::make_shared<SensorDeviceXYZIBuffer>();
            DeviceXYZIBufferPtr b(cudaMallocHelper<PixelXYZI>(bufferInOut->Width * bufferInOut->Height), 
                                  cudaFreeHelper<PixelXYZI>);
            m_buffer->Buffer = std::move(b);
            m_buffer->Width = bufferInOut->Width;
            m_buffer->Height = bufferInOut->Height;
        }

        cuda_pointcloud_from_depth(ptr, m_buffer->Buffer.get(), (int)bufferInOut->Width, (int)bufferInOut->Height, 
                                   pRadar->GetHFOV(), pRadar->GetMaxVertAngle(), pRadar->GetMinVertAngle());

        std::vector<float> buf(m_buffer->Width * m_buffer->Height * 4);
        std::vector<float> new_buf(m_buffer->Width * m_buffer->Height * 4);

        cudaMemcpy(buf.data(), m_buffer->Buffer.get(), m_buffer->Width * m_buffer->Height * sizeof(float) * 4, cudaMemcpyDeviceToHost);

        m_buffer->Beam_return_count = 0;
        for (unsigned int i = 0; i < m_buffer->Width * m_buffer->Height; i++){
            if(buf.data()[i * 4 + 3] > 0){
                new_buf.data()[m_buffer->Beam_return_count * 4] = buf.data()[i * 4];
                new_buf.data()[m_buffer->Beam_return_count * 4 + 1] = buf.data()[i * 4 + 1];
                new_buf.data()[m_buffer->Beam_return_count * 4 + 2] = buf.data()[i * 4 + 2];
                new_buf.data()[m_buffer->Beam_return_count * 4 + 3] = buf.data()[i * 4 + 3];
                m_buffer->Beam_return_count++;
            }
        }
        cudaMemcpy(m_buffer->Buffer.get(), new_buf.data(), m_buffer->Beam_return_count * sizeof(float) * 4, cudaMemcpyHostToDevice);
        
        m_buffer->LaunchedCount = bufferInOut->LaunchedCount;
        m_buffer->TimeStamp = bufferInOut->TimeStamp;
        bufferInOut = m_buffer;
    }

}
}