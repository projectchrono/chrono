#include "chrono_sensor/filters/ChFilterRadarXYZReturn.h"
#include "chrono_sensor/utils/CudaMallocHelper.h"
#include "chrono_sensor/cuda/radarprocess.cuh"
#include "chrono_sensor/utils/Dbscan.h"
#include <random>

namespace chrono{
namespace sensor{

ChFilterRadarXYZReturn::ChFilterRadarXYZReturn(std::string name) : ChFilter(name){}

CH_SENSOR_API void ChFilterRadarXYZReturn::Initialize(std::shared_ptr<ChSensor> pSensor, 
                                                      std::shared_ptr<SensorBuffer>& bufferInOut){
    if (!bufferInOut)
        InvalidFilterGraphNullBuffer(pSensor);

    m_buffer_in = std::dynamic_pointer_cast<SensorDeviceRadarBuffer>(bufferInOut);

    if (!m_buffer_in)
        InvalidFilterGraphBufferTypeMismatch(pSensor);

    if (auto pRadar = std::dynamic_pointer_cast<ChRadarSensor>(pSensor)) {
        m_cuda_stream = pRadar->GetCudaStream();
        m_hFOV = pRadar->GetHFOV();
        m_max_vert_angle = pRadar->GetMaxVertAngle();
        m_min_vert_angle = pRadar->GetMinVertAngle();
    } else {
        InvalidFilterGraphSensorTypeMismatch(pSensor);
    }
    m_radar = std::dynamic_pointer_cast<ChRadarSensor>(pSensor);
    m_buffer_out = chrono_types::make_shared<SensorDeviceRadarXYZBuffer>();
    std::shared_ptr<RadarXYZReturn[]> b(
        cudaHostMallocHelper<RadarXYZReturn>(m_buffer_in->Width * m_buffer_in->Height),
        cudaHostFreeHelper<RadarXYZReturn>);
    m_buffer_out->Buffer = std::move(b);
    m_buffer_out->Width = bufferInOut->Width;
    m_buffer_out->Height = bufferInOut->Height;
    bufferInOut = m_buffer_out;

}

CH_SENSOR_API void ChFilterRadarXYZReturn::Apply(){

    // converts azimuth and elevation to XYZ Coordinates in device
    cuda_radar_pointcloud_from_angles(m_buffer_in->Buffer.get(), m_buffer_out->Buffer.get(), 
                                      (int)m_buffer_in->Width, (int)m_buffer_in->Height, m_hFOV, m_max_vert_angle, m_min_vert_angle,
                                      m_cuda_stream);

    // Transfer pointcloud to host
    auto buf = std::vector<RadarXYZReturn>(m_buffer_out->Width * m_buffer_out->Height);
    cudaMemcpyAsync(buf.data(), m_buffer_out->Buffer.get(),
                    m_buffer_out->Width * m_buffer_out->Height * sizeof(RadarXYZReturn), cudaMemcpyDeviceToHost,
                    m_cuda_stream);
    cudaStreamSynchronize(m_cuda_stream);

    auto filtered_buf = std::vector<RadarXYZReturn>();
    m_buffer_out->Beam_return_count = 0;
    for (RadarXYZReturn point : buf){
        // remove rays with no returns
        if (point.amplitude > 0){
            filtered_buf.push_back(point);
            m_buffer_out->Beam_return_count+=1;
        }
    }

    cudaMemcpyAsync(m_buffer_out->Buffer.get(), filtered_buf.data(),
           m_buffer_out->Beam_return_count * sizeof(RadarXYZReturn), cudaMemcpyHostToDevice,
           m_cuda_stream);

    m_buffer_out->LaunchedCount = m_buffer_in->LaunchedCount;
    m_buffer_out->TimeStamp = m_buffer_in->TimeStamp;
}
}
}