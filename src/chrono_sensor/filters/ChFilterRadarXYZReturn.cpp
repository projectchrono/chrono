#include "chrono_sensor/filters/ChFilterRadarXYZReturn.h"
#include "chrono_sensor/utils/CudaMallocHelper.h"
#include "chrono_sensor/cuda/radarprocess.cuh"

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
    // converts polar coord to cartesian
    cuda_radar_pointcloud_from_angles(m_buffer_in->Buffer.get(), m_buffer_out->Buffer.get(),
                                      (int)m_buffer_in->Width, (int)m_buffer_in->Height,
                                      m_hFOV, m_max_vert_angle, m_min_vert_angle,
                                      m_cuda_stream);
    
    // remove points with no return by transfering to host and filtering the points
    int hit_count = 0;
    auto buf = std::vector<RadarXYZReturn>(m_buffer_out->Width * m_buffer_out->Height);
    auto filtered_buf = std::vector<RadarXYZReturn>();

    cudaMemcpyAsync(buf.data(), m_buffer_out->Buffer.get(), 
                    m_buffer_out->Width * m_buffer_out->Height * sizeof(RadarXYZReturn), 
                    cudaMemcpyDeviceToHost, m_cuda_stream);
    cudaStreamSynchronize(m_cuda_stream);

    for (RadarXYZReturn point : buf){
        if (point.amplitude > 0){
            filtered_buf.push_back(point);
            hit_count += 1;
        }
    }
    m_buffer_out->Beam_return_count = hit_count;

    for (RadarXYZReturn point : filtered_buf){
        printf("%f %f %f\n", point.x, point.y, point.z);
    }

    // replace existing buffer with filtered buffer
    cudaMemcpyAsync(m_buffer_out->Buffer.get(), filtered_buf.data(), hit_count, cudaMemcpyHostToDevice, m_cuda_stream);
    cudaStreamSynchronize(m_cuda_stream);

    m_buffer_out->LaunchedCount = m_buffer_in->LaunchedCount;
    m_buffer_out->TimeStamp = m_buffer_in->TimeStamp;
}
}
}