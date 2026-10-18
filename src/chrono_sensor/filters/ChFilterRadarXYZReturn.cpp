#include "chrono_sensor/filters/ChFilterRadarXYZReturn.h"

#ifdef CHRONO_HAS_OPTIX
#include "chrono_sensor/cuda/radarprocess.cuh"
#include "chrono_sensor/utils/CudaMallocHelper.h"
#endif

#include <cmath>
#include <vector>

namespace chrono {
namespace sensor {

namespace {

RadarXYZReturn RadarToXYZ(const RadarReturn& in) {
    const float proj_xy = in.range * std::cos(in.elevation);
    RadarXYZReturn out{};
    out.x = proj_xy * std::cos(in.azimuth);
    out.y = proj_xy * std::sin(in.azimuth);
    out.z = in.range * std::sin(in.elevation);
    out.vel_x = in.doppler_velocity[0];
    out.vel_y = in.doppler_velocity[1];
    out.vel_z = in.doppler_velocity[2];
    out.amplitude = in.amplitude;
    out.objectId = in.objectId;
    return out;
}

}  // namespace

ChFilterRadarXYZReturn::ChFilterRadarXYZReturn(std::string name) : ChFilter(name) {}

void ChFilterRadarXYZReturn::Initialize(std::shared_ptr<ChSensor> pSensor,
                                        std::shared_ptr<SensorBuffer>& bufferInOut) {
    if (!bufferInOut)
        InvalidFilterGraphNullBuffer(pSensor);

    m_buffer_in = std::dynamic_pointer_cast<SensorDeviceRadarBuffer>(bufferInOut);
    if (!m_buffer_in)
        InvalidFilterGraphBufferTypeMismatch(pSensor);

    if (auto pRadar = std::dynamic_pointer_cast<ChRadarSensor>(pSensor)) {
#ifdef CHRONO_HAS_OPTIX
        m_cuda_stream = pRadar->GetCudaStream();
#endif
        m_hFOV = static_cast<float>(pRadar->GetHFOV());
        m_vFOV = static_cast<float>(pRadar->GetVFOV());
        m_radar = pRadar;
    } else {
        InvalidFilterGraphSensorTypeMismatch(pSensor);
    }

    m_buffer_out = chrono_types::make_shared<SensorDeviceRadarXYZBuffer>();
#ifdef CHRONO_HAS_OPTIX
    std::shared_ptr<RadarXYZReturn[]> b(cudaHostMallocHelper<RadarXYZReturn>(m_buffer_in->Width * m_buffer_in->Height),
                                        cudaHostFreeHelper<RadarXYZReturn>);
#else
    std::shared_ptr<RadarXYZReturn[]> b(new RadarXYZReturn[static_cast<size_t>(m_buffer_in->Width) * m_buffer_in->Height]);
#endif
    m_buffer_out->Buffer = std::move(b);
    m_buffer_out->Width = bufferInOut->Width;
    m_buffer_out->Height = bufferInOut->Height;
    bufferInOut = m_buffer_out;
}

void ChFilterRadarXYZReturn::Apply() {
#ifdef CHRONO_HAS_OPTIX
    cuda_radar_pointcloud_from_angles(m_buffer_in->Buffer.get(), m_buffer_out->Buffer.get(), (int)m_buffer_in->Width,
                                      (int)m_buffer_in->Height, m_hFOV, m_vFOV, m_cuda_stream);

    auto buf = std::vector<RadarXYZReturn>(m_buffer_out->Width * m_buffer_out->Height);
    cudaMemcpyAsync(buf.data(), m_buffer_out->Buffer.get(),
                    m_buffer_out->Width * m_buffer_out->Height * sizeof(RadarXYZReturn), cudaMemcpyDeviceToHost,
                    m_cuda_stream);
    cudaStreamSynchronize(m_cuda_stream);

    auto filtered_buf = std::vector<RadarXYZReturn>();
    m_buffer_out->Beam_return_count = 0;
    for (RadarXYZReturn point : buf) {
        if (point.amplitude > 0) {
            filtered_buf.push_back(point);
            m_buffer_out->Beam_return_count += 1;
        }
    }
    cudaMemcpyAsync(m_buffer_out->Buffer.get(), filtered_buf.data(),
                    m_buffer_out->Beam_return_count * sizeof(RadarXYZReturn), cudaMemcpyHostToDevice, m_cuda_stream);
#else
    const size_t count = static_cast<size_t>(m_buffer_in->Width) * m_buffer_in->Height;
    int out_count = 0;
    for (size_t i = 0; i < count; ++i) {
        const RadarXYZReturn p = RadarToXYZ(m_buffer_in->Buffer[i]);
        if (p.amplitude > 0.f)
            m_buffer_out->Buffer[out_count++] = p;
    }
    m_buffer_out->Beam_return_count = out_count;
#endif
    m_buffer_out->LaunchedCount = m_buffer_in->LaunchedCount;
    m_buffer_out->TimeStamp = m_buffer_in->TimeStamp;
    m_buffer_out->invalid_returns = static_cast<int>(m_buffer_in->Width * m_buffer_in->Height) - m_buffer_out->Beam_return_count;
}
}  // namespace sensor
}  // namespace chrono