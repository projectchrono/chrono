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
// Authors: Bo-Hsun Chen
// =============================================================================
// 
// Filter to do defocus blur based on camera control parameters and depth map
// 
// =============================================================================

#include "chrono_sensor/ChConfigSensor.h"
#if (defined(CHRONO_HAS_VULKAN_RT) || defined(CHRONO_HAS_METAL_RT)) && !defined(CHRONO_HAS_OPTIX)

    #include "chrono_sensor/filters/ChFilterPhysCameraDefocusBlur.h"
    #ifdef CHRONO_HAS_METAL_RT
        #include "chrono_sensor/metal/ChMetalPhysCamOps.h"
    #endif

    #include <algorithm>
    #include <cmath>
    #include <memory>

namespace chrono {
namespace sensor {
namespace {
inline float gaussian_1d(int x, int kernel_size, float sigma) {
    if (sigma <= 0.f)
        return 0.f;
    constexpr float pi = 3.14159265358979323846f;
    const float coeff = 1.0f / (std::sqrt(2.0f * pi) * sigma);
    const float exponent = -static_cast<float>(x * x) / (2.0f * sigma * sigma);
    return coeff * std::exp(exponent);
}
}

ChFilterPhysCameraDefocusBlur::ChFilterPhysCameraDefocusBlur(float focal_length,
                                                             float focus_dist,
                                                             float aperture_num,
                                                             float pixel_size,
                                                             float defocus_gain,
                                                             float defocus_bias,
                                                             std::string name)
    : m_focal_length(focal_length),
      m_focus_dist(focus_dist),
      m_pixel_size(pixel_size),
      m_aperture_num(aperture_num),
      m_defocus_gain(defocus_gain),
      m_defocus_bias(defocus_bias),
      ChFilter(name) {}

CH_SENSOR_API void ChFilterPhysCameraDefocusBlur::Initialize(std::shared_ptr<ChSensor> pSensor,
                                                             std::shared_ptr<SensorBuffer>& bufferInOut) {
    if (!bufferInOut)
        InvalidFilterGraphNullBuffer(pSensor);
    m_buffer_in = std::dynamic_pointer_cast<SensorDeviceRGBDHalf4Buffer>(bufferInOut);
    if (!m_buffer_in) {
        InvalidFilterGraphBufferTypeMismatch(pSensor);
        return;
    }
    m_buffer_out = chrono_types::make_shared<SensorDeviceHalf4Buffer>();
    m_buffer_out->Width = m_buffer_in->Width;
    m_buffer_out->Height = m_buffer_in->Height;
    m_buffer_out->LaunchedCount = m_buffer_in->LaunchedCount;
    m_buffer_out->TimeStamp = m_buffer_in->TimeStamp;
    m_buffer_out->Buffer = std::shared_ptr<PixelHalf4[]>(new PixelHalf4[static_cast<size_t>(m_buffer_out->Width) * m_buffer_out->Height]);
    bufferInOut = m_buffer_out;
}

CH_SENSOR_API void ChFilterPhysCameraDefocusBlur::Apply() {
    if (!m_buffer_in || !m_buffer_out || !m_buffer_in->Buffer || !m_buffer_out->Buffer)
        return;
    const unsigned int img_w = m_buffer_in->Width;
    const unsigned int img_h = m_buffer_in->Height;
    m_buffer_out->Width = img_w;
    m_buffer_out->Height = img_h;
    m_buffer_out->LaunchedCount = m_buffer_in->LaunchedCount;
    m_buffer_out->TimeStamp = m_buffer_in->TimeStamp;
    #ifdef CHRONO_HAS_METAL_RT
    // GPU path (port of cuda_phys_cam_defocus_blur). This stage is O(w*h*k^2), by far the most
    // expensive of the five, so the CPU loop below is only a fallback when Metal is unavailable.
    if (metal_phys_cam::DefocusBlur(m_buffer_in->Buffer.get(), m_buffer_out->Buffer.get(), img_w, img_h, m_focal_length, m_focus_dist, m_aperture_num, m_pixel_size, m_defocus_gain,
                                    m_defocus_bias))
        return;
    #endif
    const float denom_const = m_aperture_num * m_pixel_size;
    for (unsigned int y = 0; y < img_h; ++y) {
        for (unsigned int x = 0; x < img_w; ++x) {
            const size_t idx = static_cast<size_t>(y) * img_w + x;
            const auto& src = m_buffer_in->Buffer[idx];
            int kernel_size = 0;
            const float d = src.D;
            const float denom = denom_const * d * (m_focus_dist - m_focal_length);
            if (d > 1e-9f && std::abs(denom) > 1e-20f) {
                kernel_size = static_cast<int>(std::ceil(m_focal_length * m_focal_length * std::abs(d - m_focus_dist) / denom));
                kernel_size = std::abs(kernel_size);
            }
            PixelHalf4 out{};
            if (kernel_size > 1) {
                kernel_size = std::max(1, static_cast<int>(m_defocus_gain * kernel_size + m_defocus_bias));
                if ((kernel_size % 2) == 0)
                    ++kernel_size;
                const int r = (kernel_size - 1) / 2;
                const int x_min = std::max(0, static_cast<int>(x) - r);
                const int x_max = std::min(static_cast<int>(img_w) - 1, static_cast<int>(x) + r);
                const int y_min = std::max(0, static_cast<int>(y) - r);
                const int y_max = std::min(static_cast<int>(img_h) - 1, static_cast<int>(y) + r);
                const float sigma = static_cast<float>(kernel_size) / 6.f;
                for (int yy = y_min; yy <= y_max; ++yy) {
                    const float gy = gaussian_1d(yy - static_cast<int>(y), kernel_size, sigma);
                    for (int xx = x_min; xx <= x_max; ++xx) {
                        const float w = gy * gaussian_1d(xx - static_cast<int>(x), kernel_size, sigma);
                        const auto& p = m_buffer_in->Buffer[static_cast<size_t>(yy) * img_w + static_cast<unsigned int>(xx)];
                        out.R += p.R * w;
                        out.G += p.G * w;
                        out.B += p.B * w;
                    }
                }
            } else {
                out.R = src.R;
                out.G = src.G;
                out.B = src.B;
            }
            out.A = 1.f;
            m_buffer_out->Buffer[idx] = out;
        }
    }
}

CH_SENSOR_API void ChFilterPhysCameraDefocusBlur::SetFilterCtrlParameters(float focal_length, float focus_dist, float aperture_num) {
    m_focal_length = focal_length;
    m_focus_dist = focus_dist;
    m_aperture_num = aperture_num;
}

CH_SENSOR_API void ChFilterPhysCameraDefocusBlur::SetFilterModelParameters(float pixel_size, float defocus_gain, float defocus_bias) {
    m_pixel_size = pixel_size;
    m_defocus_gain = defocus_gain;
    m_defocus_bias = defocus_bias;
}

}  // namespace sensor
}  // namespace chrono

#else


#include "chrono_sensor/filters/ChFilterPhysCameraDefocusBlur.h"
#include "chrono_sensor/sensors/ChOptixSensor.h"
#include "chrono_sensor/cuda/phys_cam_ops.cuh"
#include "chrono_sensor/utils/CudaMallocHelper.h"
#include <chrono>

namespace chrono {
namespace sensor {
ChFilterPhysCameraDefocusBlur::ChFilterPhysCameraDefocusBlur(
    float focal_length, float focus_dist, float aperture_num, float pixel_size, float defocus_gain, float defocus_bias,
    std::string name
) :
    m_focal_length(focal_length), m_focus_dist(focus_dist), m_aperture_num(aperture_num), m_pixel_size(pixel_size),
    m_defocus_gain(defocus_gain), m_defocus_bias(defocus_bias), ChFilter(name)
{}

CH_SENSOR_API void ChFilterPhysCameraDefocusBlur::Initialize(
    std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut
) {
    if (!bufferInOut) {
        InvalidFilterGraphNullBuffer(pSensor);
    }
    
    if (auto pRGBD = std::dynamic_pointer_cast<SensorDeviceRGBDHalf4Buffer>(bufferInOut)) {
        m_buffer_in = pRGBD;
    }
    else {
        InvalidFilterGraphBufferTypeMismatch(pSensor);
    }

    if (auto pOpx = std::dynamic_pointer_cast<ChOptixSensor>(pSensor)) {
        m_cuda_stream = pOpx->GetCudaStream();
    }

    // make new buffer for output
    m_buffer_out = chrono_types::make_shared<SensorDeviceHalf4Buffer>();
    DeviceHalf4BufferPtr b(
        cudaMallocHelper<PixelHalf4>(bufferInOut->Width * bufferInOut->Height), cudaFreeHelper<PixelHalf4>
    );
    m_buffer_out->Buffer = std::move(b);
    m_buffer_out->Width = bufferInOut->Width;
    m_buffer_out->Height = bufferInOut->Height;
    m_buffer_out->LaunchedCount = bufferInOut->LaunchedCount;
    m_buffer_out->TimeStamp = bufferInOut->TimeStamp;
    bufferInOut = m_buffer_out;
}

CH_SENSOR_API void ChFilterPhysCameraDefocusBlur::Apply() {
    m_buffer_out->Width = m_buffer_in->Width;
    m_buffer_out->Height = m_buffer_in->Height;
    m_buffer_out->LaunchedCount = m_buffer_in->LaunchedCount;
    m_buffer_out->TimeStamp = m_buffer_in->TimeStamp;

    // perform defocus blur operation in phys_cam_ops.cu
    cuda_phys_cam_defocus_blur(
        m_buffer_in->Buffer.get(), m_buffer_out->Buffer.get(), m_buffer_out->Width, m_buffer_out->Height,
        m_focal_length, m_focus_dist, m_aperture_num, m_pixel_size, m_defocus_gain, m_defocus_bias, m_cuda_stream
    );
}

CH_SENSOR_API void ChFilterPhysCameraDefocusBlur::SetFilterCtrlParameters(
    float focal_length, float focus_dist, float aperture_num
) {
    m_focal_length = focal_length;
    m_focus_dist = focus_dist;
    m_aperture_num = aperture_num;
}

CH_SENSOR_API void ChFilterPhysCameraDefocusBlur::SetFilterModelParameters(
    float pixel_size, float defocus_gain, float defocus_bias
) {
    m_pixel_size = pixel_size;
    m_defocus_gain = defocus_gain;
    m_defocus_bias = defocus_bias;
}

}  // namespace sensor
}  // namespace chrono


#endif
