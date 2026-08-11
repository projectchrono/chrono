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
// Authors: Asher Elmquist
// =============================================================================
//
//
// =============================================================================

#include "chrono_sensor/filters/ChFilterSavePtCloud.h"
#include "chrono_sensor/sensors/ChLidarSensor.h"

#include <iostream>
#include <sstream>
#include <vector>

#include "chrono/core/ChDataPath.h"
#include "chrono/input_output/ChWriterCSV.h"

#ifdef CHRONO_HAS_OPTIX
#include "chrono_sensor/sensors/ChOptixSensor.h"
#include "chrono_sensor/utils/CudaMallocHelper.h"
#include <cuda_runtime_api.h>
#endif

namespace chrono {
namespace sensor {

namespace {

void EnsureDirectoryTree(const std::string& path) {
    std::vector<std::string> split_string;
#ifdef _WIN32
    const char separator = '\\';
#else
    const char separator = '/';
#endif
    std::istringstream istring(path);
    std::string substring;
    while (std::getline(istring, substring, separator))
        split_string.push_back(substring);

    std::string partial_path;
    for (auto s : split_string) {
        if (!s.empty()) {
            partial_path += s + separator;
            if (!exists(std::filesystem::path(partial_path))) {
                if (!CreateOutputDirectory(std::filesystem::path(partial_path)))
                    std::cerr << "Could not create directory: " << partial_path << std::endl;
                else
                    std::cout << "Created directory for sensor data: " << partial_path << std::endl;
            }
        }
    }
}

}  // namespace

ChFilterSavePtCloud::ChFilterSavePtCloud(std::string data_path, std::string name) : ChFilter(name), m_path(data_path) {}
ChFilterSavePtCloud::~ChFilterSavePtCloud() {}

void ChFilterSavePtCloud::Apply() {
#ifdef CHRONO_HAS_OPTIX
    cudaMemcpyAsync(m_host_buffer->Buffer.get(), m_buffer_in->Buffer.get(),
                    sizeof(PixelXYZI) * m_host_buffer->Width * m_host_buffer->Height * (m_host_buffer->Dual_return + 1),
                    cudaMemcpyDeviceToHost, m_cuda_stream);
    cudaStreamSynchronize(m_cuda_stream);
#else
    m_host_buffer = m_buffer_in;
#endif

    std::string filename = m_path + "frame_" + std::to_string(m_frame_number) + ".csv";
    ++m_frame_number;
    ChWriterCSV csv_writer(",");
    for (unsigned int i = 0; i < m_buffer_in->Beam_return_count; i++) {
        csv_writer << m_host_buffer->Buffer[i].x << m_host_buffer->Buffer[i].y << m_host_buffer->Buffer[i].z
                   << m_host_buffer->Buffer[i].intensity << std::endl;
    }
    csv_writer.WriteToFile(filename);
}

void ChFilterSavePtCloud::Initialize(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut) {
    if (!bufferInOut)
        InvalidFilterGraphNullBuffer(pSensor);

    m_buffer_in = std::dynamic_pointer_cast<SensorDeviceXYZIBuffer>(bufferInOut);
    if (!m_buffer_in)
        InvalidFilterGraphBufferTypeMismatch(pSensor);

    if (auto pLidar = std::dynamic_pointer_cast<ChLidarSensor>(pSensor)) {
#ifdef CHRONO_HAS_OPTIX
        m_cuda_stream = pLidar->GetCudaStream();
#endif
    } else {
        InvalidFilterGraphSensorTypeMismatch(pSensor);
    }

#ifdef CHRONO_HAS_OPTIX
    m_host_buffer = chrono_types::make_shared<SensorHostXYZIBuffer>();
    std::shared_ptr<PixelXYZI[]> b(
        cudaHostMallocHelper<PixelXYZI>(m_buffer_in->Width * m_buffer_in->Height * (m_buffer_in->Dual_return + 1)),
        cudaHostFreeHelper<PixelXYZI>);
    m_host_buffer->Buffer = std::move(b);
    m_host_buffer->Width = m_buffer_in->Width;
    m_host_buffer->Height = m_buffer_in->Height;
    m_host_buffer->Dual_return = m_buffer_in->Dual_return;
#else
    m_host_buffer = m_buffer_in;
#endif

    EnsureDirectoryTree(m_path);
}

}  // namespace sensor
}  // namespace chrono
