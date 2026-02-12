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
#include "chrono_sensor/sensors/ChOptixSensor.h"

#include "chrono_thirdparty/filesystem/path.h"
#include "chrono/input_output/ChWriterCSV.h"

#include <vector>
#include <sstream>

#include <cuda_runtime_api.h>

namespace chrono {
namespace sensor {

CH_SENSOR_API ChFilterSavePtCloud::ChFilterSavePtCloud(std::string data_path, std::string name) : ChFilter(name) {
    m_path = data_path;
}

CH_SENSOR_API ChFilterSavePtCloud::~ChFilterSavePtCloud() {}

CH_SENSOR_API void ChFilterSavePtCloud::Apply() {
    cudaMemcpyAsync(m_host_buffer->Buffer.get(), m_buffer_in->Buffer.get(),
                    sizeof(PixelXYZI) * m_host_buffer->Width * m_host_buffer->Height * (m_host_buffer->Dual_return + 1),
                    cudaMemcpyDeviceToHost, m_cuda_stream);

    std::string filename = m_path + "frame_" + std::to_string(m_frame_number) + ".csv";
    m_frame_number++;
    ChWriterCSV csv_writer(",");
    cudaStreamSynchronize(m_cuda_stream);
    std::cout << "Beam count: " << m_buffer_in->Beam_return_count << std::endl;
    for (unsigned int i = 0; i < m_buffer_in->Beam_return_count; i++) {
        csv_writer << m_host_buffer->Buffer[i].x << m_host_buffer->Buffer[i].y << m_host_buffer->Buffer[i].z
                   << m_host_buffer->Buffer[i].intensity << std::endl;
    }
    csv_writer.WriteToFile(filename);
}

CH_SENSOR_API void ChFilterSavePtCloud::Initialize(std::shared_ptr<ChSensor> pSensor,
                                                   std::shared_ptr<SensorBuffer>& bufferInOut) {
    if (!bufferInOut)
        InvalidFilterGraphNullBuffer(pSensor);

    m_buffer_in = std::dynamic_pointer_cast<SensorDeviceXYZIBuffer>(bufferInOut);
    if (!m_buffer_in)
        InvalidFilterGraphBufferTypeMismatch(pSensor);

    if (auto pOpx = std::dynamic_pointer_cast<ChOptixSensor>(pSensor)) {
        m_cuda_stream = pOpx->GetCudaStream();
    } else {
        InvalidFilterGraphSensorTypeMismatch(pSensor);
    }

    m_host_buffer = chrono_types::make_shared<SensorHostXYZIBuffer>();
    std::shared_ptr<PixelXYZI[]> b(
        cudaHostMallocHelper<PixelXYZI>(m_buffer_in->Width * m_buffer_in->Height * (m_buffer_in->Dual_return + 1)),
        cudaHostFreeHelper<PixelXYZI>);
    m_host_buffer->Buffer = std::move(b);
    m_host_buffer->Width = m_buffer_in->Width;
    m_host_buffer->Height = m_buffer_in->Height;

    std::vector<std::string> split_string;
#ifdef _WIN32
    std::istringstream istring(m_path);

    std::string substring;
    while (std::getline(istring, substring, '\\')) {
        split_string.push_back(substring);
    }

    std::string partial_path = "";
    for (auto s : split_string) {
        if (s != "") {
            partial_path += s + "\\";
            if (!filesystem::path(partial_path).exists()) {
                if (!filesystem::create_directory(filesystem::path(partial_path))) {
                    std::cerr << "Could not create directory: " << partial_path << std::endl;
                } else {
                    std::cout << "Created directory for sensor data: " << partial_path << std::endl;
                }
            }
        }
    }
#else

    std::istringstream istring(m_path);

    std::string substring;
    while (std::getline(istring, substring, '/')) {
        split_string.push_back(substring);
    }

    std::string partial_path = "";
    for (auto s : split_string) {
        if (s != "") {
            partial_path += s + "/";
            if (!filesystem::path(partial_path).exists()) {
                if (!filesystem::create_directory(filesystem::path(partial_path))) {
                    std::cerr << "Could not create directory: " << partial_path << std::endl;
                } else {
                    std::cout << "Created directory for sensor data: " << partial_path << std::endl;
                }
            }
        }
    }
#endif
}

}  // namespace sensor
}  // namespace chrono
