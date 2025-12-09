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

#include "chrono_sensor/filters/ChFilterRadarSavePC.h"
#include "chrono_sensor/sensors/ChOptixSensor.h"
#include "chrono_sensor/utils/CudaMallocHelper.h"

#include "chrono/input_output/ChWriterCSV.h"

#include "chrono_thirdparty/filesystem/path.h"

#include <vector>
#include <sstream>

#include <cuda_runtime_api.h>

namespace chrono {
namespace sensor {

CH_SENSOR_API ChFilterRadarSavePC::ChFilterRadarSavePC(std::string data_path, std::string name) : ChFilter(name) {
    m_path = data_path;
}

CH_SENSOR_API ChFilterRadarSavePC::~ChFilterRadarSavePC() {}

CH_SENSOR_API void ChFilterRadarSavePC::Apply() {
    std::string filename = m_path + "frame_" + std::to_string(m_frame_number) + ".csv";
    m_frame_number++;
    ChWriterCSV csv_writer(",");
    for (int i = 0; i < m_buffer_in->Beam_return_count; i++) {
        csv_writer << m_buffer_in->Buffer[i].x << m_buffer_in->Buffer[i].y << m_buffer_in->Buffer[i].z
                   << m_buffer_in->Buffer[i].vel_x << m_buffer_in->Buffer[i].vel_y << m_buffer_in->Buffer[i].vel_z
                   << m_buffer_in->Buffer[i].amplitude << m_buffer_in->Buffer[i].objectId << std::endl;
    }
    csv_writer.WriteToFile(filename);
}

CH_SENSOR_API void ChFilterRadarSavePC::Initialize(std::shared_ptr<ChSensor> pSensor,
                                                   std::shared_ptr<SensorBuffer>& bufferInOut) {
    if (!bufferInOut)
        InvalidFilterGraphNullBuffer(pSensor);

    m_buffer_in = std::dynamic_pointer_cast<SensorDeviceRadarXYZBuffer>(bufferInOut);
    if (!m_buffer_in)
        InvalidFilterGraphBufferTypeMismatch(pSensor);

    if (auto pOpx = std::dynamic_pointer_cast<ChOptixSensor>(pSensor)) {
        m_cuda_stream = pOpx->GetCudaStream();
    } else {
        InvalidFilterGraphSensorTypeMismatch(pSensor);
    }

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
