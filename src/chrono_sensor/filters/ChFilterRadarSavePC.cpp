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
#include "chrono_sensor/sensors/ChRadarSensor.h"

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

ChFilterRadarSavePC::ChFilterRadarSavePC(std::string data_path, std::string name) : ChFilter(name), m_path(data_path) {}
ChFilterRadarSavePC::~ChFilterRadarSavePC() {}

void ChFilterRadarSavePC::Apply() {
    std::string filename = m_path + "frame_" + std::to_string(m_frame_number) + ".csv";
    ++m_frame_number;
    ChWriterCSV csv_writer(",");
    for (int i = 0; i < m_buffer_in->Beam_return_count; i++) {
        csv_writer << m_buffer_in->Buffer[i].x << m_buffer_in->Buffer[i].y << m_buffer_in->Buffer[i].z
                   << m_buffer_in->Buffer[i].vel_x << m_buffer_in->Buffer[i].vel_y << m_buffer_in->Buffer[i].vel_z
                   << m_buffer_in->Buffer[i].amplitude << m_buffer_in->Buffer[i].objectId << std::endl;
    }
    csv_writer.WriteToFile(filename);
}

void ChFilterRadarSavePC::Initialize(std::shared_ptr<ChSensor> pSensor,
                                     std::shared_ptr<SensorBuffer>& bufferInOut) {
    if (!bufferInOut)
        InvalidFilterGraphNullBuffer(pSensor);

    m_buffer_in = std::dynamic_pointer_cast<SensorDeviceRadarXYZBuffer>(bufferInOut);
    if (!m_buffer_in)
        InvalidFilterGraphBufferTypeMismatch(pSensor);

    if (auto pRadar = std::dynamic_pointer_cast<ChRadarSensor>(pSensor)) {
#ifdef CHRONO_HAS_OPTIX
        m_cuda_stream = pRadar->GetCudaStream();
#endif
    } else {
        InvalidFilterGraphSensorTypeMismatch(pSensor);
    }

    EnsureDirectoryTree(m_path);
}

}  // namespace sensor
}  // namespace chrono
