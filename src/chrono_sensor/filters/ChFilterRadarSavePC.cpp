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
#include "chrono_sensor/ChSensor.h"

#include "chrono_thirdparty/filesystem/path.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include <vector>
#include <sstream>

namespace chrono {
namespace sensor {

CH_SENSOR_API ChFilterRadarSavePC::ChFilterRadarSavePC(std::string data_path) : ChFilter("") {
    m_path = data_path;
}

CH_SENSOR_API ChFilterRadarSavePC::~ChFilterRadarSavePC() {}

CH_SENSOR_API void ChFilterRadarSavePC::Apply(std::shared_ptr<ChSensor> pSensor,
                                              std::shared_ptr<SensorBuffer>& bufferInOut) {
    std::shared_ptr<SensorDeviceRangeRcsBuffer> pRR = std::dynamic_pointer_cast<SensorDeviceRangeRcsBuffer>(bufferInOut);

    if (!pRR)
        throw std::runtime_error("This buffer type cannot be saved as as a point cloud");

    std::string filename = m_path + "frame_" + std::to_string(m_frame_number) + ".csv";
    m_frame_number++;

    if (pRR) {
        utils::CSV_writer csv_writer(",");
        float* buf = new float[pRR->Beam_return_count * 2];

        cudaMemcpy(buf, pRR->Buffer.get(), pRR->Beam_return_count * sizeof(PixelRangeRcs), cudaMemcpyDeviceToHost);

        for (unsigned int i = 0; i < pRR->Beam_return_count; i++) {
            for (int j = 0; j < 2; j++) {
                csv_writer << buf[i * 2 + j];
            }
            csv_writer << std::endl;
        }
        csv_writer.write_to_file(filename);
        delete[] buf;
    }
}

CH_SENSOR_API void ChFilterRadarSavePC::Initialize(std::shared_ptr<ChSensor> pSensor) {
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
