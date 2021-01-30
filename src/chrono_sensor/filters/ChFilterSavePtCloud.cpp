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
#include "chrono_sensor/ChSensor.h"

#include "chrono_thirdparty/filesystem/path.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include <vector>
#include <sstream>

namespace chrono {
namespace sensor {

CH_SENSOR_API ChFilterSavePtCloud::ChFilterSavePtCloud(std::string data_path) : ChFilter("") {
    m_path = data_path;
}

CH_SENSOR_API ChFilterSavePtCloud::~ChFilterSavePtCloud() {}

CH_SENSOR_API void ChFilterSavePtCloud::Apply(std::shared_ptr<ChSensor> pSensor,
                                              std::shared_ptr<SensorBuffer>& bufferInOut) {
    std::shared_ptr<SensorDeviceXYZIBuffer> pXYZI = std::dynamic_pointer_cast<SensorDeviceXYZIBuffer>(bufferInOut);

    if (!pXYZI)
        throw std::runtime_error("This buffer type cannot be saved as as a point cloud");

    std::string filename = m_path + "frame_" + std::to_string(m_frame_number) + ".csv";
    m_frame_number++;

    if (pXYZI) {
        utils::CSV_writer csv_writer(",");

        if (pXYZI->Dual_return){
            float* buf = new float[pXYZI->Beam_return_count * 4 * 2];

            cudaMemcpy(buf, pXYZI->Buffer.get(), pXYZI->Beam_return_count * 2 * sizeof(PixelXYZI), cudaMemcpyDeviceToHost);

            for (unsigned int i = 0; i < pXYZI->Beam_return_count * 2; i++) {
                for (int j = 0; j < 4; j++) {
                    csv_writer << buf[i * 4 + j];
                }
                csv_writer << std::endl;
            }
            csv_writer.write_to_file(filename);
            delete[] buf;
        } else {
            float* buf = new float[pXYZI->Beam_return_count * 4];

            cudaMemcpy(buf, pXYZI->Buffer.get(), pXYZI->Beam_return_count * sizeof(PixelXYZI), cudaMemcpyDeviceToHost);

            for (unsigned int i = 0; i < pXYZI->Beam_return_count; i++) {
                for (int j = 0; j < 4; j++) {
                    csv_writer << buf[i * 4 + j];
                }
                csv_writer << std::endl;
            }
            csv_writer.write_to_file(filename);
            delete[] buf;
        }
    }
}

CH_SENSOR_API void ChFilterSavePtCloud::Initialize(std::shared_ptr<ChSensor> pSensor) {
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
