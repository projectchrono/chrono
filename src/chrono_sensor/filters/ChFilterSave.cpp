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

#include "chrono_sensor/filters/ChFilterSave.h"
#include "chrono_sensor/ChSensor.h"

#include "chrono_thirdparty/stb/stb_image_write.h"
#include "chrono_thirdparty/filesystem/path.h"

#include <vector>
#include <sstream>

namespace chrono {
namespace sensor {

CH_SENSOR_API ChFilterSave::ChFilterSave(std::string data_path) : ChFilter("") {
    m_path = data_path;
}

CH_SENSOR_API ChFilterSave::~ChFilterSave() {}

CH_SENSOR_API void ChFilterSave::Apply(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut) {
    std::shared_ptr<SensorOptixBuffer> pOptix = std::dynamic_pointer_cast<SensorOptixBuffer>(bufferInOut);
    std::shared_ptr<SensorDeviceR8Buffer> pR8 = std::dynamic_pointer_cast<SensorDeviceR8Buffer>(bufferInOut);
    std::shared_ptr<SensorDeviceRGBA8Buffer> pRGBA8 = std::dynamic_pointer_cast<SensorDeviceRGBA8Buffer>(bufferInOut);

    if (!pOptix && !pR8 && !pRGBA8)
        throw std::runtime_error("This buffer type cannot be saved as png");

    std::string filename = m_path + "frame_" + std::to_string(m_frame_number) + ".png";
    m_frame_number++;

    // openGL buffers are bottom to top...so flip when writing png.
    stbi_flip_vertically_on_write(1);

    if (pOptix) {
        optix::Buffer buffer = pOptix->Buffer;

        // Query buffer information
        RTsize buffer_width_rts, buffer_height_rts;
        buffer->getSize(buffer_width_rts, buffer_height_rts);
        uint32_t width = static_cast<int>(buffer_width_rts);
        uint32_t height = static_cast<int>(buffer_height_rts);
        RTformat buffer_format = buffer->getFormat();

        int comp = 1;
        if (buffer_format == RT_FORMAT_UNSIGNED_BYTE4) {
            comp = 4;
        }
        void* imageData = buffer->map(0, RT_BUFFER_MAP_READ);

        // int stbi_write_png(char const* filename, int w, int h, int comp, const void* data, int stride_in_bytes);
        if (!stbi_write_png(filename.c_str(), width, height, comp, imageData, comp * width)) {
            std::cerr << "Failed to write image: " << filename << "\n";
        }

        buffer->unmap();
    } else if (pR8) {
        char* buf = new char[pR8->Width * pR8->Height];
        cudaMemcpy(buf, pR8->Buffer.get(), pR8->Width * pR8->Height, cudaMemcpyDeviceToHost);

        // write a grayscale png
        if (!stbi_write_png(filename.c_str(), pR8->Width, pR8->Height, 1, buf, pR8->Width)) {
            std::cerr << "Failed to write image: " << filename << "\n";
        }

        delete buf;
    } else if (pRGBA8) {
        char* buf = new char[pRGBA8->Width * pRGBA8->Height * 4];
        cudaMemcpy(buf, pRGBA8->Buffer.get(), pRGBA8->Width * pRGBA8->Height * 4, cudaMemcpyDeviceToHost);

        // write a grayscale png
        if (!stbi_write_png(filename.c_str(), pRGBA8->Width, pRGBA8->Height, 4, buf, 4 * pRGBA8->Width)) {
            std::cerr << "Failed to write image: " << filename << "\n";
        }

        delete buf;
    }
}

CH_SENSOR_API void ChFilterSave::Initialize(std::shared_ptr<ChSensor> pSensor) {
    std::vector<std::string> split_string;

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

    if (m_path.back() != '/' && m_path != "")
		m_path += '/';

}

}  // namespace sensor
}  // namespace chrono
