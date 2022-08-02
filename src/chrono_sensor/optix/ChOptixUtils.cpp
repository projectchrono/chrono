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
// utility functions used for optix convenience
//
// =============================================================================

#include "chrono_sensor/optix/ChOptixUtils.h"
#include "chrono_sensor/ChConfigSensor.h"
#include <cstring>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <chrono>

#include <optix_stubs.h>
// #include <optix_function_table_definition.h>

#ifdef USE_CUDA_NVRTC
    #include <nvrtc.h>
#endif

namespace chrono {
namespace sensor {

class SensorConfig {
  public:
    static std::string ptx_pre;            //= "ChronoEngine_sensor_generated_";
    static std::string ptx_suff;           // = ".cu.ptx";
    static std::string SENSOR_SHADER_DIR;  // = std::string(CMAKE_SHADER_OUTPUT_PATH);
};

std::string SensorConfig::ptx_pre = "ChronoEngine_sensor_generated_";
std::string SensorConfig::ptx_suff = ".cu.ptx";
std::string SensorConfig::SENSOR_SHADER_DIR = std::string(CMAKE_SHADER_OUTPUT_PATH);

void SetSensorShaderDir(const std::string& path){
    SensorConfig::SENSOR_SHADER_DIR = path;
}

void GetShaderFromFile(OptixDeviceContext context,
                       OptixModule& module,
                       const std::string& file_name,
                       OptixModuleCompileOptions& module_compile_options,
                       OptixPipelineCompileOptions& pipeline_compile_options) {
#ifdef USE_CUDA_NVRTC
    // std::chrono::high_resolution_clock::time_point start_compile = std::chrono::high_resolution_clock::now();

    std::string cuda_file = SensorConfig::SENSOR_SHADER_DIR + file_name + ".cu";
    std::string str;
    std::ifstream f(cuda_file.c_str());
    if (f.good()) {
        std::stringstream source_buffer;
        source_buffer << f.rdbuf();
        str = source_buffer.str();
    } else {
        throw std::runtime_error("CUDA file not found for NVRTC: " + cuda_file);
    }

    // compile CUDA code with NVRTC
    nvrtcProgram nvrtc_program;
    NVRTC_ERROR_CHECK(nvrtcCreateProgram(&nvrtc_program, str.c_str(), str.c_str(), 0, NULL, NULL));

    // complete list of flags to be used for NVRTC
    std::vector<const char*> nvrtc_compiler_flag_list;

    // include directories passed from CMake
    std::vector<std::string> scoping_dir_list;  // to keep the flags from going out of scope
    const char* nvrtc_include_dirs[] = {CUDA_NVRTC_INCLUDE_LIST};
    int num_dirs = sizeof(nvrtc_include_dirs) / sizeof(nvrtc_include_dirs[0]);
    for (int i = 0; i < num_dirs - 1; i++) {
        scoping_dir_list.push_back(std::string("-I") + nvrtc_include_dirs[i]);
        nvrtc_compiler_flag_list.push_back(scoping_dir_list[i].c_str());
    }

    // compile flags passed from CMake
    const char* nvrtc_flags[] = {CUDA_NVRTC_FLAG_LIST};
    int num_flags = sizeof(nvrtc_flags) / sizeof(nvrtc_flags[0]);
    for (int i = 0; i < num_flags - 1; i++) {
        nvrtc_compiler_flag_list.push_back(nvrtc_flags[i]);
    }

    // runtime compile CU to PTX with NVRTC
    const nvrtcResult compile_result =
        nvrtcCompileProgram(nvrtc_program, (int)nvrtc_compiler_flag_list.size(), nvrtc_compiler_flag_list.data());

    std::string nvrt_compilation_log;
    size_t log_length;
    nvrtcGetProgramLogSize(nvrtc_program, &log_length);
    nvrt_compilation_log.resize(log_length);
    if (log_length > 0) {
        NVRTC_ERROR_CHECK(nvrtcGetProgramLog(nvrtc_program, &nvrt_compilation_log[0]));
    }
    if (compile_result != NVRTC_SUCCESS) {
        throw std::runtime_error(std::string("Error: ").append(__FILE__) + " at line " + std::to_string(__LINE__) +
                                 "\n" + nvrt_compilation_log);
    }

    std::string ptx;
    size_t ptx_size = 0;
    NVRTC_ERROR_CHECK(nvrtcGetPTXSize(nvrtc_program, &ptx_size));
    ptx.resize(ptx_size);
    NVRTC_ERROR_CHECK(nvrtcGetPTX(nvrtc_program, &ptx[0]));

    // std::chrono::high_resolution_clock::time_point end_compile = std::chrono::high_resolution_clock::now();

    // std::cout << "Rebuilt root acceleration structure, addr = " << m_root << std::endl;
    // std::chrono::duration<double> wall_time = std::chrono::duration_cast<std::chrono::duration<double>>(end_compile - start_compile);
    // std::cout << "NVRTC Compilation: " << file_name << " | " << wall_time.count() << std::endl;
    // wall_time = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);

#else
    std::string ptx_file = SensorConfig::SENSOR_SHADER_DIR + SensorConfig::ptx_pre + file_name + SensorConfig::ptx_suff;
    std::cout <<"Loading PTX: "<<ptx_file<<std::endl;
    std::string ptx;
    std::ifstream f(ptx_file.c_str());
    if (f.good()) {
        std::stringstream source_buffer;
        source_buffer << f.rdbuf();
        ptx = source_buffer.str();
    } else {
        throw std::runtime_error("PTX file not found: " + ptx_file);
    }

#endif

    char log[2048];
    size_t sizeof_log = sizeof(log);
    OPTIX_ERROR_CHECK(optixModuleCreateFromPTX(context, &module_compile_options, &pipeline_compile_options, ptx.c_str(),
                                               ptx.size(), log, &sizeof_log, &module));
}

void optix_log_callback(unsigned int level, const char* tag, const char* message, void*) {
    std::cerr << "[" << std::setw(2) << level << "][" << std::setw(12) << tag << "]: " << message << "\n";
}

ByteImageData LoadByteImage(const std::string& filename) {
    ByteImageData img_data;
    int w;
    int h;
    int c;
    unsigned char* data = stbi_load(filename.c_str(), &w, &h, &c, 0);

    if (!data) {
        img_data.w = 0;
        img_data.h = 0;
        img_data.c = 0;
        return img_data;  // return if loading failed
    }

    img_data.data = std::vector<unsigned char>(w * h * c);
    img_data.w = w;
    img_data.h = h;
    img_data.c = c;
    memcpy(img_data.data.data(), data, sizeof(unsigned char) * img_data.data.size());

    stbi_image_free(data);

    return img_data;
}

}  // namespace sensor
}  // namespace chrono
