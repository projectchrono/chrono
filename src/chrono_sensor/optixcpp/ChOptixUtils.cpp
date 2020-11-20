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

#include "chrono_sensor/ChConfigSensor.h"
#include "chrono_sensor/optixcpp/ChOptixUtils.h"
#include <cstring>
#include <fstream>
#include <sstream>

#include <nvrtc.h>

namespace chrono {
namespace sensor {

using namespace optix;

#ifdef USE_CUDA_NVRTC

// if the code was not compiled, we will need to compile with NVRTC
CH_SENSOR_API optix::Program GetRTProgram(optix::Context context, std::string file_name, std::string program_name) {
    // read the CUDA file
    std::string cuda_file = std::string(SENSOR_CUDA_SRC_DIR) + file_name + ".cu";
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
    const nvrtcResult create_result =
        nvrtcCreateProgram(&nvrtc_program, str.c_str(), program_name.c_str(), 0, NULL, NULL);
    if (create_result != NVRTC_SUCCESS) {
        throw std::runtime_error("Could not create nvrtc program. Error: " +
                                 std::string(nvrtcGetErrorString(create_result)));
    }

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
        nvrtcGetProgramLog(nvrtc_program, &nvrt_compilation_log[0]);
    }
    if (compile_result != NVRTC_SUCCESS) {
        throw std::runtime_error("Could not compile nvrtc program. Error: " +
                                 std::string(nvrtcGetErrorString(compile_result)) + "\n Log:\n" + nvrt_compilation_log);
    }

    std::string ptx;
    size_t ptx_size = 0;
    const nvrtcResult size_result = nvrtcGetPTXSize(nvrtc_program, &ptx_size);
    if (size_result != NVRTC_SUCCESS) {
        throw std::runtime_error("Could not get size of ptx from nvrtc program. Error: " +
                                 std::string(nvrtcGetErrorString(size_result)));
    }

    ptx.resize(ptx_size);
    const nvrtcResult ptx_result = nvrtcGetPTX(nvrtc_program, &ptx[0]);
    if (ptx_result != NVRTC_SUCCESS) {
        throw std::runtime_error("Could not get ptx from nvrtc program. Error: " +
                                 std::string(nvrtcGetErrorString(ptx_result)));
    }

    // create the rt program
    optix::Program program = context->createProgramFromPTXString(ptx, program_name);

    // destroy the nvrtc program
    const nvrtcResult destroy_result = nvrtcDestroyProgram(&nvrtc_program);
    if (destroy_result != NVRTC_SUCCESS) {
        throw std::runtime_error("Could not destroy nvrtc program. Error: " +
                                 std::string(nvrtcGetErrorString(destroy_result)));
    }

    return program;
}

#else
CH_SENSOR_API optix::Program GetRTProgram(optix::Context context, std::string file_name, std::string program_name) {
    std::string ptx_file = std::string(PTX_GENERATED_PATH) + ptx_pre + file_name + ptx_suff;
    optix::Program program = context->createProgramFromPTXFile(ptx_file, program_name);
    return program;
}
#endif

ByteImageData LoadImage(std::string filename) {
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

Transform CreateEmptyTransform(optix::Context context) {
    optix::Transform t = context->createTransform();
    t->setMotionRange(0, 1);
    return t;
}

Transform CreateTransform(optix::Context context, ChMatrix33<double> a, ChVector<double> b) {
    optix::Transform t = context->createTransform();
    UpdateTransform(t, a, b);
    return t;
}

Transform CreateTransform(optix::Context context, ChMatrix33<double> a, ChVector<double> b, ChVector<double> s) {
    optix::Transform t = context->createTransform();
    UpdateTransform(t, a, b, s);
    return t;
}

Transform CreateTransformFromEndPoints(optix::Context context, ChVector<> a, ChVector<> b, ChVector<> from) {
    ChVector<> to = (b - a);
    to.Normalize();
    ChVector<> axis = from.Cross(to);
    axis.Normalize();
    from.Normalize();
    double angle = std::acos(from.Dot(to));

    return CreateTransform(context, ChMatrix33<>(angle, axis), (b + a) / 2.0);
}

Transform CreateTransformFromEndPoints(optix::Context context,
                                       ChVector<> a,
                                       ChVector<> b,
                                       ChVector<> from,
                                       ChVector<> s) {
    ChVector<> to = (b - a);
    to.Normalize();
    ChVector<> axis = from.Cross(to);
    axis.Normalize();
    from.Normalize();
    double angle = std::acos(from.Dot(to));

    return CreateTransform(context, ChMatrix33<>(angle, axis), (b + a) / 2.0, s);
}

void UpdateTransform(optix::Transform t, ChMatrix33<double> a, ChVector<double> b) {
    const float t_mat[16] = {(float)a(0), (float)a(1),  (float)a(2), (float)b.x(), (float)a(3), (float)a(4),
                             (float)a(5), (float)b.y(), (float)a(6), (float)a(7),  (float)a(8), (float)b.z(),
                             0.f,         0.f,          0.f,         1.f};
    t->setMatrix(false, t_mat, NULL);
}

void UpdateTransform(optix::Transform t, ChMatrix33<double> a, ChVector<double> b, ChVector<double> s) {
    const float t_mat[16] = {(float)(s.x() * a(0)),
                             (float)(s.x() * a(1)),
                             (float)(s.x() * a(2)),
                             (float)b.x(),
                             (float)(s.y() * a(3)),
                             (float)(s.y() * a(4)),
                             (float)(s.y() * a(5)),
                             (float)b.y(),
                             (float)(s.z() * a(6)),
                             (float)(s.z() * a(7)),
                             (float)(s.z() * a(8)),
                             (float)b.z(),
                             0.f,
                             0.f,
                             0.f,
                             1.f};
    t->setMatrix(false, t_mat, NULL);
}

}  // namespace sensor
}  // namespace chrono
