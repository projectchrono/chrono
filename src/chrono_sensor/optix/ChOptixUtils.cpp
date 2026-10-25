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

#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <chrono>

#include <optix_stubs.h>
// #include <optix_function_table_definition.h>

#include "chrono_sensor/ChConfigSensor.h"
#include "chrono_sensor/optix/ChOptixUtils.h"
#include "chrono_sensor/utils/ChSensorUtils.h"

#ifdef USE_CUDA_NVRTC
    #include <cuda.h>  // for CUDA_VERSION
    #include <nvrtc.h>

// Whether to ASK NVRTC for an OptiX-IR module rather than PTX. This selects what is tried first;
// GetShaderFromFile below falls back to PTX at run time if the driver rejects the IR.
//
// OptiX-IR is preferred where it works: on Blackwell / RTX 50-series GPUs (sm_120) the driver's
// OptiX PTX front end has been seen to abort inside optixModuleCreate, while OptiX-IR compiles
// cleanly.
//
// NVRTC gained --optix-ir and nvrtcGetOptiXIR in CUDA 12.0; CUDA 11.8 has neither, and Chrono
// declares no minimum CUDA version, so below that floor only PTX is possible. That is exactly
// what every build did before OptiX-IR was introduced, so nothing regresses on older toolkits.
//
// This cannot be more than a first choice. NVRTC (the toolkit) produces the IR but libnvoptix
// (the display driver) consumes it, and OptiX refuses IR newer than its own front end, so a
// toolkit ahead of the driver fails at run time however new both are: CUDA 13.3 emits NVVM IR
// 115 and a 580-series driver accepts at most 98. The two are versioned independently and a
// container makes them independently installable, so the working combination has to be found by
// trying rather than asserted here.
//
// OptiX needs no companion check. OptiX-IR input landed in OptiX 7.5, and Chrono already calls
// optixModuleCreate rather than the older optixModuleCreateFromPTX, so it requires 7.7 or newer
// regardless. optixModuleCreate detects the buffer format itself and is passed an explicit size,
// so the call below is identical for either input.
    #if CUDA_VERSION >= 12000
        #define CH_OPTIX_EMIT_OPTIXIR 1
    #else
        #define CH_OPTIX_EMIT_OPTIXIR 0
    #endif
#endif

namespace chrono {
namespace sensor {

// Shader directory, resolved on first use: the shaders of the installation (relative to the Chrono::Sensor library)
// if Chrono::Sensor is installed, and otherwise those of the build tree.
static std::string& ShaderDir() {
    static std::string shader_dir = LocateSensorDirectory(CHRONO_SENSOR_SHADER_DIR_REL, CHRONO_SENSOR_SHADER_DIR);
    return shader_dir;
}

void SetSensorShaderDir(const std::string& path) {
    ShaderDir() = path;
}

const std::string& GetSensorShaderDir() {
    return ShaderDir();
}

#ifdef USE_CUDA_NVRTC

// Value of an environment variable, or an empty string if it is not set.
static std::string GetEnvironmentValue(const char* name) {
    #ifdef _MSC_VER
    char* value = nullptr;
    size_t length = 0;
    std::string result;
    if (_dupenv_s(&value, &length, name) == 0 && value != nullptr)
        result = value;
    free(value);
    return result;
    #else
    const char* value = std::getenv(name);
    return value ? std::string(value) : std::string();
    #endif
}

// Include directories for NVRTC compilation of the RT programs.
// The OptiX SDK and CUDA toolkit headers are taken from the locations used to build Chrono if these exist, and
// otherwise from the OptiX_INSTALL_DIR and CUDA_PATH (or CUDA_HOME) environment variables. The Chrono headers are those
// of the installation if Chrono::Sensor is installed, and otherwise those of the source tree.
static std::vector<std::string> GetNvrtcIncludeDirs() {
    auto is_dir = [](const std::string& dir) {
        std::error_code ec;
        return !dir.empty() && std::filesystem::is_directory(dir, ec);
    };

    std::vector<std::string> dirs;

    std::string optix_dir = CUDA_NVRTC_OPTIX_INCLUDE;
    if (!is_dir(optix_dir)) {
        const std::string optix_root = GetEnvironmentValue("OptiX_INSTALL_DIR");
        if (!optix_root.empty() && is_dir(optix_root + "/include"))
            optix_dir = optix_root + "/include";
    }
    dirs.push_back(optix_dir);

    const char* cuda_dirs[] = {CUDA_NVRTC_CUDA_INCLUDE_LIST};
    const int num_cuda_dirs = sizeof(cuda_dirs) / sizeof(cuda_dirs[0]) - 1;
    std::vector<std::string> found_cuda_dirs;
    for (int i = 0; i < num_cuda_dirs; i++) {
        if (is_dir(cuda_dirs[i]))
            found_cuda_dirs.push_back(cuda_dirs[i]);
    }
    if (found_cuda_dirs.empty()) {
        std::string cuda_root = GetEnvironmentValue("CUDA_PATH");
        if (cuda_root.empty())
            cuda_root = GetEnvironmentValue("CUDA_HOME");
        if (!cuda_root.empty() && is_dir(cuda_root + "/include")) {
            found_cuda_dirs.push_back(cuda_root + "/include");
            // CUDA 13 places the CCCL/libcu++ headers in include/cccl (see the Chrono::Sensor CMake script)
            if (is_dir(cuda_root + "/include/cccl"))
                found_cuda_dirs.push_back(cuda_root + "/include/cccl");
        } else {
            // Keep the build-time locations, so that a failed compilation reports where headers were expected
            found_cuda_dirs.assign(cuda_dirs, cuda_dirs + num_cuda_dirs);
        }
    }
    dirs.insert(dirs.end(), found_cuda_dirs.begin(), found_cuda_dirs.end());

    dirs.push_back(LocateSensorDirectory(CHRONO_SENSOR_INCLUDE_DIR_REL, CUDA_NVRTC_CHRONO_INCLUDE, "chrono_sensor"));

    return dirs;
}

#endif

void GetShaderFromFile(OptixDeviceContext context,
                       OptixModule& module,
                       const std::string& file_name,
                       OptixModuleCompileOptions& module_compile_options,
                       OptixPipelineCompileOptions& pipeline_compile_options) {
    
#ifdef USE_CUDA_NVRTC
    std::string cuda_file = GetSensorShaderDir() + "/" + file_name + ".cu";
    std::string str;
    std::ifstream f(cuda_file);
    if (f.good()) {
        std::stringstream source_buffer;
        source_buffer << f.rdbuf();
        str = source_buffer.str();
    } else {
        throw std::runtime_error("CUDA file not found for NVRTC: " + cuda_file);
    }

    // Compile the shader with NVRTC, emitting either OptiX-IR or PTX.
    //
    // Which of the two the driver accepts is not decidable at compile time. The IR is produced
    // by NVRTC, which comes from the CUDA toolkit, but it is consumed by libnvoptix, which comes
    // from the installed display driver, and OptiX refuses IR newer than its own front end. A
    // CUDA 13.3 NVRTC emits NVVM IR 115 while a 580-series driver tops out at 98, so every
    // optixModuleCreate fails with OPTIX_ERROR_INVALID_INPUT and
    //   "minor NvvmIRVersion (115) newer than tool (should be 98)".
    // CUDA_VERSION describes the producer and says nothing about the consumer, so the choice is
    // made by trying rather than by testing it.
    auto compile_shader = [&](bool emit_optixir) {
        nvrtcProgram nvrtc_program;
        NVRTC_ERROR_CHECK(nvrtcCreateProgram(&nvrtc_program, str.c_str(), cuda_file.c_str(), 0, NULL, NULL));

        // complete list of flags to be used for NVRTC
        std::vector<const char*> nvrtc_compiler_flag_list;

        // include directories, resolved at run time from those passed from CMake
        std::vector<std::string> scoping_dir_list;  // to keep the flags from going out of scope
        for (const std::string& dir : GetNvrtcIncludeDirs()) {
            scoping_dir_list.push_back("-I" + dir);
        }
        for (const std::string& include_dir : scoping_dir_list) {
            nvrtc_compiler_flag_list.push_back(include_dir.c_str());
        }

        // compile flags passed from CMake
        const char* nvrtc_flags[] = {CUDA_NVRTC_FLAG_LIST};
        int num_flags = sizeof(nvrtc_flags) / sizeof(nvrtc_flags[0]);
        for (int i = 0; i < num_flags - 1; i++) {
            nvrtc_compiler_flag_list.push_back(nvrtc_flags[i]);
        }

        if (emit_optixir) {
            nvrtc_compiler_flag_list.push_back("--optix-ir");
        }

        const nvrtcResult compile_result =
            nvrtcCompileProgram(nvrtc_program, (int)nvrtc_compiler_flag_list.size(), nvrtc_compiler_flag_list.data());

        std::string nvrt_compilation_log;
        size_t log_length;
        nvrtcGetProgramLogSize(nvrtc_program, &log_length);
        nvrt_compilation_log.resize(log_length);
        if (log_length > 0) {
            NVRTC_ERROR_CHECK(nvrtcGetProgramLog(nvrtc_program, &nvrt_compilation_log[0]));
        }
        // The log size includes the terminating null character, which would cut off anything appended to the log
        while (!nvrt_compilation_log.empty() && nvrt_compilation_log.back() == '\0')
            nvrt_compilation_log.pop_back();
        if (compile_result != NVRTC_SUCCESS) {
            std::string include_dirs;
            for (const std::string& flag : scoping_dir_list)
                include_dirs += "\n  " + flag.substr(2);
            throw std::runtime_error(std::string("Error: ").append(__FILE__) + " at line " + std::to_string(__LINE__) + "\n" + nvrt_compilation_log +
                                     "\nNVRTC include directories:" + include_dirs + "\nIf OptiX or CUDA headers are not found, set OptiX_INSTALL_DIR or CUDA_PATH.");
        }

        // Retrieve the module. OptiX-IR is binary and can contain embedded NULs, which is safe
        // here because optixModuleCreate is passed the size explicitly rather than relying on the
        // terminator. The PTX branch is byte-for-byte the pre-existing behavior.
        std::string shader;
        size_t shader_size = 0;
        if (emit_optixir) {
            NVRTC_ERROR_CHECK(nvrtcGetOptiXIRSize(nvrtc_program, &shader_size));
            shader.resize(shader_size);
            NVRTC_ERROR_CHECK(nvrtcGetOptiXIR(nvrtc_program, &shader[0]));
        } else {
            NVRTC_ERROR_CHECK(nvrtcGetPTXSize(nvrtc_program, &shader_size));
            shader.resize(shader_size);
            NVRTC_ERROR_CHECK(nvrtcGetPTX(nvrtc_program, &shader[0]));
        }
        return shader;
    };

    char log[2048];
    size_t sizeof_log = sizeof(log);

    // Decided once per process rather than per module: the first shader that has to fall back
    // records it here and every later shader skips the attempt that is already known to fail.
    // A benign race at worst costs another process-wide-consistent retry, so no lock is taken.
    static bool emit_optixir = (CH_OPTIX_EMIT_OPTIXIR != 0);

    if (emit_optixir) {
        const std::string optixir = compile_shader(true);
        log[0] = '\0';
        sizeof_log = sizeof(log);
        const OptixResult result = optixModuleCreate(context, &module_compile_options, &pipeline_compile_options,
                                                     optixir.c_str(), optixir.size(), log, &sizeof_log, &module);
        if (result == OPTIX_SUCCESS)
            return;

        emit_optixir = false;
        std::cerr << "Chrono::Sensor: this driver's OptiX rejected the OptiX-IR produced by the installed CUDA "
                     "toolkit (" << optixGetErrorName(result) << "); falling back to PTX for all shaders.\n"
                  << log << std::endl;
    }

    const std::string ptx = compile_shader(false);
    sizeof_log = sizeof(log);
    OPTIX_ERROR_CHECK(optixModuleCreate(context, &module_compile_options, &pipeline_compile_options, ptx.c_str(),
                                        ptx.size(), log, &sizeof_log, &module));

#else
    std::string ptx_file = GetSensorShaderDir() + "/" + file_name + ".ptx";
    std::string ptx;
    std::ifstream f(ptx_file);
    if (f.good()) {
        std::stringstream source_buffer;
        source_buffer << f.rdbuf();
        ptx = source_buffer.str();
    } else {
        throw std::runtime_error("PTX file not found: " + ptx_file);
    }

    char log[2048];
    size_t sizeof_log = sizeof(log);
    OPTIX_ERROR_CHECK(optixModuleCreate(context, &module_compile_options, &pipeline_compile_options, ptx.c_str(),
                                        ptx.size(), log, &sizeof_log, &module));
#endif  // USE_CUDA_NVRTC
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
