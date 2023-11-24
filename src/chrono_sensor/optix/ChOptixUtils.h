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

#ifndef CHOPTIXUTILS_H
#define CHOPTIXUTILS_H

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#endif

#include <optix.h>
#include <cuda_runtime_api.h>
#include <nvrtc.h>

#include "chrono/assets/ChVisualMaterial.h"
#include "chrono/assets/ChVisualShapeBox.h"
#include "chrono/core/ChMatrix33.h"

#include "chrono_thirdparty/stb/stb_image.h"

#include <string>

#include "chrono_sensor/ChApiSensor.h"

namespace chrono {
namespace sensor {

/// @addtogroup sensor_optix
/// @{


/// Checks the output of an optix call for any error, will throw a runtime error if not success
#define OPTIX_ERROR_CHECK(result) { \
    if (result != OPTIX_SUCCESS) { \
        std::string error_name = std::string(optixGetErrorName(result)); \
        std::string error_string = std::string(optixGetErrorString(result)); \
        std::string file = std::string(__FILE__); \
        std::string line = std::to_string(__LINE__); \
        throw std::runtime_error(error_name + ": " + error_string + " at " + file + ":" + line); \
    } \
} \

/// Checks the output of a cuda call for any error, will throw a runtime error if not success
#define CUDA_ERROR_CHECK(result) { \
    if (result != cudaSuccess) { \
        std::string error_name = std::string(cudaGetErrorName(result));\
        std::string error_string = std::string(cudaGetErrorString(result));\
        std::string file = std::string(__FILE__);\
        std::string line = std::to_string(__LINE__);\
        throw std::runtime_error(error_name + ": " + error_string + " at " + file + ":" + line);\
    }\
}

#ifdef USE_CUDA_NVRTC
/// Checks the output of a cuda call for any error, will throw a runtime error if not success
#define NVRTC_ERROR_CHECK(result) { \
    if (result != NVRTC_SUCCESS) { \
        std::string error_name = "NVRTC ERROR"; \
        std::string error_string = std::string(nvrtcGetErrorString(result)); \
        std::string file = std::string(__FILE__); \
        std::string line = std::to_string(__LINE__); \
        throw std::runtime_error(error_name + ": " + error_string + " at " + file + ":" + line); \
    } \
}
#endif

/// holds string values for ptx file and ray generation program
struct ProgramString {
    std::string file_name;
    std::string program_name;
};

/// stores image data
struct ByteImageData {
    /// image width
    int w;
    /// image height
    int h;
    ///
    int c;
    /// image pixel valules
    std::vector<unsigned char> data;
};

/// launches ray generation program
/// @param context optix device context
/// @param module optix module that will be created
/// @param file_name the file where the shader program is implemented
/// @param module_compile_options compile options for the module
/// @param pipeline_compile_options compile options for the pipeline
CH_SENSOR_API void GetShaderFromFile(OptixDeviceContext context,
                                     OptixModule& module,
                                     const std::string& file_name,
                                     OptixModuleCompileOptions& module_compile_options,
                                     OptixPipelineCompileOptions& pipeline_compile_options);

CH_SENSOR_API void optix_log_callback(unsigned int level, const char* tag, const char* message, void*);

// #ifdef USE_CUDA_NVRTC
// /// launches ray generation program
// /// @param context optix device context
// /// @param module optix module that will be created
// /// @param file_name the file where the shader program is implemented
// /// @param module_compile_options compile options for the module
// /// @param pipeline_compile_options compile options for the pipeline
// CH_SENSOR_API void GetShaderFromPtx(OptixDeviceContext context,
//                                     OptixModule& module,
//                                     std::string file_name,
//                                     OptixModuleCompileOptions& module_compile_options,
//                                     OptixPipelineCompileOptions& pipeline_compile_options);
// #endif

/// loads image to struct ByteImageData, returns an empty struct with 0 values if loading failed
/// @param filename
CH_SENSOR_API ByteImageData LoadByteImage(const std::string& filename);

/*
/// creates an empty optix transform::node
/// @param context the optix context
optix::Transform CreateEmptyTransform(optix::Context context);

/// creates an opti::transform node
/// @param context optix context
/// @param a projection matrix
/// @param b
optix::Transform CreateTransform(optix::Context context, ChMatrix33<double> a, ChVector<double> b);

/// creates an optix::transform node
/// @param context optix context
/// @param a  projection matrix
/// @param b
/// @param s
/// @return an optix::transform
optix::Transform CreateTransform(optix::Context context, ChMatrix33<double> a, ChVector<double> b, ChVector<double> s);

/// creatse an optix::transform node based on end points
/// @param context optix context
/// @param a projection matrix
/// @param b
/// @param from
/// @return an optix::transform
optix::Transform CreateTransformFromEndPoints(optix::Context context, ChVector<> a, ChVector<> b, ChVector<> from);

/// creates an optix::transform node based on end points
/// @param context optix context
/// @param a projection matrix
/// @param b
/// @param from
/// @param s
/// @return an optix::transform
optix::Transform CreateTransformFromEndPoints(optix::Context context,
                                              ChVector<> a,
                                              ChVector<> b,
                                              ChVector<> from,
                                              ChVector<double> s);

/// updates the projection matrix in the optix::transform object
/// @param t optix transform object
/// @param a projection matrix
/// @param b
void UpdateTransform(optix::Transform t, ChMatrix33<double> a, ChVector<double> b);

/// updates the projection matrix in the optix::transform object
/// @param t optix tranform object
/// @param a projection matrix
/// @param b
/// @param s
void UpdateTransform(optix::Transform t, ChMatrix33<double> a, ChVector<double> b, ChVector<double> s);
*/

CH_SENSOR_API void SetSensorShaderDir(const std::string& path);

/*
/// prefix for ptx file
const std::string ptx_pre = "ChronoEngine_sensor_generated_";
/// suffix for ptx file
const std::string ptx_suff = ".cu.ptx";

extern std::string SENSOR_SHADER_DIR;// = "test"; //std::string(CMAKE_SHADER_OUTPUT_PATH);
*/

/// @} sensor_optix

}  // namespace sensor
}  // namespace chrono

#endif
