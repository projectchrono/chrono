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
 #define NOMINMAX
#endif

#include <optix.h>
#include <optix_world.h>
#include <optixu/optixu_math_namespace.h>

#include "chrono/assets/ChVisualMaterial.h"
#include "chrono/assets/ChBoxShape.h"
#include "chrono/core/ChMatrix33.h"

#include "chrono_thirdparty/stb/stb_image.h"

#include <string>

#include "chrono_sensor/ChApiSensor.h"

namespace chrono {
namespace sensor {

/// @addtogroup sensor_optix
/// @{

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
/// @param context optix context
/// @param file_name the .cu file where the RT program is implemented
/// @param program_name RT program
CH_SENSOR_API optix::Program GetRTProgram(optix::Context context, std::string file_name, std::string program_name);

/// loads image to struct ByteImageData, returns an empty struct with 0 values if loading failed
/// @param filename
ByteImageData LoadImage(std::string filename);

/// creates an empty optix transform::node
/// @param context the optix context
optix::Transform CreateEmptyTransform(optix::Context contex);

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

/// creatse an optix::transform node based on end points
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

/// prefix for ptx file
const std::string ptx_pre = "ChronoEngine_sensor_generated_";
/// suffix for ptx file
const std::string ptx_suff = ".cu.ptx";

/// @} sensor_optix

}  // namespace sensor
}  // namespace chrono

#endif
