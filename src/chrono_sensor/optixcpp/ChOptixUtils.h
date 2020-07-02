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

#include <optix.h>
#include <optix_world.h>
#include <optixu/optixu_math_namespace.h>

#include "chrono/assets/ChVisualMaterial.h"
#include "chrono/assets/ChBoxShape.h"
#include "chrono/core/ChMatrix33.h"

#include "chrono_thirdparty/nothings/stb_image.h"

#include <string>

#include "chrono_sensor/ChApiSensor.h"

namespace chrono {
namespace sensor {

struct ProgramString {
    std::string file_name;
    std::string program_name;
};

struct ByteImageData {
    int w;
    int h;
    int c;
    std::vector<unsigned char> data;
};

CH_SENSOR_API optix::Program GetRTProgram(optix::Context context, std::string file_name, std::string program_name);

ByteImageData LoadImage(std::string filename);

optix::Transform CreateEmptyTransform(optix::Context contex);
optix::Transform CreateTransform(optix::Context context, ChMatrix33<double> a, ChVector<double> b);
optix::Transform CreateTransform(optix::Context context, ChMatrix33<double> a, ChVector<double> b, ChVector<double> s);
optix::Transform CreateTransformFromEndPoints(optix::Context context, ChVector<> a, ChVector<> b, ChVector<> from);
optix::Transform CreateTransformFromEndPoints(optix::Context context,
                                              ChVector<> a,
                                              ChVector<> b,
                                              ChVector<> from,
                                              ChVector<double> s);
void UpdateTransform(optix::Transform t, ChMatrix33<double> a, ChVector<double> b);
void UpdateTransform(optix::Transform t, ChMatrix33<double> a, ChVector<double> b, ChVector<double> s);

const std::string ptx_pre = "ChronoEngine_sensor_generated_";
const std::string ptx_suff = ".cu.ptx";

}  // namespace sensor
}  // namespace chrono

#endif
