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
// Author: Asher Elmquist
// =============================================================================
//
// Macro defines for exporting DLL
// =============================================================================

#ifndef CH_API_SENSOR_H
#define CH_API_SENSOR_H

#include "chrono/ChVersion.h"
#include "chrono/core/ChPlatform.h"

// When compiling this library, remember to define CH_API_COMPILE_SENSOR
// (so that the symbols with 'CH_SENSOR_API' in front of them will be
// marked as exported). Otherwise, just do not define it if you
// link the library to your code, and the symbols will be imported.

#if defined(CH_API_COMPILE_SENSOR)
    #define CH_SENSOR_API ChApiEXPORT
#else
    #define CH_SENSOR_API ChApiIMPORT
#endif

/**
    @defgroup sensor SENSOR module
    @brief Modeling and simulation of sensors

    This module provides support for modeling and simulating sensors for autonomous behavior.

    For additional information, see:
    - the [installation guide](@ref module_sensor_installation)
    - the [tutorials](@ref tutorial_table_of_content_chrono_sensor)

    @{
        @defgroup sensor_sensors Sensors
        @defgroup sensor_buffers Sensor Data Buffers
        @defgroup sensor_filters Sensor Filters
        @defgroup sensor_cuda CUDA Wrapper Functions
        @defgroup sensor_optix OptiX-Based Code
        @defgroup sensor_tensorrt TensorRT-Based Code
        @defgroup sensor_scene Scene
        @defgroup sensor_utils Utilities
    @}
*/

namespace chrono {

/// @addtogroup sensor
/// @{

/// Namespace for Chrono::Sensor
namespace sensor {}

/// @} sensor

}  // namespace chrono

#endif
