// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Author: Aaron Young
// =============================================================================
//
// Macro defines for exporting DLL
//
// =============================================================================

#ifndef CH_ROS_API_H
#define CH_ROS_API_H

#include "chrono/ChVersion.h"
#include "chrono/core/ChPlatform.h"

// When compiling this library, remember to define CH_API_COMPILE_ROS
// (so that the symbols with 'CH_ROS_API' in front of them will be marked as
// exported). Otherwise, just do not define it if you link the library to your
// code, and the symbols will be imported.
// Note: For header-only classes, do NOT put CH_ROS_API in front of the class definition

#if defined(CH_API_COMPILE_ROS)
#define CH_ROS_API ChApiEXPORT
#else
#define CH_ROS_API ChApiIMPORT
#endif

/**
    @defgroup ros ROS module
    @brief Interface to the Robot Operating System (ROS)

    This module provides support for interfacing simulations with the ROS network.

    For additional information, see:
    - the [overview](@ref module_ros_overview)
    - the [installation guide](@ref module_ros_installation)

    @{
        @defgroup ros_core ROS Core
        @defgroup ros_handlers ROS Handlers
        @brief ROS Handlers
        @{
            @defgroup ros_vehicle_handlers ROS Vehicle Handlers
            @defgroup ros_sensor_handlers ROS Sensor Handlers
            @defgroup ros_robot_handlers ROS Robot Handlers
        @}
    @}
*/

namespace chrono {

/// @addtogroup ros
/// @{

/// Namespace for Chrono::ROS
namespace ros {}

/// @}
}  // namespace chrono

#endif
