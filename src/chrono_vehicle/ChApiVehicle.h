// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================

#ifndef CH_APISUBSYS_H
#define CH_APISUBSYS_H

#include "chrono/ChVersion.h"
#include "chrono/core/ChPlatform.h"

// When compiling this library, remember to define CH_API_COMPILE_VEHICLE
// (so that the symbols with 'CH_VEHICLE_API' in front of them will be marked as
// exported). Otherwise, just do not define it if you link the library to your
// code, and the symbols will be imported.

#if defined(CH_API_COMPILE_VEHICLE)
#define CH_VEHICLE_API ChApiEXPORT
#else
#define CH_VEHICLE_API ChApiIMPORT
#endif


/**
    @defgroup vehicle VEHICLE module
    @brief Ground vehicle modeling and simulation

    This module introduces template-based modeling tools
    for creating wheeled and tracked vehicles.

    For additional information, see:
    - the [installation guide](@ref module_vehicle_installation)
    - the [tutorials](@ref tutorial_table_of_content_chrono_vehicle)

    @{
        @defgroup vehicle_driver Driver models
        @defgroup vehicle_powertrain Powertrain models
        @defgroup vehicle_terrain Terrain models
        @defgroup vehicle_utils Utility classes
        
        @defgroup vehicle_wheeled Wheeled vehicles
        @{
            @defgroup vehicle_wheeled_suspension Suspension subsystem
            @defgroup vehicle_wheeled_steering Steering subsystem
            @defgroup vehicle_wheeled_tire Tire subsystem
            @defgroup vehicle_wheeled_driveline Driveline subsystem
            @defgroup vehicle_wheeled_antirollbar Anti-roll bar subsystem
            @defgroup vehicle_wheeled_wheel Wheel subsystem
            @defgroup vehicle_wheeled_brake Brake subsystem
            @defgroup vehicle_wheeled_test_rig Suspension test rig classes
            @defgroup vehicle_wheeled_utils Utility classes
        @}

        @defgroup vehicle_tracked Tracked vehicles
        @{
            @defgroup vehicle_tracked_idler Idler subsystem
            @defgroup vehicle_tracked_suspension Suspension subsystem
            @defgroup vehicle_tracked_roller Roller subsystem
            @defgroup vehicle_tracked_sprocket Sprocket subsystem
            @defgroup vehicle_tracked_brake Brake subsystem
            @defgroup vehicle_tracked_driveline Driveline subsystem
            @defgroup vehicle_tracked_shoe Track-shoe subsystem
            @defgroup vehicle_tracked_test_rig Track test rig classes
            @defgroup vehicle_tracked_utils Utility classes
        @}
    @}
*/


namespace chrono {

/// @addtogroup vehicle
/// @{

/// Namespace with classes for the VEHICLE module.
namespace vehicle {}

/// @}

}

#endif
