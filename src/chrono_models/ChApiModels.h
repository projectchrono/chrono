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

#ifndef CH_API_MODELS_H
#define CH_API_MODELS_H

#include "chrono/core/ChPlatform.h"

// When compiling the Chrono models libraries, remember to define CH_API_COMPILE_MODELS
// (so that the symbols with 'CH_MODELS_API' in front of them will be marked as
// exported). Otherwise, just do not define it if you link the library to your
// code, and the symbols will be imported.

#if defined(CH_API_COMPILE_MODELS)
    #define CH_MODELS_API ChApiEXPORT
#else
    #define CH_MODELS_API ChApiIMPORT
#endif

/**
    @defgroup chrono_models Models
    @brief Collections of Chrono models
    @{
       @defgroup vehicle_models Vehicle models
       @brief Collection of concrete ground vehicle models
       @{
         @defgroup vehicle_models_hmmwv HMMWV
         @defgroup vehicle_models_feda FED-alpha vehicle
         @defgroup vehicle_models_sedan Passenger car
         @defgroup vehicle_models_citybus City bus
         @defgroup vehicle_models_uaz UAZ vehicle
         @defgroup vehicle_models_gator Gator vehicle
         @defgroup vehicle_models_fmtv FMTV trucks
         @defgroup vehicle_models_man MAN trucks
         @defgroup vehicle_models_kraz Kraz 64431 truck
         @defgroup vehicle_models_mrole Multi-role vehicle
         @defgroup vehicle_models_artcar ARTcar vehicle
         @defgroup vehicle_models_generic Generic wheeled vehicle
         @defgroup vehicle_models_m113 M113 tracked vehicle
         @defgroup vehicle_models_marder Marder tracked vehicle
         @defgroup vehicle_models_duro Duro vehicle
         @defgroup vehicle_models_gclass Mercedes G500 vehicle
         @defgroup vehicle_models_bmw BMW E90 vehicle
         @defgroup vehicle_models_cherokee Jeep Cherokee vehicle
         @defgroup vehicle_models_unimog Unimog 401 vehicle
       @}
       @defgroup robot_models Robot models
       @brief Collection of robotic system models
       @{
         @defgroup robot_models_robosimian RoboSimian legged robot model
         @defgroup robot_models_viper Viper moon rover model
         @defgroup robot_models_curiosity Curiosity Mars rover model
         @defgroup robot_models_turtlebot Turtlebot robot model
         @defgroup robot_models_copter LittleHexy copter model
         @defgroup robot_models_industrial Industrial robot models
       @}
    @}
*/

#endif
