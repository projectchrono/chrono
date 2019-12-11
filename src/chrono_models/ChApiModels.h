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
    @defgroup vehicle_models Vehicle models
    @brief Collection of concrete ground vehicle models
    @{
      @defgroup vehicle_models_hmmwv HMMWV model
      @defgroup vehicle_models_sedan Passenger car model
      @defgroup vehicle_models_citybus City bus model
      @defgroup vehicle_models_generic Generic wheeled vehicle
      @defgroup vehicle_models_m113 M113 tracked vehicle model
      @defgroup vehicle_models_uaz UAZ model
    @}
*/

#endif
