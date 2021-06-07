// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2016 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Nic Olsen
// =============================================================================

#ifndef CH_API_DISTRIBUTED_H
#define CH_API_DISTRIBUTED_H

#include "chrono/core/ChPlatform.h"

// When compiling this library, remember to define CH_API_COMPILE_DISTRIBUTED
// (so that the symbols with 'CH_DISTR_API' in front of them will be
// marked as exported). Otherwise, just do not define it if you
// link the library to your code, and the symbols will be imported.

#if defined(CH_API_COMPILE_DISTRIBUTED)
#define CH_DISTR_API ChApiEXPORT
#else
#define CH_DISTR_API ChApiIMPORT
#endif

/**
    @defgroup distributed_module DISTRIBUTED module
    @brief Module for distributed parallel simulation

    This module provides support for granular dynamics on distributed parallel computing
    architectures through MPI. This module depends on the Chrono::Multicore module.

    For additional information, see:
    - the [installation guide](@ref module_distributed_installation)
    - the [tutorials](@ref tutorial_table_of_content_chrono_distributed)

    @{
        @defgroup distributed_collision Collision objects
        @defgroup distributed_comm Communication objects
        @defgroup distributed_physics Physics objects
    @}

*/

#endif
