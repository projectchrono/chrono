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
    @brief Module that enables distributed parallel computation in Chrono

    This module implements MPI parallel computing algorithms that can be
    used as a faster alternative to the default simulation algorithms
    in Chrono::Engine. This module depends on the Chrono::Parallel module.

    For additional information, see:
    - the [installation guide](@ref module_distributed_installation)
*/

#endif