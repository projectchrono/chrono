#ifndef CH_APISUBSYS_H
#define CH_APISUBSYS_H

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
    @brief Ground vehicle modeling

    This module introduces template-based modeling tools
    for creating wheeled and tracked vehicles.

    For additional information, see:
    - the [installation guide](@ref module_vehicle_installation)
    - the [tutorials](@ref tutorial_table_of_content_chrono_vehicle)
*/


namespace chrono {

/// @addtogroup vehicle
/// @{

/// Namespace with classes for the VEHICLE module.
namespace vehicle {}

/// @}

}

#endif
