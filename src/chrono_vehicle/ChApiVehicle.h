#ifndef CH_APISUBSYS_H
#define CH_APISUBSYS_H

#include "core/ChPlatform.h"

// When compiling this library, remember to define CH_API_COMPILE_UNIT_VEHICLE
// (so that the symbols with 'CH_VEHICLE_API' in front of them will be marked as
// exported). Otherwise, just do not define it if you link the library to your
// code, and the symbols will be imported.

#if defined(CH_API_COMPILE_UNIT_VEHICLE)
#define CH_VEHICLE_API ChApiEXPORT
#else
#define CH_VEHICLE_API ChApiIMPORT
#endif

#endif
