#ifndef CH_APISUBSYS_H
#define CH_APISUBSYS_H

#include "core/ChPlatform.h"

// When compiling this library, remember to define CH_API_COMPILE_SUBSYS
// (so that the symbols with 'CH_SUBSYS_API' in front of them will be marked as
// exported). Otherwise, just do not define it if you link the library to your
// code, and the symbols will be imported.

#if defined(CH_API_COMPILE_SUBSYS)
#define CH_SUBSYS_API ChApiEXPORT
#else
#define CH_SUBSYS_API ChApiIMPORT
#endif

#endif
