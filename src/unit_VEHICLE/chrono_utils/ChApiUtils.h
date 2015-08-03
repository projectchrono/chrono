#ifndef CHAPIUTILS_H
#define CHAPIUTILS_H

#include "core/ChPlatform.h"

// When compiling this library, remember to define CH_API_COMPILE_UTILS
// (so that the symbols with 'CH_UTILS_API' in front of them will be marked as
// exported). Otherwise, just do not define it if you link the library to your
// code, and the symbols will be imported.

#if defined(CH_API_COMPILE_UTILS)
#define CH_UTILS_API ChApiEXPORT
#else
#define CH_UTILS_API ChApiIMPORT
#endif

#endif