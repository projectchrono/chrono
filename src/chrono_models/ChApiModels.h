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


#endif
