#ifndef CHAPIGL_H
#define CHAPIGL_H

#include "core/ChPlatform.h"

// When compiling this library, remember to define CH_UTILS_OPENGL_API
// (so that the symbols with 'ChApi' in front of them will be
// marked as exported). Otherwise, just do not define it if you
// link the library to your code, and the symbols will be imported.

#if defined(CH_API_COMPILE_UTILS_OPENGL)
	#define CH_UTILS_OPENGL_API ChApiEXPORT
#else
	#define CH_UTILS_OPENGL_API ChApiINPORT
#endif

#endif  // END of header