#ifndef CH_API_OGRE_H
#define CH_API_OGRE_H

#include "chrono/core/ChPlatform.h"

#if defined(CH_API_COMPILE_OGRE)
#define CHOGRE_DLL_TAG ChApiEXPORT
#else
#define CHOGRE_DLL_TAG ChApiIMPORT
#endif

#endif