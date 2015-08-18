// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// =============================================================================

#ifndef CH_API_UTILS_H
#define CH_API_UTILS_H

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
