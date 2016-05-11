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
// Author: Arman Pazouki
// =============================================================================
//
// Macro defines for exporting DLL
// =============================================================================

#ifndef CHAPIFSI_H_
#define CHAPIFSI_H_

#include "chrono/core/ChPlatform.h"

// When compiling this library, remember to define CH_API_COMPILE_FSI
// (so that the symbols with 'CH_FSI_API' in front of them will be
// marked as exported). Otherwise, just do not define it if you
// link the library to your code, and the symbols will be imported.

#if defined(CH_API_COMPILE_FSI)
#define CH_FSI_API ChApiEXPORT
#else
#define CH_FSI_API ChApiIMPORT
#endif

#endif /* CHAPIFSI_H_ */
