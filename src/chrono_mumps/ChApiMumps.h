// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Dario Mangoni
// =============================================================================

#ifndef CHAPIMUMPS_H
#define CHAPIMUMPS_H

#include "chrono/core/ChPlatform.h"

// Chrono::Engine version
//
// This is an integer, as 0xaabbccdd where
// for example version 1.2.0 is 0x00010200

#define CH_VERSION_MUMPS_MODULE 0x00000100

// When compiling this library, remember to define CH_API_COMPILE_MUMPS
// (so that the symbols with 'ChApiMumps' in front of them will be
// marked as exported). Otherwise, just do not define it if you
// link the library to your code, and the symbols will be imported.

#if defined(CH_API_COMPILE_MUMPS)
#define ChApiMumps ChApiEXPORT
#else
#define ChApiMumps ChApiIMPORT
#endif

#endif  // END of header
