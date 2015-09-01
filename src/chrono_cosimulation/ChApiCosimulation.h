//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2012 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHAPICOSIMULATION_H
#define CHAPICOSIMULATION_H

#include "chrono/core/ChPlatform.h"

// Chrono::Engine version
//
// This is an integer, as 0xaabbccdd where
// for example version 1.2.0 is 0x00010200

#define CH_VERSION_COSIMULATION_MODULE 0x00000100

// When compiling this library, remember to define CH_API_COMPILE_COSIMULATION
// (so that the symbols with 'ChApiCosimulation' in front of them will be
// marked as exported). Otherwise, just do not define it if you
// link the library to your code, and the symbols will be imported.

#if defined(CH_API_COMPILE_COSIMULATION)
#define ChApiCosimulation ChApiEXPORT
#else
#define ChApiCosimulation ChApiIMPORT
#endif

#endif  // END of header