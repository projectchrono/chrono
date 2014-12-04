//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//
// Author: Justin Madsen


#ifndef CH_APISUBSYS_H
#define CH_APISUBSYS_H

#include "core/ChPlatform.h"

// When compiling this library, remember to define CH_API_COMPILE_SUBSYS
// (so that the symbols with 'CH_SUBSYS_API' in front of them will be marked as
// exported). Otherwise, just do not define it if you link the library to your
// code, and the symbols will be imported.

#if defined(CH_API_COMPILE_UNIT_TRACKS)
#define CH_SUBSYS_API ChApiEXPORT
#else
#define CH_SUBSYS_API ChApiIMPORT
#endif

#endif
