//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//
// File author: A.Tasora

#ifndef CHAPIFEM_H
#define CHAPIFEM_H


#include "core/ChPlatform.h"

// Chrono::Engine version
//
// This is an integer, as 0xaabbccdd where
// for example version 1.2.0 is 0x00010200

#define CH_VERSION_UNIT_FEM 0x00000001

// When compiling this library, remember to define CH_API_COMPILE_UNIT_FEM
// (so that the symbols with 'ChApiFem' in front of them will be
// marked as exported). Otherwise, just do not define it if you
// link the library to your code, and the symbols will be imported.

#if defined(CH_API_COMPILE_UNIT_FEM)
	#define ChApiFem ChApiEXPORT
#else
	#define ChApiFem ChApiINPORT
#endif

#endif  // END of header