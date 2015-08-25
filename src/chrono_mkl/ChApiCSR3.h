//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2011-2012 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHAPICSR3_H
#define CHAPICSR3_H

//////////////////////////////////////////////////
//
//   ChApiCE.h
//
//   Base header for all headers that have symbols
//   that can be exported.
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "core/ChPlatform.h"

// When compiling this library, remember to define CH_API_COMPILE_MKL
// (so that the symbols with 'ChApiCSR3' in front of them will be
// marked as exported). Otherwise, just do not define it if you
// link the library to your code, and the symbols will be imported.

#if defined(CH_API_COMPILE_MKL)
#define ChApiCSR3 ChApiEXPORT
#else
#define ChApiCSR3 ChApiIMPORT
#endif

#endif  // END of header
