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

#ifndef CHAPIPYPARSER_H
#define CHAPIPYPARSER_H

//////////////////////////////////////////////////
//  
//   ChApiPyParser.h
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

//#pragma warning(disable: 4251)

#include "core/ChPlatform.h"


// Chrono::Engine unit PYTHON version
//
// This is an integer, as 0xaabbccdd where
// for example version 1.2.0 is 0x00010200

#define CH_VERSION_PYPARSER_MODULE 0x00010200


// When compiling this library, remember to define CH_API_COMPILE_PYPARSER
// (so that the symbols with 'ChApiPYPARSER' in front of them will be
// marked as exported). Otherwise, just do not define it if you 
// link the library to your code, and the symbols will be imported.

#if defined(CH_API_COMPILE_PYPARSER)
	#define ChApiPYPARSER ChApiEXPORT
#else
	#define ChApiPYPARSER ChApiIMPORT	
#endif




#endif  // END of header