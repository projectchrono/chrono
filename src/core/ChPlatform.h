//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHPLATFORM_H
#define CHPLATFORM_H

//////////////////////////////////////////////////
//  
//   ChPlatform.h
//
//   platform-specific stuff
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////



// Functionality for API import/export symbols,
// in a platform independent way.
//
// When building the DLL, the CH_EXPORTS symbol must be defined.
//
// Each exported function / class in Chrono::Engine
// will use the 'ChApi' macro in headers (for non-MS compilers,
// this has no effect because all symbols will be exported).

#if (((defined WIN32)|| (defined WIN64))  || (defined(__MINGW32__) || defined(__CYGWIN__))) && defined(_DLL)
	#if !defined(CH_DLL) && !defined(CH_STATIC)
		#define CH_DLL
	#endif
#endif

#if (((defined WIN32)|| (defined WIN64))  || (defined(__MINGW32__) || defined(__CYGWIN__))) && defined(CH_DLL)
		#define ChApiEXPORT __declspec(dllexport)
		#define ChApiIMPORT __declspec(dllimport)	
#else
		#define ChApiEXPORT  
		#define ChApiIMPORT  
#endif


// Disable the C4251 warning under MSVC, that happens when using
// templated classes in data members of other exported classes.

#pragma warning(disable:4251)





#endif  // END of header

