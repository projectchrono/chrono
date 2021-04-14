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
// Authors: Alessandro Tasora
// =============================================================================

#ifndef CHPLATFORM_H
#define CHPLATFORM_H

// Functionality for API import/export symbols,
// in a platform independent way.
//
// When building the DLL, the CH_EXPORTS symbol must be defined.
//
// Each exported function / class in Chrono::Engine
// will use the 'ChApi' macro in headers (for non-MS compilers,
// this has no effect because all symbols will be exported).

#if ((defined _WIN32) || (defined(__MINGW32__) || defined(__CYGWIN__))) && defined(_DLL)
#if !defined(CH_DLL) && !defined(CH_STATIC)
#define CH_DLL
#endif
#endif

#if ((defined _WIN32) || (defined(__MINGW32__) || defined(__CYGWIN__))) && defined(CH_DLL)
#define ChApiEXPORT __declspec(dllexport)
#define ChApiIMPORT __declspec(dllimport)
#else
#define ChApiEXPORT
#define ChApiIMPORT
#endif

// Define a deprecated macro which generates a warning at compile time.
// Usage:
//   For typedef:         typedef CH_DEPRECATED int test1;
//   For classes/structs: class CH_DEPRECATED test2 { ... };
//   For methods:         class test3 { CH_DEPRECATED virtual void foo() {} };
//   For functions:       template<class T> CH_DEPRECATED void test4() {}
//
// When building the Chrono libraries, define CH_IGNORE_DEPRECATED to stop issuing these warnings.

#if defined(CH_IGNORE_DEPRECATED)
#define CH_DEPRECATED(msg)
#else
#if __cplusplus >= 201402L
#define CH_DEPRECATED(msg) [[deprecated(msg)]]
#elif defined(__GNUC__)
#define CH_DEPRECATED(msg) __attribute__((deprecated(msg)))
#elif defined(_MSC_VER)
#define CH_DEPRECATED(msg) __declspec(deprecated(msg))
#else
#define CH_DEPRECATED(msg)
#endif
#endif

#endif
