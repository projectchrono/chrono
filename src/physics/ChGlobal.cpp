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

///////////////////////////////////////////////////
//  
//   ChGlobal.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include <string.h>

#include "physics/ChGlobal.h" 

#if defined(_WIN32) || defined(_WIN64)
#include "Windows.h"
#endif

#if defined(__APPLE__)
#include <libkern/OSAtomic.h>
#endif

namespace chrono
{


int GetUniqueIntID()
{
#if defined(_WIN32) || defined(_WIN64)
  volatile static long id = 100000;
  return (int)InterlockedIncrement(&id);
#endif

#if defined(__APPLE__)
  static volatile int32_t id = 100000;
  return (int)OSAtomicIncrement32Barrier(&id);
#endif

#if defined(__GNUC__)
  static volatile int id = 100000;
  return __sync_add_and_fetch(&id, 1);
#endif
}


} // END_OF_NAMESPACE____

