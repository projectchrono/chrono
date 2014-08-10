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

// -----------------------------------------------------------------------------
// Functions for assigning unique identifiers
// -----------------------------------------------------------------------------

// Set the start value for the sequence of IDs (ATTENTION: not thread safe)
// Subsequent calls to GetUniqueIntID() will return val+1, val+2, etc.
static volatile int first_id = 100000;

void SetFirstIntID(int val)
{
  first_id = val;
}

// Obtain a unique identifier (thread-safe)
int GetUniqueIntID()
{
#if defined(_WIN32) || defined(_WIN64)
  volatile static long id = first_id;
  return (int)InterlockedIncrement(&id);
#endif

#if defined(__APPLE__)
  static volatile int32_t id = first_id;
  return (int)OSAtomicIncrement32Barrier(&id);
#endif

#if defined(__GNUC__)
  #if defined(__APPLE__)
  #else
      static volatile int id = first_id;
      return __sync_add_and_fetch(&id, 1);
  #endif
#endif
}


// -----------------------------------------------------------------------------
// Functions for manipulating the Chrono data directory
// -----------------------------------------------------------------------------

static std::string chrono_data_path("../data/");

// Set the path to the Chrono data directory (ATTENTION: not thread safe)
void SetChronoDataPath(const std::string& path)
{
  chrono_data_path = path;
}

// Obtain the current path to the Chrono data directory (thread safe)
const std::string& GetChronoDataPath()
{
  return chrono_data_path;
}

// Obtain the complete path to the specified filename, given relative to the
// Chrono data directory (thread safe)
std::string GetChronoDataFile(const std::string& filename)
{
  return chrono_data_path + filename;
}


} // END_OF_NAMESPACE____

