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

#include <cstring>

#include "chrono/physics/ChGlobal.h"
#include "chrono/core/ChFileutils.h"

#if defined(_WIN32) || defined(_WIN64)
#include "Windows.h"
#endif

#if defined(__APPLE__)
#include <libkern/OSAtomic.h>
#endif

namespace chrono {

// -----------------------------------------------------------------------------
// Functions for assigning unique identifiers
// -----------------------------------------------------------------------------

// Set the start value for the sequence of IDs (ATTENTION: not thread safe)
// Subsequent calls to GetUniqueIntID() will return val+1, val+2, etc.
static volatile int first_id = 100000;

void SetFirstIntID(int val) {
    first_id = val;
}

// Obtain a unique identifier (thread-safe; platform-dependent)
#if (defined(__GCC_HAVE_SYNC_COMPARE_AND_SWAP_4) || defined(__ARM_ARCH_5T__) || defined(__ARM_ARCH_5TE__))

int GetUniqueIntID() {
    static volatile int id = first_id;
    return __sync_add_and_fetch(&id, 1);
}

#elif defined(_WIN32)

int GetUniqueIntID() {
    volatile static long id = first_id;
    return (int)InterlockedIncrement(&id);
}

#elif defined(_WIN64)

int GetUniqueIntID() {
    volatile static long long id = first_id;
    return (int)InterlockedIncrement64(&id);
}

#elif defined(__APPLE__)

int GetUniqueIntID() {
    static volatile int32_t id = first_id;
    return (int)OSAtomicIncrement32Barrier(&id);
}

#else

//// TODO
#error "No support for atomic operations on current platform!"

#endif

// -----------------------------------------------------------------------------
// Functions for manipulating the Chrono data directory
// -----------------------------------------------------------------------------

static std::string chrono_data_path("../data/");

// Set the path to the Chrono data directory (ATTENTION: not thread safe)
void SetChronoDataPath(const std::string& path) {
    chrono_data_path = path;
}

// Obtain the current path to the Chrono data directory (thread safe)
const std::string& GetChronoDataPath() {
    return chrono_data_path;
}

// Obtain the complete path to the specified filename, given relative to the
// Chrono data directory (thread safe)
std::string GetChronoDataFile(const std::string& filename) {
    return chrono_data_path + filename;
}

// -----------------------------------------------------------------------------
// Functions for manipulating the Chrono data directory
// -----------------------------------------------------------------------------

static std::string chrono_out_path("DEMO_OUTPUT/");

const std::string& GetChronoOutputPath() {
    // If the directory does not yet exists, create it.
    ChFileutils::MakeDirectory(chrono_out_path.c_str());

    return chrono_out_path;
}

}  // end namespace chrono
