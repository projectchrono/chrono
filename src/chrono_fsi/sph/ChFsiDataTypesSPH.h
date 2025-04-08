// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2025 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Author: Radu Serban
// =============================================================================
//
// Math utilities for the Chrono::FSI module.
// These functions can be invoked either on the CPU (host) or on the GPU (device)
//
// =============================================================================

#ifndef CHFSI_DATA_TYPES_H
#define CHFSI_DATA_TYPES_H

#include <cuda_runtime.h>
#include <cmath>
#include <ostream>

#include "chrono_fsi/ChConfigFsi.h"

namespace chrono {
namespace fsi {
namespace sph {

/// @addtogroup fsisph
/// @{

/// Real type used in the SPH Chrono::FSI module (float or double).
#ifdef CHRONO_FSI_USE_DOUBLE
typedef double Real;
#else
typedef float Real;
#endif

/// Unsigned int type used in the SPH Chrono::FSI module.
typedef unsigned int uint;

/// Unsigned short type used in the SPH Chrono::FSI module.
typedef unsigned short ushort;

/// Pair of reals.
struct Real2 {
    Real x;
    Real y;
};

/// Triplet of reals.
struct Real3 {
    Real x;
    Real y;
    Real z;
};

/// Quadruplet of reals.
struct Real4 {
    Real x;
    Real y;
    Real z;
    Real w;
};

/// Insertion of a Real2 to output stream.
inline std::ostream& operator<<(std::ostream& out, const Real2& v) {
    out << v.x << "  " << v.y;
    return out;
}

/// Insertion of a Real3 to output stream.
inline std::ostream& operator<<(std::ostream& out, const Real3& v) {
    out << v.x << "  " << v.y << "  " << v.z;
    return out;
}

/// Insertion of a Real4 to output stream.
inline std::ostream& operator<<(std::ostream& out, const Real4& v) {
    out << v.x << "  " << v.y << "  " << v.z << "  " << v.w;
    return out;
}

/// @} fsisph

}  // namespace sph
}  // end namespace fsi
}  // end namespace chrono

#endif
