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
// Author: Arman Pazouki
// =============================================================================
//
// Utility function to print structured data from fsi
// =============================================================================

#ifndef CH_FSI_UTILS_PRINTSTRUCT_H
#define CH_FSI_UTILS_PRINTSTRUCT_H

#include <iostream>

#include "chrono_fsi/ChApiFsi.h"
#include "chrono_fsi/sph/math/CustomMath.h"

namespace chrono {
namespace fsi {
namespace sph {

/// @addtogroup fsi_utils
/// @{

/// Print a Real2 struct.
void printStruct(struct Real2& s) {
    std::cout << "x = " << s.x << ", ";
    std::cout << "y = " << s.y << ", " << std::endl;
}

/// Print a Int2 struct.
void printStruct(struct int2& s) {
    std::cout << "x = " << s.x << ", ";
    std::cout << "y = " << s.y << ", " << std::endl;
}

/// Print a Real3 struct.
void printStruct(struct Real3& s) {
    std::cout << "x = " << s.x << ", ";
    std::cout << "y = " << s.y << ", ";
    std::cout << "z = " << s.z << ", " << std::endl;
}

/// Print a Int3 struct.
void printStruct(struct int3& s) {
    std::cout << "x = " << s.x << ", ";
    std::cout << "y = " << s.y << ", ";
    std::cout << "z = " << s.z << ", " << std::endl;
}

/// Print a Real4 struct.
void printStruct(struct Real4& s) {
    std::cout << "x = " << s.x << ", ";
    std::cout << "y = " << s.y << ", ";
    std::cout << "z = " << s.z << ", ";
    std::cout << "w = " << s.w << ", " << std::endl;
}

/// Print a Int4 struct.
void printStruct(struct int4& s) {
    std::cout << "x = " << s.x << ", ";
    std::cout << "y = " << s.y << ", ";
    std::cout << "z = " << s.z << ", ";
    std::cout << "w = " << s.w << ", " << std::endl;
}

/// @} fsi_utils

}  // end namespace sph
}  // end namespace fsi
}  // end namespace chrono

#endif
