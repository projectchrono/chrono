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
#ifndef CHUTILSPRINTSTRUCT_H
#define CHUTILSPRINTSTRUCT_H

#include "chrono_fsi/ChApiFsi.h"
#include "chrono_fsi/custom_math.h"
#include <thrust/device_vector.h>
#include <thrust/host_vector.h>

namespace chrono {
namespace fsi {
namespace utils {

void printStruct(struct Real2 &s) {
  std::cout << "x = " << s.x << ", ";
  std::cout << "y = " << s.y << ", " << std::endl;
}

void printStruct(struct int2 &s) {
  std::cout << "x = " << s.x << ", ";
  std::cout << "y = " << s.y << ", " << std::endl;
}

void printStruct(struct Real3 &s) {
  std::cout << "x = " << s.x << ", ";
  std::cout << "y = " << s.y << ", ";
  std::cout << "z = " << s.z << ", " << std::endl;
}

void printStruct(struct int3 &s) {
  std::cout << "x = " << s.x << ", ";
  std::cout << "y = " << s.y << ", ";
  std::cout << "z = " << s.z << ", " << std::endl;
}

void printStruct(struct Real4 &s) {
  std::cout << "x = " << s.x << ", ";
  std::cout << "y = " << s.y << ", ";
  std::cout << "z = " << s.z << ", ";
  std::cout << "w = " << s.w << ", " << std::endl;
}

void printStruct(struct int4 &s) {
  std::cout << "x = " << s.x << ", ";
  std::cout << "y = " << s.y << ", ";
  std::cout << "z = " << s.z << ", ";
  std::cout << "w = " << s.w << ", " << std::endl;
}

} // end namespace utils
} // end namespace fsi
} // end namespace chrono

#endif
