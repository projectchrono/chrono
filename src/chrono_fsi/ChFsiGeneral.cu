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
// Class for fsi properties and functions.//
// =============================================================================

#include "chrono_fsi/ChFsiGeneral.cuh"
namespace chrono {
namespace fsi {

ChFsiGeneral::ChFsiGeneral() : paramsH(NULL), numObjectsH(NULL) {}

ChFsiGeneral::ChFsiGeneral(SimParams *other_paramsH,
                           NumberOfObjects *other_numObjects)
    : paramsH(other_paramsH), numObjectsH(other_numObjects) {}

ChFsiGeneral::~ChFsiGeneral() {}

uint ChFsiGeneral::iDivUp(uint a, uint b) {
  return (a % b != 0) ? (a / b + 1) : (a / b);
}

void ChFsiGeneral::computeGridSize(uint n, uint blockSize, uint &numBlocks,
                                   uint &numThreads) {
  uint n2 = (n == 0) ? 1 : n;
  numThreads = min(blockSize, n2);
  numBlocks = iDivUp(n2, numThreads);
}

} // end namespace fsi
} // end namespace chrono