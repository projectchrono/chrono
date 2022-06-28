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
// Author: Milad Rakhsha, Arman Pazouki
// =============================================================================
//
// Class for FSI properties and functions.
// =============================================================================

#include "chrono_fsi/physics/ChFsiGeneral.h"

namespace chrono {
namespace fsi {

ChFsiGeneral::ChFsiGeneral() : paramsH(NULL), numObjectsH(NULL) {}

ChFsiGeneral::ChFsiGeneral(std::shared_ptr<SimParams> other_paramsH, std::shared_ptr<ChCounters> other_numObjects)
    : paramsH(other_paramsH), numObjectsH(other_numObjects) {}

uint ChFsiGeneral::iDivUp(uint a, uint b) {
    return (a % b != 0) ? (a / b + 1) : (a / b);
}

void ChFsiGeneral::computeGridSize(uint n, uint blockSize, uint& numBlocks, uint& numThreads) {
    uint n2 = (n == 0) ? 1 : n;
    numThreads = min(blockSize, n2);
    numBlocks = iDivUp(n2, numThreads);
}

}  // end namespace fsi
}  // end namespace chrono
