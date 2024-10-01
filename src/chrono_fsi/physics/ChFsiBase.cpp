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

#include "chrono_fsi/physics/ChFsiBase.h"

namespace chrono {
namespace fsi {

ChFsiBase::ChFsiBase() : paramsH(NULL), numObjectsH(NULL) {}

ChFsiBase::ChFsiBase(std::shared_ptr<SimParams> params, std::shared_ptr<ChCounters> numObjects)
    : paramsH(params), numObjectsH(numObjects) {}

void ChFsiBase::computeGridSize(uint n, uint blockSize, uint& numBlocks, uint& numThreads) {
    uint n2 = (n == 0) ? 1 : n;
    numThreads = min(blockSize, n2);
    numBlocks = (n2 % numThreads != 0) ? (n2 / numThreads + 1) : (n2 / numThreads);
}

}  // end namespace fsi
}  // end namespace chrono
