// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
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

#ifndef CH_FSIGENERAL_H_
#define CH_FSIGENERAL_H_

#include "chrono_fsi/ChApiFsi.h"
#include "chrono_fsi/ChFsiDataManager.cuh"
#include "chrono_fsi/ChParams.cuh"
#include "chrono_fsi/custom_math.h"

namespace chrono {
namespace fsi {

class CH_FSI_API ChFsiGeneral {
public:
  ChFsiGeneral();
  ChFsiGeneral(SimParams *hostParams, NumberOfObjects *hostNumObjects);
  ~ChFsiGeneral();
  /**
* @brief computeGridSize
* @details Compute grid and thread block size for a given number of elements
*
* @param n Total number of elements. Each elements needs a thread to be computed
* @param blockSize Number of threads per block.
* @param numBlocks output
* @param numThreads Output: number of threads per block
*/
  void computeGridSize(uint n, uint blockSize, uint &numBlocks,
                       uint &numThreads);
  void setParameters(SimParams *hostParams, NumberOfObjects *hostNumObjects);
  virtual void Finalize(){};

protected:
  uint iDivUp(uint a, uint b);

private:
  SimParams *paramsH;
  NumberOfObjects *numObjectsH;
};
} // end namespace fsi
} // end namespace chrono

#endif
