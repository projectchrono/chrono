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
// Class for fsi properties and functions.//
// =============================================================================

#ifndef CH_FSIGENERAL_H_
#define CH_FSIGENERAL_H_

#include "chrono_fsi/ChApiFsi.h"
#include "chrono_fsi/physics/ChParams.cuh"
#include "chrono_fsi/ChFsiDataManager.cuh"

namespace chrono {
namespace fsi {

class CH_FSI_API ChFsiGeneral {
  public:
    ChFsiGeneral();
    ChFsiGeneral(std::shared_ptr<SimParams> hostParams, std::shared_ptr<NumberOfObjects> hostNumObjects);
    ~ChFsiGeneral();

    /// @brief Compute number of blocks and threads for calculation on GPU
    /// This function calculates the number of blocks and threads for a given number of elements based on the blockSize
    void computeGridSize(uint n,           ///< Total num elements
                         uint blockSize,   ///< BlockSize Number of threads per block
                         uint& numBlocks,  ///< numBlocks (output)
                         uint& numThreads  ///< numThreads (output)
    );

    void setParameters(std::shared_ptr<SimParams> hostParams, std::shared_ptr<NumberOfObjects> hostNumObjects);
    //    virtual void Finalize(){};
  protected:
    uint iDivUp(uint a, uint b);

  private:
    std::shared_ptr<SimParams> paramsH;
    std::shared_ptr<NumberOfObjects> numObjectsH;
};
}  // end namespace fsi
}  // end namespace chrono

#endif
