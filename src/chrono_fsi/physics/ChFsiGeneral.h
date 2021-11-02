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

#include <memory>
#include "chrono_fsi/ChApiFsi.h"
#include "chrono_fsi/physics/ChParams.h"

namespace chrono {
namespace fsi {

/**
 * @brief Number of fluid markers, solid bodies, solid markers, boundary markers
 * @details
 * 		This structure holds the number of SPH particles and rigid/flexible bodies. Note that the order of makers in
 * 		the memory is as follows: i) fluid markers, ii) markers attached to fixed objects (boundary markers), iii)
 * 		markers attached to rigid bodies, and iv) markers attached to flexible bodies
 */
struct NumberOfObjects {
    size_t numRigidBodies;      ///< Number of rigid bodies
    size_t numFlexNodes;        ///< Number of Nodes in a flexible mesh, Each FE is made up of nodes
    size_t numFlexBodies1D;     ///< Number of 1D-Flexible bodies, Each FE is one body
    size_t numFlexBodies2D;     ///< Number of 2D-Flexible bodies, Each FE is one body
    size_t numGhostMarkers;     ///< Number of Ghost SPH markers that comes into play with Variable Resolution methods
    size_t numHelperMarkers;    ///< Number of helper SPH markers that is used for merging particles
    size_t numFluidMarkers;     ///< Number of fluid SPH markers
    size_t numBoundaryMarkers;  ///< Number of boundary SPH markers
    size_t startRigidMarkers;   ///< Index of the first SPH marker that covers the first rigid body.
    size_t startFlexMarkers;    ///< Index of the first SPH marker that covers the first flexible body.
    size_t numRigid_SphMarkers; ///< Number of SPH markers attached to rigid bodies
    size_t numFlex_SphMarkers;  ///< Number of SPH markers attached to flexible bodies
    size_t numAllMarkers;       ///< Total number of SPH markers in the simulation
};

class ChFsiGeneral {
  public:
    ChFsiGeneral();
    ChFsiGeneral(std::shared_ptr<SimParams> hostParams, 
                 std::shared_ptr<NumberOfObjects> hostNumObjects);
    ~ChFsiGeneral();

    /// @brief Compute number of blocks and threads for calculation on GPU
    /// This function calculates the number of blocks and threads for a given number of elements based on the blockSize
    void computeGridSize(uint n,           ///< Total num elements
                         uint blockSize,   ///< BlockSize Number of threads per block
                         uint& numBlocks,  ///< numBlocks (output)
                         uint& numThreads  ///< numThreads (output)
    );

  protected:
    uint iDivUp(uint a, uint b);

  private:
    std::shared_ptr<SimParams> paramsH;
    std::shared_ptr<NumberOfObjects> numObjectsH;
};
}  // end namespace fsi
}  // end namespace chrono

#endif
