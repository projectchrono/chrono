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
// Author: Milad Rakhsha, Arman Pazouki, Wei Hu
// =============================================================================
//
// Class for FSI properties and functions.
// =============================================================================

#ifndef CH_FSI_GENERAL_H
#define CH_FSI_GENERAL_H

#include <memory>
#include "chrono_fsi/ChApiFsi.h"
#include "chrono_fsi/physics/ChParams.h"

namespace chrono {
namespace fsi {

/// @addtogroup fsi_physics
/// @{

/**
 * @brief Number of rigid and flexible solid bodies, fluid SPH particles, solid SPH particles, boundary SPH particles
 * @details
 * 		This structure holds the number of SPH particles and rigid/flexible bodies. Note that the order of makers in
 * 		the memory is as follows: i) fluid particles (type = -1), ii) particles attached to fixed objects (boundary
 * 		particles with type = 0), iii) particles attached to rigid bodies (type = 1), and iv) particles attached to
 *    flexible bodies (type = 2).
 */
struct ChCounters {
    size_t numRigidBodies;      ///< Number of rigid bodies
    size_t numFlexNodes;        ///< Number of Nodes in a flexible mesh; each FE is made up of nodes
    size_t numFlexBodies1D;     ///< Number of 1D-Flexible bodies; each FE is one body
    size_t numFlexBodies2D;     ///< Number of 2D-Flexible bodies; each FE is one body
    size_t numGhostMarkers;     ///< Number of Ghost SPH particles that comes into play with Variable Resolution methods
    size_t numHelperMarkers;    ///< Number of helper SPH particles that is used for merging particles
    size_t numFluidMarkers;     ///< Number of fluid SPH particles
    size_t numBoundaryMarkers;  ///< Number of BCE markers on boundaries
    size_t numRigidMarkers;     ///< Number of BCE markers on rigid bodies
    size_t numFlexMarkers;      ///< Number of BCE markers on flexible bodies
    size_t numAllMarkers;       ///< Total number of particles in the simulation
    size_t startRigidMarkers;   ///< Index of the first BCE marker that covers the first rigid body.
    size_t startFlexMarkers;    ///< Index of the first BCE marker that covers the first flexible body.
};

/// Class for FSI properties and functions.
class ChFsiGeneral {
  public:
    ChFsiGeneral();

    /// Constructor for the ChFsiGeneral class.
    ChFsiGeneral(std::shared_ptr<SimParams> hostParams,      ///< Simulation parameters on host
                 std::shared_ptr<ChCounters> hostNumObjects  ///< Counters of objects and markers
    );

    /// Destructor of the ChFsiGeneral class.
    virtual ~ChFsiGeneral() {}

    /// Compute number of blocks and threads for calculation on GPU.
    /// This function calculates the number of blocks and threads for a given number of elements based on the blockSize
    void computeGridSize(uint n,           ///< Total num elements
                         uint blockSize,   ///< BlockSize Number of threads per block
                         uint& numBlocks,  ///< numBlocks (output)
                         uint& numThreads  ///< numThreads (output)
    );

  protected:
    /// Return a/b or a/b + 1
    uint iDivUp(uint a, uint b);

  private:
    std::shared_ptr<SimParams> paramsH;       ///< Simulation parameters on host.
    std::shared_ptr<ChCounters> numObjectsH;  ///< NUmber of objects on host.
};

/// @} fsi_physics

}  // end namespace fsi
}  // end namespace chrono

#endif
