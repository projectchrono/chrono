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

#include "chrono_fsi/physics/ChParams.h"

namespace chrono {
namespace fsi {

/// @addtogroup fsi_physics
/// @{

/// Number of rigid and flexible solid bodies, fluid SPH particles, solid SPH particles, boundary SPH particles.
/// This structure holds the number of SPH particles and rigid/flexible bodies.
///  Note that the order of makers in the memory is as follows:
///  -  (1) fluid particles (type = -1)
///  -  (2) particles attached to fixed objects (boundary particles with type = 0)
///  -  (3) particles attached to rigid bodies (type = 1)
///  -  (4) particles attached to flexible bodies (type = 2)
struct ChCounters {
    size_t numRigidBodies;   ///< Number of rigid bodies
    size_t numFlexNodes1D;   ///< Number of nodes in 1-D FEA mesh segments
    size_t numFlexNodes2D;   ///< Number of nodes in 2-D flexible mesh faces
    size_t numFlexNodes;     ///< Number of Nodes in a flexible mesh; each FE is made up of nodes //// OBSOLETE
    size_t numFlexBodies1D;  ///< Number of 1-D flexible bodies; each FE segment is one body
    size_t numFlexBodies2D;  ///< Number of 2-D flexible bodies; each FE face is one body

    size_t numGhostMarkers;     ///< Number of Ghost SPH particles for Variable Resolution methods
    size_t numHelperMarkers;    ///< Number of helper SPH particles used for merging particles
    size_t numFluidMarkers;     ///< Number of fluid SPH particles
    size_t numBoundaryMarkers;  ///< Number of BCE markers on boundaries
    size_t numRigidMarkers;     ///< Number of BCE markers on rigid bodies
    size_t numFlexMarkers1D;    ///< Number of BCE markers on flexible segments
    size_t numFlexMarkers2D;    ///< Number of BCE markers on flexible faces
    size_t numFlexMarkers;      ///< Number of BCE markers on flexible bodies //// OBSOLETE
    size_t numAllMarkers;       ///< Total number of particles in the simulation
    
    size_t startRigidMarkers;   ///< Index of first BCE marker on first rigid body
    size_t startFlexMarkers1D;  ///< Index of first BCE marker on first flex segment
    size_t startFlexMarkers2D;  ///< Index of first BCE marker on first flex face
    size_t startFlexMarkers;    ///< Index of the first BCE marker that covers the first flexible body //// OBSOLETE
};

/// Base class for various FSI classes.
/// Provides access to the FSI simulation parameters and counters.
class ChFsiBase {
  public:
    ChFsiBase();

    /// Constructor for the ChFsiBase class.
    ChFsiBase(std::shared_ptr<SimParams> params,      ///< Simulation parameters on host
                 std::shared_ptr<ChCounters> numObjects  ///< Counters of objects and markers
    );

    /// Destructor of the ChFsiBase class.
    virtual ~ChFsiBase() {}

    /// Compute number of blocks and threads for calculation on GPU.
    /// This function calculates the number of blocks and threads for a given number of elements based on the blockSize.
    void computeGridSize(uint n,           ///< Total num elements
                         uint blockSize,   ///< BlockSize Number of threads per block
                         uint& numBlocks,  ///< numBlocks (output)
                         uint& numThreads  ///< numThreads (output)
    );

  protected:
    std::shared_ptr<SimParams> paramsH;       ///< Simulation parameters on host
    std::shared_ptr<ChCounters> numObjectsH;  ///< NUmber of objects on host
};

/// @} fsi_physics

}  // end namespace fsi
}  // end namespace chrono

#endif
