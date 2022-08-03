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
// Author: Arman Pazouki, Milad Rakhsha, Wei Hu
// =============================================================================
//
// Base class for processing proximity in an FSI system.
// =============================================================================

#ifndef CH_COLLISIONSYSTEM_FSI_H_
#define CH_COLLISIONSYSTEM_FSI_H_

#include "chrono_fsi/ChApiFsi.h"
#include "chrono_fsi/physics/ChFsiGeneral.h"
#include "chrono_fsi/physics/ChSystemFsi_impl.cuh"

namespace chrono {
namespace fsi {

/// @addtogroup fsi_collision
/// @{

/// Base class for processing proximity computation in an FSI system.
class ChCollisionSystemFsi : public ChFsiGeneral {
  public:
    /// Constructor of the ChCollisionSystemFsi class
    ChCollisionSystemFsi(
        std::shared_ptr<SphMarkerDataD> otherSortedSphMarkersD,  ///< Information of the particles in the sorted array
        std::shared_ptr<ProximityDataD> otherMarkersProximityD,  ///< Proximity information of the system
        std::shared_ptr<FsiGeneralData> otherFsiGeneralData,     ///< Pointer to the SPH general data
        std::shared_ptr<SimParams> otherParamsH,                 ///< Parameters of the simulation
        std::shared_ptr<ChCounters> otherNumObjects              ///< Size of different objects in the system
    );

    /// Destructor of the ChCollisionSystemFsi class
    ~ChCollisionSystemFsi();

    /// Encapsulate calcHash and reaorderDataAndFindCellStart
    void ArrangeData(std::shared_ptr<SphMarkerDataD> otherSphMarkersD);

    /// Complete construction.
    void Initialize();

  private:
    std::shared_ptr<SphMarkerDataD> sphMarkersD;        ///< Information of the particles in the original array
    std::shared_ptr<SphMarkerDataD> sortedSphMarkersD;  ///< Information of the particles in the sorted array
    std::shared_ptr<ProximityDataD> markersProximityD;  ///< Proximity information of the system
    std::shared_ptr<FsiGeneralData> fsiGeneralData;     ///< Pointer to the SPH general data
    std::shared_ptr<SimParams> paramsH;                 ///< Parameters of the simulation
    std::shared_ptr<ChCounters> numObjectsH;            ///< Size of different objects in the system

    void ResetCellSize(int s);

    /// calcHash is the wrapper function for calcHashD. calcHashD is a kernel
    /// function, which means that all the code in it is executed in parallel on the GPU.
    void calcHash();

    /// Wrapper function for reorderDataAndFindCellStartD
    void reorderDataAndFindCellStart();
};

/// @} fsi_collision

}  // end namespace fsi
}  // end namespace chrono

#endif
