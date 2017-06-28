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
// Author: Arman Pazouki, Milad Rakhsha
// =============================================================================
//
// Base class for processing proximity in an FSI system.
// =============================================================================

#ifndef CH_COLLISIONSYSTEM_FSI_H_
#define CH_COLLISIONSYSTEM_FSI_H_

#include "chrono_fsi/ChApiFsi.h"
#include "chrono_fsi/ChFsiDataManager.cuh"
#include "chrono_fsi/ChFsiGeneral.cuh"

namespace chrono {
namespace fsi {

/// @addtogroup fsi_collision
/// @{

/// Base class for processing proximity computation in an FSI system.
class CH_FSI_API ChCollisionSystemFsi : public ChFsiGeneral {
  public:
    /// Constructor of the ChCollisionSystemFsi
    ChCollisionSystemFsi(SphMarkerDataD* otherSortedSphMarkersD,  ///< Information of the markers in the sorted array
                         ProximityDataD* otherMarkersProximityD,  ///< Proximity information of the system
                         SimParams* otherParamsH,                 ///< Parameters of the simulation
                         NumberOfObjects* otherNumObjects         ///< Size of different objects in the system
                         );

    /// Destructor of the ChCollisionSystemFsi
    ~ChCollisionSystemFsi();

    /// Encapsulate calcHash and reaorderDataAndFindCellStart
    void ArrangeData(SphMarkerDataD* otherSphMarkersD);

    virtual void Finalize();

  private:
    SphMarkerDataD* sphMarkersD;        ///< Information of the markers in the original array
    SphMarkerDataD* sortedSphMarkersD;  ///< Information of the markers in the sorted array
    ProximityDataD* markersProximityD;  ///< Proximity information of the system
    SimParams* paramsH;                 ///< Parameters of the simulation
    NumberOfObjects* numObjectsH;       ///< Size of different objects in the system

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
