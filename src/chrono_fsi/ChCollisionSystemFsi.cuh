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

/// Base class for processing proximity in an FSI system.
class CH_FSI_API ChCollisionSystemFsi : public ChFsiGeneral {
  public:
    // ChCollisionSystemFsi();
    ChCollisionSystemFsi(SphMarkerDataD* otherSortedSphMarkersD,
                         ProximityDataD* otherMarkersProximityD,
                         SimParams* otherParamsH,
                         NumberOfObjects* otherNumObjects);
    ~ChCollisionSystemFsi();
    /**
                    * @brief Encapsulate calcHash and
     * reaorderDataAndFindCellStart;
                    * @details
    */
    void ArrangeData(SphMarkerDataD* otherSphMarkersD);

    virtual void Finalize();

  private:
    SphMarkerDataD* sphMarkersD;
    SphMarkerDataD* sortedSphMarkersD;
    ProximityDataD* markersProximityD;

    SimParams* paramsH;
    NumberOfObjects* numObjectsH;

    void ResetCellSize(int s);
    /**
  * @brief calcHash - calcHashD
  *
  * @details calcHash is the wrapper function for calcHashD. calcHashD is a kernel
  * function, which
  * means that all the code in it is executed in parallel on the GPU.
  * 			calcHashD:
  * 		 				1. Get particle index. Determine by the
  * block and thread we are in.
  * 		     			2. From x,y,z position determine which bin
  * it is in.
  * 		        		3. Calculate hash from bin index.
  * 		          		4. Store hash and particle index associated
  * with it.
  *
  * @param gridMarkerHash Store marker hash here
  * @param gridMarkerIndex Store marker index here
  * @param posRad Vector containing the positions of all particles, including
  * boundary particles
  * @param numAllMarkers Total number of markers (fluid + boundary)
  */
    void calcHash();
    /**
  * @brief Wrapper function for reorderDataAndFindCellStartD
  * @details
  * 		See SDKCollisionSystem.cuh for brief.
  */
    void reorderDataAndFindCellStart();
};

/// @} fsi_collision

} // end namespace fsi
} // end namespace chrono

#endif
