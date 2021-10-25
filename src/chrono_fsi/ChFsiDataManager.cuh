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
// Base class for managing data in chrono_fsi, aka fluid system.//
// =============================================================================

#ifndef CH_FSI_DATAMANAGER_H_
#define CH_FSI_DATAMANAGER_H_
#include <thrust/device_vector.h>
#include <thrust/host_vector.h>
#include <thrust/iterator/detail/normal_iterator.h>
#include <thrust/iterator/transform_iterator.h>
#include <thrust/iterator/zip_iterator.h>

#include <thrust/tuple.h>

#include "chrono_fsi/ChApiFsi.h"
#include "chrono_fsi/physics/ChParams.h"
#include "chrono_fsi/math/custom_math.h"
#include "chrono_fsi/utils/ChUtilsDevice.cuh"

namespace chrono {
namespace fsi {



/// Data manager class that holds all the information of the SPH markers and MBD system
class CH_FSI_API ChFsiDataManager {
  public:
    ChFsiDataManager();
    ~ChFsiDataManager();

    void AddSphMarker(Real4 pos, Real3 vel, Real4 rhoPresMu, Real3 tauXxYyZz = mR3(0.0), Real3 tauXyXzYz = mR3(0.0));
    void ResizeDataManager(int numNode = 0);

    std::shared_ptr<NumberOfObjects> numObjects;

    std::shared_ptr<SphMarkerDataD> sphMarkersD1;       ///< Information of SPH markers at state 1 on device
    std::shared_ptr<SphMarkerDataD> sphMarkersD2;       ///< Information of SPH markers at state 2 on device
    std::shared_ptr<SphMarkerDataD> sortedSphMarkersD;  ///< Sorted information of SPH markers at state 1 on device
    std::shared_ptr<SphMarkerDataH> sphMarkersH;        ///< Information of SPH markers on host

    std::shared_ptr<FsiBodiesDataD> fsiBodiesD1;  ///< Information of rigid bodies at state 1 on device
    std::shared_ptr<FsiBodiesDataD> fsiBodiesD2;  ///< Information of rigid bodies at state 2 on device
    std::shared_ptr<FsiBodiesDataH> fsiBodiesH;   ///< Information of rigid bodies at state 1 on host
    std::shared_ptr<FsiMeshDataD> fsiMeshD;
    std::shared_ptr<FsiMeshDataH> fsiMeshH;

    std::shared_ptr<FsiGeneralData> fsiGeneralData;

    std::shared_ptr<ProximityDataD> markersProximityD;

  private:
    void ArrangeDataManager();
    void ConstructReferenceArray();
    void InitNumObjects();
    void CalcNumObjects();  ///< Calculates the number of rigid bodies, flexible bodies, etc. based on the type of markers
};

}  // end namespace fsi
}  // end namespace chrono

#endif /* CH_FSI_DATAMANAGER_H_ */
