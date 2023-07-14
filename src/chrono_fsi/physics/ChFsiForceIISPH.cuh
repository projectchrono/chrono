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
// Author: Milad Rakhsha
// =============================================================================

#ifndef CH_FSI_FORCEIISPH_H_
#define CH_FSI_FORCEIISPH_H_

#include "chrono_fsi/ChApiFsi.h"
#include "chrono_fsi/physics/ChFsiForce.cuh"

namespace chrono {
namespace fsi {

/// @addtogroup fsi_physics
/// @{

/// Inter-particle force calculation for the IISPH method.
class ChFsiForceIISPH : public ChFsiForce {
  public:
    /// Force class implemented using IISPH method
    ChFsiForceIISPH(
        std::shared_ptr<ChBce> otherBceWorker,                   ///< object that handles BCE particles
        std::shared_ptr<SphMarkerDataD> otherSortedSphMarkersD,  ///< information of particle in the sorted device array
        std::shared_ptr<ProximityDataD> otherMarkersProximityD,  ///< object that holds device proximity info
        std::shared_ptr<FsiData> otherFsiGeneralData,            ///< SPH general data
        std::shared_ptr<SimParams> params,                       ///< simulation parameters
        std::shared_ptr<ChCounters> numObjects,                  ///< problem counters
        bool verb                                                ///< verbose output
    );

    ~ChFsiForceIISPH();
    void Initialize() override;

  private:
    void ForceSPH(std::shared_ptr<SphMarkerDataD> otherSphMarkersD,
                  std::shared_ptr<FsiBodyStateD> fsiBodyStateD,
                  std::shared_ptr<FsiMeshStateD> fsiMesh1DStateD,
                  std::shared_ptr<FsiMeshStateD> fsiMesh2DStateD) override;

    void calcPressureIISPH(std::shared_ptr<FsiBodyStateD> fsiBodyStateD,
                           std::shared_ptr<FsiMeshStateD> fsiMesh1DStateD,
                           std::shared_ptr<FsiMeshStateD> fsiMesh2DStateD,
                           thrust::device_vector<Real> sumWij_inv,
                           thrust::device_vector<Real>& p_old,
                           thrust::device_vector<Real3> Normals,
                           thrust::device_vector<Real> G_i,
                           thrust::device_vector<Real>& Color);
};

/// @} fsi_physics

}  // end namespace fsi
}  // end namespace chrono

#endif
