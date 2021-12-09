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

/// @brief Derived class of ChFsiForce that implements the IISPH method.
class ChFsiForceIISPH : public ChFsiForce {
  public:
    /// Force class implemented using IISPH method
    ChFsiForceIISPH(
      std::shared_ptr<ChBce> otherBceWorker,                   ///< Pointer to the ChBce object that handles BCE particles
      std::shared_ptr<SphMarkerDataD> otherSortedSphMarkersD,  ///< Information of SPH particles in the sorted array on device
      std::shared_ptr<ProximityDataD> otherMarkersProximityD,  ///< Pointer to the object that holds the proximity of particles on device
      std::shared_ptr<FsiGeneralData> otherFsiGeneralData,     ///< Pointer to the sph general data
      std::shared_ptr<SimParams> otherParamsH,                 ///< Pointer to the simulation parameters on host
      std::shared_ptr<NumberOfObjects> otherNumObjects         ///< Pointer to number of objects, fluid and boundary particles, etc.
    );

    ~ChFsiForceIISPH();
    void Finalize() override;

  private:
    void ForceSPH(std::shared_ptr<SphMarkerDataD> otherSphMarkersD,
                  std::shared_ptr<FsiBodiesDataD> otherFsiBodiesD,
                  std::shared_ptr<FsiMeshDataD> otherFsiMeshD) override;

    void calcPressureIISPH(std::shared_ptr<FsiBodiesDataD> otherFsiBodiesD,
                           thrust::device_vector<Real3> pos_fsi_fea_D,
                           thrust::device_vector<Real3> vel_fsi_fea_D,
                           thrust::device_vector<Real3> acc_fsi_fea_D,
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
