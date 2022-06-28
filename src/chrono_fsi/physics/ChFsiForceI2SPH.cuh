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

#ifndef CH_FSI_FORCEI2SPH_H_
#define CH_FSI_FORCEI2SPH_H_

#include "chrono_fsi/ChApiFsi.h"
#include "chrono_fsi/physics/ChFsiForce.cuh"

namespace chrono {
namespace fsi {

/// @addtogroup fsi_physics
/// @{

/// @brief Derived class of ChFsiForce that implements the I2SPH method.
class ChFsiForceI2SPH : public ChFsiForce {
  public:
    /// Force class implemented using incompressible SPH method with implicit integrator
    ChFsiForceI2SPH(
        std::shared_ptr<ChBce> otherBceWorker,                   ///< object that handles BCE particles
        std::shared_ptr<SphMarkerDataD> otherSortedSphMarkersD,  ///< information of particle in the sorted device array
        std::shared_ptr<ProximityDataD> otherMarkersProximityD,  ///< object that holds device proximity info
        std::shared_ptr<FsiGeneralData> otherFsiGeneralData,     ///< SPH general data
        std::shared_ptr<SimParams> otherParamsH,                 ///< simulation parameters on host
        std::shared_ptr<ChCounters> otherNumObjects,        ///< counters
        bool verb                                                ///< verbose terminal output
    );

    ~ChFsiForceI2SPH();
    void Initialize() override;

  private:
    thrust::device_vector<Real> _sumWij_inv;
    thrust::device_vector<uint> Contact_i;
    thrust::device_vector<Real> G_i;
    thrust::device_vector<Real> A_i;
    thrust::device_vector<Real> L_i;
    thrust::device_vector<uint> csrColInd;
    thrust::device_vector<Real> csrValLaplacian;
    thrust::device_vector<Real3> csrValGradient;
    thrust::device_vector<Real> csrValFunciton;
    thrust::device_vector<Real> AMatrix;
    thrust::device_vector<Real3> Normals;
    thrust::device_vector<Real3> V_star_new;
    thrust::device_vector<Real3> V_star_old;
    thrust::device_vector<Real> q_new;
    thrust::device_vector<Real> q_old;
    thrust::device_vector<Real> b1Vector;
    thrust::device_vector<Real3> b3Vector;
    thrust::device_vector<Real> Residuals;
    bool *isErrorH, *isErrorD, *isErrorD2;
    size_t numAllMarkers;
    int NNZ;
    void ForceSPH(std::shared_ptr<SphMarkerDataD> otherSphMarkersD,
                  std::shared_ptr<FsiBodiesDataD> otherFsiBodiesD,
                  std::shared_ptr<FsiMeshDataD> otherFsiMeshD) override;
    void PreProcessor(std::shared_ptr<SphMarkerDataD> otherSphMarkersD,
                      bool calcLaplacianOperator = true);
};

/// @} fsi_physics

}  // end namespace fsi
}  // end namespace chrono

#endif
