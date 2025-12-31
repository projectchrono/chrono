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
// Author: Arman Pazouki, Wei Hu, Radu Serban
// =============================================================================

#ifndef CH_SPH_FORCE_EXPLICITSPH_H
#define CH_SPH_FORCE_EXPLICITSPH_H

#include "chrono_fsi/ChApiFsi.h"
#include "chrono_fsi/sph/physics/SphForce.cuh"

namespace chrono {
namespace fsi {
namespace sph {

/// @addtogroup fsisph_physics
/// @{

/// Inter-particle force calculation for explicit SPH schemes.
class SphForceWCSPH : public SphForce {
  public:
    /// Force class implemented using WCSPH with an explicit integrator.
    /// Supports for both fluid and granular material dynamics.
    SphForceWCSPH(FsiDataManager& data_mgr,  ///< FSI data manager
                  SphBceManager& bce_mgr,       ///< BCE manager
                  bool verbose,              ///< verbose output
                  bool check_errors          ///< check errors
    );

    ~SphForceWCSPH();

    void Initialize() override;

  private:
    /// Function to calculate forces on SPH particles.
    void ForceSPH(std::shared_ptr<SphMarkerDataD> sortedSphMarkersD, Real time, Real step) override;

    /// Perform density re-initialization (as needed).
    void DensityReinitialization(std::shared_ptr<SphMarkerDataD> sortedSphMarkersD);

    // CRM
    void CrmApplyBC(std::shared_ptr<SphMarkerDataD> sortedSphMarkersD);
    void CrmCalcRHS(std::shared_ptr<SphMarkerDataD> sortedSphMarkersD);

    // CFD
    void CfdApplyBC(std::shared_ptr<SphMarkerDataD> sortedSphMarkersD);
    void CfdCalcRHS(std::shared_ptr<SphMarkerDataD> sortedSphMarkersD);

    /// Function to calculate the shifting of the particles.
    /// Can use PPST, XSPH, or both.
    void CalculateShifting(std::shared_ptr<SphMarkerDataD> sortedSphMarkersD);

    int density_initialization;

    // CUDA execution configuration grid
    uint numActive;   ///< total number of threads
    uint numBlocks;   ///< number of blocks
    uint numThreads;  ///< number of threads per block

    bool m_check_errors;
};

/// @} fsisph_physics

}  // namespace sph
}  // end namespace fsi
}  // end namespace chrono

#endif
