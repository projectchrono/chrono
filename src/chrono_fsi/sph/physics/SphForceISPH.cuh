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
// Author: Milad Rakhsha, Radu Serban
// =============================================================================

#ifndef CH_SPH_FORCE_ISPH_H
#define CH_SPH_FORCE_ISPH_H

#include "chrono_fsi/sph/physics/SphForce.cuh"
#include "chrono_fsi/sph/math/SphLinearSolver.h"

namespace chrono {
namespace fsi {
namespace sph {

/// @addtogroup fsisph_physics
/// @{

/// Inter-particle force calculation for the implicit SPH method.
class SphForceISPH : public SphForce {
  public:
    /// Force class implemented using incompressible SPH method with implicit integrator.
    SphForceISPH(FsiDataManager& data_mgr,  ///< FSI data manager
                 SphBceManager& bce_mgr,       ///< BCE manager
                 bool verbose,              ///< verbose output
                 bool check_errors          ///< check errors
    );

    ~SphForceISPH();

    virtual void Initialize() override;

  private:
    std::shared_ptr<LinearSolver> myLinearSolver;

    thrust::device_vector<Real> _sumWij_inv;
    thrust::device_vector<Real> G_i;
    thrust::device_vector<Real> A_i;
    thrust::device_vector<Real> L_i;
    thrust::device_vector<Real> csrValLaplacian;
    thrust::device_vector<Real3> csrValGradient;
    thrust::device_vector<Real> csrValFunction;
    thrust::device_vector<Real> AMatrix;
    thrust::device_vector<Real3> Normals;
    thrust::device_vector<Real3> V_star_new;
    thrust::device_vector<Real3> V_star_old;
    thrust::device_vector<Real> q_new;
    thrust::device_vector<Real> q_old;
    thrust::device_vector<Real> b1Vector;
    thrust::device_vector<Real3> b3Vector;
    thrust::device_vector<Real> Residuals;

    size_t numAllMarkers;
    size_t NNZ;

    bool m_check_errors;

    void ForceSPH(std::shared_ptr<SphMarkerDataD> sortedSphMarkersD, Real time, Real step) override;

    void PreProcessor(std::shared_ptr<SphMarkerDataD> sortedSphMarkersD, bool calcLaplacianOperator);
};

/// @} fsisph_physics

}  // namespace sph
}  // end namespace fsi
}  // end namespace chrono

#endif
