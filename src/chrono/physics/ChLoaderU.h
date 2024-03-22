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

#ifndef CH_LOADER_U_H
#define CH_LOADER_U_H

#include "chrono/core/ChApiCE.h"

#include "chrono/physics/ChLoader.h"

namespace chrono {

/// Loaders for ChLoadableU objects (which support line loads).
class ChApi ChLoaderU : public ChLoader {
  public:
    std::shared_ptr<ChLoadableU> loadable;

    ChLoaderU(std::shared_ptr<ChLoadableU> mloadable) : loadable(mloadable) {}
    virtual ~ChLoaderU() {}

    /// Evaluate F = F(u) for this line load.
    /// The vector F is set to zero on entry.
    /// The function provided by derived classes is called by ComputeQ to perform integration over the domain.
    virtual void ComputeF(double U,                    ///< parametric coordinate in line
                          ChVectorDynamic<>& F,        ///< result vector, size = field dim of loadable
                          ChVectorDynamic<>* state_x,  ///< if != 0, update state (pos. part) to this, then evaluate F
                          ChVectorDynamic<>* state_w   ///< if != 0, update state (speed part) to this, then evaluate F
                          ) = 0;

    void SetLoadable(std::shared_ptr<ChLoadableU> mloadable) { loadable = mloadable; }
    virtual std::shared_ptr<ChLoadable> GetLoadable() override { return loadable; }
    std::shared_ptr<ChLoadableU> GetLoadableU() { return loadable; }
};

//--------------------------------------------------------------------------------

/// Loader for ChLoadableU objects (which support line loads), for loads of distributed type.
/// These loads will undergo Gauss quadrature to integrate them on the line.
class ChApi ChLoaderUdistributed : public ChLoaderU {
  public:
    ChLoaderUdistributed(std::shared_ptr<ChLoadableU> mloadable) : ChLoaderU(mloadable) {}
    virtual ~ChLoaderUdistributed() {}

    virtual int GetIntegrationPointsU() = 0;

    /// Compute the generalized load Q = integral (N'*F*detJ du), using the ComputeF method.
    virtual void ComputeQ(ChVectorDynamic<>* state_x,  ///< if != 0, update state (pos. part) to this, then evaluate Q
                          ChVectorDynamic<>* state_w   ///< if != 0, update state (speed part) to this, then evaluate Q
                          ) override;
};

//--------------------------------------------------------------------------------

/// Loader for ChLoadableU objects (which support line loads), for concentrated loads.
class ChApi ChLoaderUatomic : public ChLoaderU {
  public:
    ChLoaderUatomic(std::shared_ptr<ChLoadableU> mloadable) : ChLoaderU(mloadable), Pu(0) {}
    virtual ~ChLoaderUatomic() {}

    /// Compute the generalized load Q = N'*F, using the ComputeF method.
    virtual void ComputeQ(ChVectorDynamic<>* state_x,  ///< if != 0, update state (pos. part) to this, then evaluate Q
                          ChVectorDynamic<>* state_w   ///< if != 0, update state (speed part) to this, then evaluate Q
                          ) override;

    /// Set the position, on the surface where the atomic load is applied
    void SetApplication(double u) { Pu = u; }

  private:
    double Pu;
};

}  // end namespace chrono

#endif
