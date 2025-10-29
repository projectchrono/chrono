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

#ifndef CH_LOADER_UVW_H
#define CH_LOADER_UVW_H

#include "chrono/core/ChApiCE.h"

#include "chrono/physics/ChLoader.h"

namespace chrono {

/// Loaders for ChLoadableUVW objects (which support volume loads).
class ChApi ChLoaderUVW : public ChLoader {
  public:
    std::shared_ptr<ChLoadableUVW> loadable;

    ChLoaderUVW(std::shared_ptr<ChLoadableUVW> mloadable) : loadable(mloadable) {}
    virtual ~ChLoaderUVW() {}

    /// Evaluate F = F(u,v,w) for this line load.
    /// The vector F is set to zero on entry.
    /// The function provided by derived classes is called by ComputeQ to perform integration over the domain.
    virtual void ComputeF(double U,                    ///< parametric coordinate in volume
                          double V,                    ///< parametric coordinate in volume
                          double W,                    ///< parametric coordinate in volume
                          ChVectorDynamic<>& F,        ///< result vector, size = field dim of loadable
                          ChVectorDynamic<>* state_x,  ///< if != 0, update state (pos. part) to this, then evaluate F
                          ChVectorDynamic<>* state_w   ///< if != 0, update state (speed part) to this, then evaluate F
                          ) = 0;

    void SetLoadable(std::shared_ptr<ChLoadableUVW> mloadable) { loadable = mloadable; }
    virtual std::shared_ptr<ChLoadable> GetLoadable() override { return loadable; }
    std::shared_ptr<ChLoadableUVW> GetLoadableUVW() { return loadable; }
};

//--------------------------------------------------------------------------------

/// Loaders for ChLoadableUVW objects (which support volume loads), for loads of distributed type.
/// These loads will undergo Gauss quadrature to integrate them in the volume.
class ChApi ChLoaderUVWdistributed : public ChLoaderUVW {
  public:
    ChLoaderUVWdistributed(std::shared_ptr<ChLoadableUVW> mloadable) : ChLoaderUVW(mloadable) {}
    virtual ~ChLoaderUVWdistributed() {}

    virtual int GetIntegrationPointsU() = 0;
    virtual int GetIntegrationPointsV() = 0;
    virtual int GetIntegrationPointsW() = 0;

    /// Compute the generalized load  Q = integral (N'*F*detJ dudvdz), using the ComputeF method.
    virtual void ComputeQ(ChVectorDynamic<>* state_x,  ///< if != 0, update state (pos. part) to this, then evaluate Q
                          ChVectorDynamic<>* state_w   ///< if != 0, update state (speed part) to this, then evaluate Q
                          ) override;
};

//--------------------------------------------------------------------------------

/// Loaders for ChLoadableUVW objects (which support volume loads), for concentrated loads.
class ChApi ChLoaderUVWatomic : public ChLoaderUVW {
  public:
    ChLoaderUVWatomic(std::shared_ptr<ChLoadableUVW> mloadable, double mU, double mV, double mW)
        : ChLoaderUVW(mloadable), Pu(mU), Pv(mV), Pw(mW) {}
    virtual ~ChLoaderUVWatomic() {}

    /// Compute the generalized load  Q = N'*F, using the ComputeF method.
    virtual void ComputeQ(ChVectorDynamic<>* state_x,  ///< if != 0, update state (pos. part) to this, then evaluate Q
                          ChVectorDynamic<>* state_w   ///< if != 0, update state (speed part) to this, then evaluate Q
                          ) override;

    /// Set the position, in the volume, where the atomic load is applied
    void SetApplication(double u, double v, double w);

  private:
    double Pu;
    double Pv;
    double Pw;
};




}  // end namespace chrono

#endif
