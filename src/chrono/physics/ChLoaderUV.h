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

#ifndef CH_LOADER_UV_H
#define CH_LOADER_UV_H

#include "chrono/core/ChApiCE.h"

#include "chrono/physics/ChLoader.h"

namespace chrono {

/// Loaders for ChLoadableUV objects (which support surface loads).
class ChApi ChLoaderUV : public ChLoader {
  public:
    std::shared_ptr<ChLoadableUV> loadable;

    ChLoaderUV(std::shared_ptr<ChLoadableUV> mloadable) : loadable(mloadable) {}
    virtual ~ChLoaderUV() {}

    /// Evaluate F = F(u,v) for this line load.
    /// The vector F is set to zero on entry.
    /// The function provided by derived classes is called by ComputeQ to perform integration over the domain.
    virtual void ComputeF(double U,                    ///< parametric coordinate in surface
                          double V,                    ///< parametric coordinate in surface
                          ChVectorDynamic<>& F,        ///< result vector, size = field dim of loadable
                          ChVectorDynamic<>* state_x,  ///< if != 0, update state (pos. part) to this, then evaluate F
                          ChVectorDynamic<>* state_w   ///< if != 0, update state (speed part) to this, then evaluate F
                          ) = 0;

    void SetLoadable(std::shared_ptr<ChLoadableUV> mloadable) { loadable = mloadable; }
    virtual std::shared_ptr<ChLoadable> GetLoadable() override { return loadable; }
    std::shared_ptr<ChLoadableUV> GetLoadableUV() { return loadable; }
};

//--------------------------------------------------------------------------------

/// Loaders for ChLoadableUV objects (which support surface loads), for loads of distributed type.
/// These loads will undergo Gauss quadrature to integrate them on the surface.
class ChApi ChLoaderUVdistributed : public ChLoaderUV {
  public:
    ChLoaderUVdistributed(std::shared_ptr<ChLoadableUV> mloadable) : ChLoaderUV(mloadable){};
    virtual ~ChLoaderUVdistributed() {}

    virtual int GetIntegrationPointsU() = 0;
    virtual int GetIntegrationPointsV() = 0;

    /// Compute the generalized load Q = integral (N'*F*detJ dudvdz), using the ComputeF method.
    virtual void ComputeQ(ChVectorDynamic<>* state_x,  ///< if != 0, update state (pos. part) to this, then evaluate Q
                          ChVectorDynamic<>* state_w   ///< if != 0, update state (speed part) to this, then evaluate Q
                          ) override;
};

//--------------------------------------------------------------------------------

/// Loaders for ChLoadableUV objects (which support surface loads), for concentrated loads.
class ChApi ChLoaderUVatomic : public ChLoaderUV {
  public:
    ChLoaderUVatomic(std::shared_ptr<ChLoadableUV> mloadable) : ChLoaderUV(mloadable), Pu(0), Pv(0) {}
    virtual ~ChLoaderUVatomic() {}

    /// Compute the generalized load  Q = N'*F, using the ComputeF method.
    virtual void ComputeQ(ChVectorDynamic<>* state_x,  ///< if != 0, update state (pos. part) to this, then evaluate Q
                          ChVectorDynamic<>* state_w   ///< if != 0, update state (speed part) to this, then evaluate Q
                          ) override;

    /// Set the position on the surface where the atomic load is applied.
    void SetApplication(double u, double v);

  private:
    double Pu;
    double Pv;
};

// ===============================================================================

// BASIC UV LOADERS
// Some ready-to use basic loaders

/// Simple surface loader: a constant force vector, applied to a point on a u,v surface.
class ChApi ChLoaderForceOnSurface : public ChLoaderUVatomic {
  public:
    ChLoaderForceOnSurface(std::shared_ptr<ChLoadableUV> mloadable) : ChLoaderUVatomic(mloadable) {}

    virtual void ComputeF(double U,              ///< parametric coordinate in surface
                          double V,              ///< parametric coordinate in surface
                          ChVectorDynamic<>& F,  ///< Result F vector here, size must be = n.field coords.of loadable
                          ChVectorDynamic<>* state_x,  ///< if != 0, update state (pos. part) to this, then evaluate F
                          ChVectorDynamic<>* state_w   ///< if != 0, update state (speed part) to this, then evaluate F
                          ) override {
        F.segment(0, 3) = force.eigen();
    }

    /// Set constant force (assumed in absolute coordinates).
    void SetForce(ChVector3d mforce) { force = mforce; }

    /// Get constant force (assumed in absolute coordinates).
    ChVector3d GetForce() { return force; }

    virtual bool IsStiff() override { return false; }

  private:
    ChVector3d force;
};

//--------------------------------------------------------------------------------

/// Commonly used surface loader: constant pressure load, a 3D per-area force aligned with the surface normal.
class ChApi ChLoaderPressure : public ChLoaderUVdistributed {
  public:
    ChLoaderPressure(std::shared_ptr<ChLoadableUV> mloadable)
        : ChLoaderUVdistributed(mloadable), is_stiff(false), num_integration_points(1) {}

    virtual void ComputeF(double U,              ///< parametric coordinate in surface
                          double V,              ///< parametric coordinate in surface
                          ChVectorDynamic<>& F,  ///< Result F vector here, size must be = n.field coords.of loadable
                          ChVectorDynamic<>* state_x,  ///< if != 0, update state (pos. part) to this, then evaluate F
                          ChVectorDynamic<>* state_w   ///< if != 0, update state (speed part) to this, then evaluate F
                          ) override {
        ChVector3d mnorm = this->loadable->ComputeNormal(U, V);
        F.segment(0, 3) = -pressure * mnorm.eigen();
    }

    void SetPressure(double mpressure) { pressure = mpressure; }
    double GetPressure() { return pressure; }

    void SetIntegrationPoints(int val) { num_integration_points = val; }
    virtual int GetIntegrationPointsU() override { return num_integration_points; }
    virtual int GetIntegrationPointsV() override { return num_integration_points; }

    void SetStiff(bool val) { is_stiff = val; }
    virtual bool IsStiff() override { return is_stiff; }

  private:
    double pressure;
    bool is_stiff;
    int num_integration_points;
};

}  // end namespace chrono

#endif
