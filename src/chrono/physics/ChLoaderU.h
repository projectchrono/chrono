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

#ifndef CHLOADERU_H
#define CHLOADERU_H

#include "chrono/physics/ChLoader.h"

namespace chrono {

/// Class of loaders for ChLoadableU objects (which support line loads).

class ChLoaderU : public ChLoader {
  public:
    typedef ChLoadableU type_loadable;

    std::shared_ptr<ChLoadableU> loadable;

    ChLoaderU(std::shared_ptr<ChLoadableU> mloadable) : loadable(mloadable) {}
    virtual ~ChLoaderU() {}

    /// Children classes must provide this function that evaluates F = F(u)
    /// This will be evaluated during ComputeQ() to perform integration over the domain.
    virtual void ComputeF(const double U,        ///< parametric coordinate in line
                          ChVectorDynamic<>& F,  ///< Result F vector here, size must be = n.field coords.of loadable
                          ChVectorDynamic<>* state_x,  ///< if != 0, update state (pos. part) to this, then evaluate F
                          ChVectorDynamic<>* state_w   ///< if != 0, update state (speed part) to this, then evaluate F
                          ) = 0;

    void SetLoadable(std::shared_ptr<ChLoadableU> mloadable) { loadable = mloadable; }
    virtual std::shared_ptr<ChLoadable> GetLoadable() override { return loadable; }
    std::shared_ptr<ChLoadableU> GetLoadableU() { return loadable; }
};

/// Class of loaders for ChLoadableU objects (which support
/// line loads), for loads of distributed type, so these loads
/// will undergo Gauss quadrature to integrate them in the surface.

class ChLoaderUdistributed : public ChLoaderU {
  public:
    ChLoaderUdistributed(std::shared_ptr<ChLoadableU> mloadable) : ChLoaderU(mloadable) {}
    virtual ~ChLoaderUdistributed() {}

    virtual int GetIntegrationPointsU() = 0;

    /// Computes Q = integral (N'*F*detJ du)
    virtual void ComputeQ(ChVectorDynamic<>* state_x,  ///< if != 0, update state (pos. part) to this, then evaluate Q
                          ChVectorDynamic<>* state_w   ///< if != 0, update state (speed part) to this, then evaluate Q
                          ) override {
        assert(GetIntegrationPointsU() <= ChQuadrature::GetStaticTables()->Lroots.size());

        Q.Reset(loadable->LoadableGet_ndof_w());
        ChVectorDynamic<> mF(loadable->Get_field_ncoords());

        std::vector<double>* Ulroots = &ChQuadrature::GetStaticTables()->Lroots[GetIntegrationPointsU() - 1];
        std::vector<double>* Uweight = &ChQuadrature::GetStaticTables()->Weight[GetIntegrationPointsU() - 1];

        ChVectorDynamic<> mNF(Q.GetRows());  // temporary value for loop

        // Gauss quadrature :  Q = sum (N'*F*detJ * wi)
        for (unsigned int iu = 0; iu < Ulroots->size(); iu++) {
            double detJ;
            // Compute F= F(u)
            this->ComputeF(Ulroots->at(iu), mF, state_x, state_w);
            // Compute mNF= N(u)'*F
            loadable->ComputeNF(Ulroots->at(iu), mNF, detJ, mF, state_x, state_w);
            // Compute Q+= mNF detJ * wi
            mNF *= (detJ * Uweight->at(iu));
            Q += mNF;
        }
    }
};

/// Class of loaders for ChLoadableU objects (which support
/// line loads) of atomic type, that is, with a concentrated load in a point Pu

class ChLoaderUatomic : public ChLoaderU {
  public:
    double Pu;

    ChLoaderUatomic(std::shared_ptr<ChLoadableU> mloadable) : ChLoaderU(mloadable) {}

    /// Computes Q = N'*F
    virtual void ComputeQ(ChVectorDynamic<>* state_x,  ///< if != 0, update state (pos. part) to this, then evaluate Q
                          ChVectorDynamic<>* state_w   ///< if != 0, update state (speed part) to this, then evaluate Q
                          ) override {
        Q.Reset(loadable->LoadableGet_ndof_w());
        ChVectorDynamic<> mF(loadable->Get_field_ncoords());

        double detJ;  // not used btw

        // Compute F=F(u)
        this->ComputeF(Pu, mF, state_x, state_w);

        // Compute N(u)'*F
        loadable->ComputeNF(Pu, Q, detJ, mF, state_x, state_w);
    }

    /// Set the position, on the surface where the atomic load is applied
    void SetApplication(double mu) { Pu = mu; }
};

}  // end namespace chrono

#endif
