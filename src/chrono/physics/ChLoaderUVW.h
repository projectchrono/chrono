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

#ifndef CHLOADERUVW_H
#define CHLOADERUVW_H

#include "chrono/physics/ChLoader.h"

namespace chrono {

/// Class of loaders for ChLoadableUVW objects (which support volume loads).

class ChLoaderUVW : public ChLoader {
  public:
    typedef ChLoadableUVW type_loadable;

    std::shared_ptr<ChLoadableUVW> loadable;

    ChLoaderUVW(std::shared_ptr<ChLoadableUVW> mloadable) : loadable(mloadable) {}
    virtual ~ChLoaderUVW() {}

    /// Children classes must provide this function that evaluates F = F(u,v,w)
    /// This will be evaluated during ComputeQ() to perform integration over the domain.
    virtual void ComputeF(const double U,        ///< parametric coordinate in volume
                          const double V,        ///< parametric coordinate in volume
                          const double W,        ///< parametric coordinate in volume
                          ChVectorDynamic<>& F,  ///< Result F vector here, size must be = n.field coords.of loadable
                          ChVectorDynamic<>* state_x,  ///< if != 0, update state (pos. part) to this, then evaluate F
                          ChVectorDynamic<>* state_w   ///< if != 0, update state (speed part) to this, then evaluate F
                          ) = 0;

    void SetLoadable(std::shared_ptr<ChLoadableUVW> mloadable) { loadable = mloadable; }
    virtual std::shared_ptr<ChLoadable> GetLoadable() override { return loadable; }
    std::shared_ptr<ChLoadableUVW> GetLoadableUVW() { return loadable; }
};

/// Class of loaders for ChLoadableUVW objects (which support volume loads), for loads of distributed type,
/// so these loads will undergo Gauss quadrature to integrate them in the volume.

class ChLoaderUVWdistributed : public ChLoaderUVW {
  public:
    ChLoaderUVWdistributed(std::shared_ptr<ChLoadableUVW> mloadable) : ChLoaderUVW(mloadable) {}
    virtual ~ChLoaderUVWdistributed() {}

    virtual int GetIntegrationPointsU() = 0;
    virtual int GetIntegrationPointsV() = 0;
    virtual int GetIntegrationPointsW() = 0;

    /// Computes Q = integral (N'*F*detJ dudvdz)
    virtual void ComputeQ(ChVectorDynamic<>* state_x,  ///< if != 0, update state (pos. part) to this, then evaluate Q
                          ChVectorDynamic<>* state_w   ///< if != 0, update state (speed part) to this, then evaluate Q
                          ) override {
        Q.Reset(loadable->LoadableGet_ndof_w());
        ChVectorDynamic<> mF(loadable->Get_field_ncoords());
        mF.Reset();

        if (!loadable->IsTetrahedronIntegrationNeeded()) {
            // Case of normal box isoparametric coords
            assert(GetIntegrationPointsU() <= ChQuadrature::GetStaticTables()->Lroots.size());
            assert(GetIntegrationPointsV() <= ChQuadrature::GetStaticTables()->Lroots.size());
            assert(GetIntegrationPointsW() <= ChQuadrature::GetStaticTables()->Lroots.size());
            const std::vector<double>& Ulroots = ChQuadrature::GetStaticTables()->Lroots[GetIntegrationPointsU() - 1];
            const std::vector<double>& Uweight = ChQuadrature::GetStaticTables()->Weight[GetIntegrationPointsU() - 1];
            const std::vector<double>& Vlroots = ChQuadrature::GetStaticTables()->Lroots[GetIntegrationPointsV() - 1];
            const std::vector<double>& Vweight = ChQuadrature::GetStaticTables()->Weight[GetIntegrationPointsV() - 1];
            const std::vector<double>& Wlroots = ChQuadrature::GetStaticTables()->Lroots[GetIntegrationPointsW() - 1];
            const std::vector<double>& Wweight = ChQuadrature::GetStaticTables()->Weight[GetIntegrationPointsW() - 1];

            ChVectorDynamic<> mNF(Q.GetRows());  // temporary value for loop

            // Gauss quadrature :  Q = sum (N'*F*detJ * wi*wj*wk)
            for (unsigned int iu = 0; iu < Ulroots.size(); iu++) {
                for (unsigned int iv = 0; iv < Vlroots.size(); iv++) {
                    for (unsigned int iw = 0; iw < Wlroots.size(); iw++) {
                        double detJ;
                        // Compute F= F(u,v,w)
                        this->ComputeF(Ulroots[iu], Vlroots[iv], Wlroots[iw], mF, state_x, state_w);
                        // Compute mNF= N(u,v,w)'*F
                        loadable->ComputeNF(Ulroots[iu], Vlroots[iv], Wlroots[iw], mNF, detJ, mF, state_x,
                                            state_w);
                        // Compute Q+= mNF detJ * wi*wj*wk
                        mNF *= (detJ * Uweight[iu] * Vweight[iv] * Wweight[iw]);
                        Q += mNF;
                    }
                }
            }
        } else {
            // case of tetrahedron: use special 3d quadrature tables (given U,V,W orders, use the U only)
            assert(GetIntegrationPointsU() <= ChQuadrature::GetStaticTablesTetrahedron()->Weight.size());
            const std::vector<double>& Ulroots = ChQuadrature::GetStaticTablesTetrahedron()->LrootsU[GetIntegrationPointsU() - 1];
            const std::vector<double>& Vlroots = ChQuadrature::GetStaticTablesTetrahedron()->LrootsV[GetIntegrationPointsU() - 1];
            const std::vector<double>& Wlroots = ChQuadrature::GetStaticTablesTetrahedron()->LrootsW[GetIntegrationPointsU() - 1];
            const std::vector<double>& weight =  ChQuadrature::GetStaticTablesTetrahedron()->Weight[GetIntegrationPointsU() - 1];

            ChVectorDynamic<> mNF(Q.GetRows());  // temporary value for loop

            // Gauss quadrature :  Q = sum (N'*F*detJ * wi * 1/6)   often detJ=6*tetrahedron volume
            for (unsigned int i = 0; i < Ulroots.size(); i++) {
                double detJ;
                // Compute F= F(u,v,w)
                this->ComputeF(Ulroots[i], Vlroots[i], Wlroots[i], mF, state_x, state_w);
                // Compute mNF= N(u,v,w)'*F
                loadable->ComputeNF(Ulroots[i], Vlroots[i], Wlroots[i], mNF, detJ, mF, state_x, state_w);
                // Compute Q+= mNF detJ * wi * 1/6
                mNF *= (detJ * weight[i] * (1. / 6.));  // (the 1/6 coefficient is not in the table);
                Q += mNF;
            }
        }
    }
};

/// Class of loaders for ChLoadableUVW objects (which support volume loads) of atomic type,
/// that is, with a concentrated load in a point Pu,Pv,Pz.

class ChLoaderUVWatomic : public ChLoaderUVW {
  public:
    double Pu;
    double Pv;
    double Pw;

    ChLoaderUVWatomic(std::shared_ptr<ChLoadableUVW> mloadable, double mU, double mV, double mW)
        : ChLoaderUVW(mloadable), Pu(mU), Pv(mV), Pw(mW) {}
    virtual ~ChLoaderUVWatomic() {}

    /// Computes Q = N'*F
    virtual void ComputeQ(ChVectorDynamic<>* state_x,  ///< if != 0, update state (pos. part) to this, then evaluate Q
                          ChVectorDynamic<>* state_w   ///< if != 0, update state (speed part) to this, then evaluate Q
                          ) override {
        Q.Reset(loadable->LoadableGet_ndof_w());
        ChVectorDynamic<> mF(loadable->Get_field_ncoords());
        mF.Reset();

        // Compute F=F(u,v,w)
        this->ComputeF(Pu, Pv, Pw, mF, state_x, state_w);

        // Compute N(u,v,w)'*F
        double detJ;
        loadable->ComputeNF(Pu, Pv, Pw, Q, detJ, mF, state_x, state_w);
    }

    /// Set the position, in the volume, where the atomic load is applied
    void SetApplication(double mu, double mv, double mw) {
        Pu = mu;
        Pv = mv;
        Pw = mw;
    }
};

//--------------------------------------------------------------------------------
// BASIC UVW LOADERS
//
// Some ready-to use basic loaders

/// A very usual type of volume loader: the constant gravitational load on Y

class ChLoaderGravity : public ChLoaderUVWdistributed {
  private:
    ChVector<> G_acc;
    int num_int_points;

  public:
    ChLoaderGravity(std::shared_ptr<ChLoadableUVW> mloadable)
        : ChLoaderUVWdistributed(mloadable), G_acc(0, -9.8, 0), num_int_points(1){};

    virtual void ComputeF(const double U,        ///< parametric coordinate in volume
                          const double V,        ///< parametric coordinate in volume
                          const double W,        ///< parametric coordinate in volume
                          ChVectorDynamic<>& F,  ///< Result F vector here, size must be = n.field coords.of loadable
                          ChVectorDynamic<>* state_x,  ///< if != 0, update state (pos. part) to this, then evaluate F
                          ChVectorDynamic<>* state_w   ///< if != 0, update state (speed part) to this, then evaluate F
                          ) override {
        if ((F.GetRows() == 3) || (F.GetRows() == 6) || (F.GetRows() == 9)) {
            // only for force or wrench fields
            F(0) = G_acc.x() * loadable->GetDensity();
            F(1) = G_acc.y() * loadable->GetDensity();
            F(2) = G_acc.z() * loadable->GetDensity();
        }
    }
    /// Sets the number of integration points for gravity (assumed, for now, same number per direction)
    void SetNumIntPoints(int val) { num_int_points = val; }
    /// Gets the number of integration points for gravity
    int GetNumIntPoints() const { return num_int_points; }
    /// Sets the G (gravity) acceleration vector affecting the loadable object
    void Set_G_acc(ChVector<> m_acc) { G_acc = m_acc; }
    /// Gets the G (gravity) acceleration vector affecting the loadable object
    ChVector<> Get_G_acc() { return G_acc; }

    virtual int GetIntegrationPointsU() override { return num_int_points; }
    virtual int GetIntegrationPointsV() override { return num_int_points; }
    virtual int GetIntegrationPointsW() override { return num_int_points; }
};

}  // end namespace chrono

#endif
