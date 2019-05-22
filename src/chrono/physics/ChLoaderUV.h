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

#ifndef CHLOADERUV_H
#define CHLOADERUV_H

#include "chrono/physics/ChLoader.h"

namespace chrono {

/// Class of loaders for ChLoadableUV objects (which support surface loads).

class ChLoaderUV : public ChLoader {
  public:
    typedef ChLoadableUV type_loadable;

    std::shared_ptr<ChLoadableUV> loadable;

    ChLoaderUV(std::shared_ptr<ChLoadableUV> mloadable) : loadable(mloadable) {}
    virtual ~ChLoaderUV() {}

    /// Children classes must provide this function that evaluates F = F(u,v)
    /// This will be evaluated during ComputeQ() to perform integration over the domain.
    virtual void ComputeF(const double U,        ///< parametric coordinate in surface
                          const double V,        ///< parametric coordinate in surface
                          ChVectorDynamic<>& F,  ///< Result F vector here, size must be = n.field coords.of loadable
                          ChVectorDynamic<>* state_x,  ///< if != 0, update state (pos. part) to this, then evaluate F
                          ChVectorDynamic<>* state_w   ///< if != 0, update state (speed part) to this, then evaluate F
                          ) = 0;

    void SetLoadable(std::shared_ptr<ChLoadableUV> mloadable) { loadable = mloadable; }
    virtual std::shared_ptr<ChLoadable> GetLoadable() override { return loadable; }
    std::shared_ptr<ChLoadableUV> GetLoadableUV() { return loadable; }
};

/// Class of loaders for ChLoadableUV objects (which support surface loads), for loads of distributed type,
/// so these loads will undergo Gauss quadrature to integrate them in the surface.

class ChLoaderUVdistributed : public ChLoaderUV {
  public:
    ChLoaderUVdistributed(std::shared_ptr<ChLoadableUV> mloadable) : ChLoaderUV(mloadable){};
    virtual ~ChLoaderUVdistributed() {}

    virtual int GetIntegrationPointsU() = 0;
    virtual int GetIntegrationPointsV() = 0;

    /// Computes Q = integral (N'*F*detJ dudvdz)
    virtual void ComputeQ(ChVectorDynamic<>* state_x,  ///< if != 0, update state (pos. part) to this, then evaluate Q
                          ChVectorDynamic<>* state_w   ///< if != 0, update state (speed part) to this, then evaluate Q
                          ) override {
        Q.Reset(loadable->LoadableGet_ndof_w());
        ChVectorDynamic<> mF(loadable->Get_field_ncoords());
        mF.Reset();

        if (!loadable->IsTriangleIntegrationNeeded()) {
            // Case of normal quadrilateral isoparametric coords
            assert(GetIntegrationPointsU() <= ChQuadrature::GetStaticTables()->Weight.size());
            assert(GetIntegrationPointsV() <= ChQuadrature::GetStaticTables()->Weight.size());
            const std::vector<double>& Ulroots = ChQuadrature::GetStaticTables()->Lroots[GetIntegrationPointsU() - 1];
            const std::vector<double>& Uweight = ChQuadrature::GetStaticTables()->Weight[GetIntegrationPointsU() - 1];
            const std::vector<double>& Vlroots = ChQuadrature::GetStaticTables()->Lroots[GetIntegrationPointsV() - 1];
            const std::vector<double>& Vweight = ChQuadrature::GetStaticTables()->Weight[GetIntegrationPointsV() - 1];

            ChVectorDynamic<> mNF(Q.GetRows());  // temporary value for loop

            // Gauss quadrature :  Q = sum (N'*F*detJ * wi*wj)
            for (unsigned int iu = 0; iu < Ulroots.size(); iu++) {
                for (unsigned int iv = 0; iv < Vlroots.size(); iv++) {
                    double detJ;
                    // Compute F= F(u,v)
                    this->ComputeF(Ulroots[iu], Vlroots[iv], mF, state_x, state_w);
                    // Compute mNF= N(u,v)'*F
                    loadable->ComputeNF(Ulroots[iu], Vlroots[iv], mNF, detJ, mF, state_x, state_w);
                    // Compute Q+= mNF detJ * wi*wj
                    mNF *= (detJ * Uweight[iu] * Vweight[iv]);
                    Q += mNF;
                }
            }
        } else {
            // case of triangle: use special 3d quadrature tables (given U,V,W orders, use the U only)
            assert(GetIntegrationPointsU() <= ChQuadrature::GetStaticTablesTriangle()->Weight.size());
            const std::vector<double>& Ulroots = ChQuadrature::GetStaticTablesTriangle()->LrootsU[GetIntegrationPointsU() - 1];
            const std::vector<double>& Vlroots = ChQuadrature::GetStaticTablesTriangle()->LrootsV[GetIntegrationPointsU() - 1];
            const std::vector<double>& weight = ChQuadrature::GetStaticTablesTriangle()->Weight[GetIntegrationPointsU() - 1];

            ChVectorDynamic<> mNF(Q.GetRows());  // temporary value for loop

            // Gauss quadrature :  Q = sum (N'*F*detJ * wi *1/2)   often detJ= 2 * triangle area
            for (unsigned int i = 0; i < Ulroots.size(); i++) {
                double detJ;
                // Compute F= F(u,v)
                this->ComputeF(Ulroots[i], Vlroots[i], mF, state_x, state_w);
                // Compute mNF= N(u,v)'*F
                loadable->ComputeNF(Ulroots[i], Vlroots[i], mNF, detJ, mF, state_x, state_w);
                // Compute Q+= mNF detJ * wi *1/2
                mNF *= (detJ * weight[i] * (1. / 2.));  // (the 1/2 coefficient is not in the table);
                Q += mNF;
            }
        }
    }
};

/// Class of loaders for ChLoadableUV objects (which support surface loads) of atomic type,
/// that is, with a concentrated load in a point Pu,Pv.

class ChLoaderUVatomic : public ChLoaderUV {
  public:
    double Pu;
    double Pv;

    ChLoaderUVatomic(std::shared_ptr<ChLoadableUV> mloadable) : ChLoaderUV(mloadable), Pu(0), Pv(0) {}
    virtual ~ChLoaderUVatomic() {}

    /// Computes Q = N'*F
    virtual void ComputeQ(ChVectorDynamic<>* state_x,  ///< if != 0, update state (pos. part) to this, then evaluate Q
                          ChVectorDynamic<>* state_w   ///< if != 0, update state (speed part) to this, then evaluate Q
                          ) override {
        Q.Reset(loadable->LoadableGet_ndof_w());
        ChVectorDynamic<> mF(loadable->Get_field_ncoords());
        mF.Reset();

        // Compute F=F(u,v)
        this->ComputeF(Pu, Pv, mF, state_x, state_w);

        // Compute N(u,v)'*F
        double detJ;
        loadable->ComputeNF(Pu, Pv, Q, detJ, mF, state_x, state_w);
    }

    /// Set the position, on the surface where the atomic load is applied
    void SetApplication(double mu, double mv) {
        Pu = mu;
        Pv = mv;
    }
};

//--------------------------------------------------------------------------------
// BASIC UV LOADERS
//
// Some ready-to use basic loaders

/// A very simple surface loader: a constant force vector, applied to a point on a u,v surface

class ChLoaderForceOnSurface : public ChLoaderUVatomic {
  private:
    ChVector<> force;

  public:
    ChLoaderForceOnSurface(std::shared_ptr<ChLoadableUV> mloadable) : ChLoaderUVatomic(mloadable) {}

    virtual void ComputeF(const double U,        ///< parametric coordinate in surface
                          const double V,        ///< parametric coordinate in surface
                          ChVectorDynamic<>& F,  ///< Result F vector here, size must be = n.field coords.of loadable
                          ChVectorDynamic<>* state_x,  ///< if != 0, update state (pos. part) to this, then evaluate F
                          ChVectorDynamic<>* state_w   ///< if != 0, update state (speed part) to this, then evaluate F
                          ) override {
        F.PasteVector(force, 0, 0);
    }
    // Set constant force (assumed in absolute coordinates)
    void SetForce(ChVector<> mforce) { force = mforce; }
    // Get constant force (assumed in absolute coordinates)
    ChVector<> GetForce() { return force; }

    virtual bool IsStiff() override { return false; }
};

/// A very usual type of surface loader: the constant pressure load, a 3D per-area force that is aligned to the surface normal.

class ChLoaderPressure : public ChLoaderUVdistributed {
  private:
    double pressure;
    bool is_stiff;
    int num_integration_points;

  public:
    ChLoaderPressure(std::shared_ptr<ChLoadableUV> mloadable)
        : ChLoaderUVdistributed(mloadable), is_stiff(false), num_integration_points(1) {}

    virtual void ComputeF(const double U,        ///< parametric coordinate in surface
                          const double V,        ///< parametric coordinate in surface
                          ChVectorDynamic<>& F,  ///< Result F vector here, size must be = n.field coords.of loadable
                          ChVectorDynamic<>* state_x,  ///< if != 0, update state (pos. part) to this, then evaluate F
                          ChVectorDynamic<>* state_w   ///< if != 0, update state (speed part) to this, then evaluate F
                          ) override {
        ChVector<> mnorm = this->loadable->ComputeNormal(U, V);
        F.PasteVector(mnorm * (-pressure), 0, 0);
    }

    void SetPressure(double mpressure) { pressure = mpressure; }
    double GetPressure() { return pressure; }

    void SetIntegrationPoints(int val) { num_integration_points = val; }
    virtual int GetIntegrationPointsU() override { return num_integration_points; }
    virtual int GetIntegrationPointsV() override { return num_integration_points; }

    void SetStiff(bool val) { is_stiff = val; }
    virtual bool IsStiff() override { return is_stiff; }
};

}  // end namespace chrono

#endif
