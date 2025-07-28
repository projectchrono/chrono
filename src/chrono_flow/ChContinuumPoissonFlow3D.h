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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#ifndef CHCONTINUUMPOISSONFLOW3D_H
#define CHCONTINUUMPOISSONFLOW3D_H

#include "chrono/fea/ChContinuumMaterial.h" 
#include "chrono_flow/ChFlowApi.h"

using namespace chrono::fea;

namespace chrono {
namespace flow {

/// @addtogroup chrono_fea
/// @{

/// Class for the basic properties of scalar fields P in 3D FEM problems
/// that can be described by Laplace PDEs of type
///    rho dP/dt + div [C] grad P = 0
class ChFlowApi ChContinuumPoissonFlow3D : public ChContinuumMaterial {
  protected:
    ChMatrixDynamic<> ConstitutiveMatrix;  // constitutive matrix

    double W_mct = 0;
    double A_1c = 0;
    double A_2c = 0;
    double eta_c = 0;
    double a_fi = 0;
    double b_fi = 0;
    double Eac_over_R = 0;
    double alpha_infinity = 0;
    double rho = 0;
    double CEMENT = 0;
    double SILICA = 0;
    double AGGREGATE = 0;
    double k_c = 0; 
    double g_1 = 0; 
    double k_vg_c = 0; 
    double k_vg_s = 0; 
    double Q_over_R = 0;
    double ct = 0; 
    double Lampda = 0; 
    double Q_hydr_inf = 0; 
    double Q_S_inf = 0; 
    double m_c = 0;



  public:
    ChContinuumPoissonFlow3D() { ConstitutiveMatrix.setIdentity(3, 3); }
    ChContinuumPoissonFlow3D(const ChContinuumPoissonFlow3D& other) : ChContinuumMaterial(other) {
        ConstitutiveMatrix = other.ConstitutiveMatrix;
    }
    virtual ~ChContinuumPoissonFlow3D() {}

    /// Get the constitutive matrix [C] to compute the bilinear form in the weak formulation
    ChMatrixDynamic<>& GetConstitutiveMatrix() { return ConstitutiveMatrix; }

    /// Get the rho multiplier for the 'rho dP/dt term', if any (default, none)
    virtual double Get_DtMultiplier() { return 0; }

    virtual ChMatrixDynamic<> ComputeUpdatedConstitutiveMatrixK(ChVectorDynamic<>& NVal, ChVectorDynamic<>& NValDt) { 
      ChMatrixDynamic<> IdentityMatrix;
      IdentityMatrix.setIdentity(3, 3);
      return IdentityMatrix; 
    }

    // virtual ChMatrixDynamic<> ComputeUpdatedConstitutiveMatrixM(ChVectorDynamic<>& NVal, ChVectorDynamic<>& NValDt) { 
    //   ChMatrixDynamic<> IdentityMatrix;
    //   IdentityMatrix.setIdentity(3, 3);
    //   return IdentityMatrix; 
    // }

    virtual ChMatrixDynamic<> ComputeUpdatedConstitutiveMatrixM(ChVectorDynamic<>& NVal, ChVectorDynamic<>& NValDt, double m_cap) { 
      ChMatrixDynamic<> IdentityMatrix;
      IdentityMatrix.setIdentity(3, 3);
      return IdentityMatrix; 
    }

    // virtual ChMatrixDynamic<> ComputeUpdatedConstitutiveVectorS(ChVectorDynamic<>& NVal, ChVectorDynamic<>& NValDt) { 
    //   ChMatrixDynamic<> IdentityMatrix;
    //   IdentityMatrix.setIdentity(3, 3);
    //   return IdentityMatrix; 
    // }

    virtual ChMatrixDynamic<> ComputeUpdatedConstitutiveVectorS(ChVectorDynamic<>& NVal, ChVectorDynamic<>& NValDt, double dalpha_dt) { 
      ChMatrixDynamic<> IdentityMatrix;
      IdentityMatrix.setIdentity(3, 3);
      return IdentityMatrix; 
    }

    // private:
    // ChVectorDynamic<> ElementState; 

    // public:

    // void SetElementStateVariable(ChVectorDynamic<>& ElState) { ElementState = ElState; }
    // const ChVectorDynamic<>& GetElementStateVariable() { return ElementState; }

    

    void SetHydrationParameters(double wct, double A1, double A2, double eta, double af, double bf, double Eac, double alphainf) {
        W_mct = wct;
        A_1c = A1;
        A_2c = A2;
        eta_c = eta;
        a_fi = af;
        b_fi = bf;
        Eac_over_R = Eac;
        alpha_infinity = alphainf;
        
    }

    double Getwmct() const { return W_mct; }
    double GetA1c() const { return A_1c; }
    double GetA2c() const { return A_2c; }
    double GetEtac() const { return eta_c; }
    double GetAf() const { return a_fi; }
    double GetBf() const { return b_fi; }
    double GetEacOverR() const { return Eac_over_R; }
    double GetAlphaInfinity() const { return alpha_infinity; }

    void SetHeatParameters(double ct1, double Lampda1, double Q_hydr_inf1, double Q_S_inf1) {       
        
       ct = ct1;
       Lampda = Lampda1; 
       Q_hydr_inf = Q_hydr_inf1; 
       Q_S_inf = Q_S_inf1; 
        
    }

    double Getct_1() const { return ct; }
    double GetLampda_1() const { return Lampda; }
    double GetQ_hydr_inf_1() const { return Q_hydr_inf; }
    double GetQ_S_inf_1() const { return Q_S_inf; }

    void SetMoistureCapacity(double m_c1) {       
    
       m_c = m_c1; 
        
    }
    
    double Getmc_1() const { return m_c; }


    void SetMatParameters(double rho1, double CEMENT1, double SILICA1, double AGGREGATE1, double k_c1, 
            double g_11, double k_vg_c1, double k_vg_s1, double Q_over_R1) {       
        
        rho = rho1;
        CEMENT = CEMENT1;
        SILICA = SILICA1;
        AGGREGATE = AGGREGATE1;
        k_c = k_c1; 
        g_1 = g_11; 
        k_vg_c = k_vg_c1; 
        k_vg_s = k_vg_s1; 
        Q_over_R = Q_over_R1;
        
    }

    double Getrho_1() const { return rho; }
    double GetCEMENT_1() const { return CEMENT; }
    double GetSILICA_1() const { return SILICA; }
    double GetAGGREGATE_1() const { return AGGREGATE; }
    double Getk_c_1() const { return  k_c; }
    double Getg_1_1() const { return g_1; }
    double Getk_vg_c_1() const { return k_vg_c; }
    double Getk_vg_s_1() const { return k_vg_s; }
    double GetQ_over_R_1() const { return Q_over_R; }
    
};

/// @} chrono_fea

}  // end namespace flow
}  // end namespace chrono

#endif
