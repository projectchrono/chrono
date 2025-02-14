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
// Authors: Erol Lale
//          Ke Yu
// =============================================================================
// Material class for LDPM and CSL elements 
//
// A description of the material parameter can be found in: https://doi.org/10.1016/j.cemconcomp.2011.02.011
// =============================================================================

#ifndef CHWOODMATERIALVECT_H
#define CHWOODMATERIALVECT_H

#include "chrono_wood/ChWoodApi.h"
#include "chrono/core/ChMatrix33.h"
#include <vector>
#include <string>
//#include "chrono/fea/ChElementBeam.h"

namespace chrono {
namespace wood {

/// @addtogroup wood_elements
/// @{

/// Definition of materials to be used for CSL beams and LDPM tets utilizing the lattice discrete particle model.
class ChWoodApi ChWoodMaterialVECT {
  public:
    /// Construct an isotropic elastic material.
    ChWoodMaterialVECT(double rho,  		///< material density
                       double E0,    	///< Mesoscale Young's modulus
                       double alpha,   	///< Mesoscale Poisson like parameter
                       double sigmat, double sigmas, double nt, double lt, 
					   double Ed, double sigmac0, double beta, double Hc0,
					   double Hc1, double kc0, double kc1, double kc2, double kc3,
					   double mu0, double muinf, double sigmaN0, double kt
    );
    
    ChWoodMaterialVECT();

    /*
    //copy constructor:
	ChWoodMaterialVECT(const ChWoodMaterialVECT& my_material);
	*/

    // Destructor declared:
    ~ChWoodMaterialVECT();

    /// Return the material density.
     double Get_density() const { return m_rho; }
     
     void Set_density(double rho) { m_rho=rho; }
	
    /// Return the Macroscale Modulus of Elasticity.
    double Get_E0() const { return m_E0; }
	  
    void Set_E0(double E0) { m_E0=E0; }
    
    /// Return the Macroscale Poisson Ratio.
    double Get_alpha() const { return m_alpha; }
    
    void Set_alpha(double alpha) { m_alpha=alpha; }
    
    /// Return the tensile strength.
    double Get_sigmat() const { return m_sigmat; }
    void Set_sigmat(double sigmat) { m_sigmat = sigmat; }

    /// Return the shear strength.
    double Get_sigmas() const { return m_sigmas; }
    void Set_sigmas(double sigmas) { m_sigmas = sigmas; }

    /// Return the softening exponent.
    double Get_nt() const { return m_nt; }
    void Set_nt(double nt) { m_nt = nt; }

    /// Return the fracture energy.
    // double Get_Gt() const { return m_Gt; }
    // void Set_Gt(double Gt) { m_Gt = Gt; }

    /// Return the tensile characteristic length.
    double Get_lt() const { return m_lt; }
    void Set_lt(double lt) { m_lt = lt; }

    /// Return the densified normal modulus.
    double Get_Ed() const { return m_Ed; }
    void Set_Ed(double Ed) { m_Ed = Ed; }

    /// Return the compressive yield strength.
    double Get_sigmac0() const { return m_sigmac0; }
    void Set_sigmac0(double sigmac0) { m_sigmac0 = sigmac0; }

    /// Return the volunmetric deviatoric coupling.
    double Get_beta() const { return m_beta; }
    void Set_beta(double beta) { m_beta = beta; }

    /// Return the initial hardening modulus.
    double Get_Hc0() const { return m_Hc0; }
    void Set_Hc0(double Hc0) { m_Hc0 = Hc0; }

    /// Return the final hardening modulus.
    double Get_Hc1() const { return m_Hc1; }
    void Set_Hc1(double Hc1) { m_Hc1 = Hc1; }

    /// Return the transitional strain ratio.
    double Get_kc0() const { return m_kc0; }
    void Set_kc0(double kc0) { m_kc0 = kc0; }

    /// Return the deviatoric strain threshold ratio.
    double Get_kc1() const { return m_kc1; }
    void Set_kc1(double kc1) { m_kc1 = kc1; }

    /// Return the deviatoric damage parameter.
    double Get_kc2() const { return m_kc2; }
    void Set_kc2(double kc2) { m_kc2 = kc2; }

    /// Return the material parameter, 0.1.
    double Get_kc3() const { return m_kc3; }
    void Set_kc3(double kc3) { m_kc3 = kc3; }

    /// Return the initial friction.
    double Get_mu0() const { return m_mu0; }
    void Set_mu0(double mu0) { m_mu0 = mu0; }

    /// Return the asymptotic friction
    double Get_muinf() const { return m_muinf; }
    void Set_muinf(double muinf) { m_muinf = muinf; }

    /// Return the transitional stress.
    double Get_sigmaN0() const { return m_sigmaN0; }
    void Set_sigmaN0(double sigmaN0) { m_sigmaN0 = sigmaN0; }

    /// Return the tensile unloading.
    double Get_kt() const { return m_kt; }
    void Set_kt(double kt) { m_kt = kt; }
    
    /// Set and Get RayleighDampingK coefficient.
    double GetRayleighDampingK() const { return RayleighDampingK; }
    void SetRayleighDampingK(double myRayleighDampingK) { RayleighDampingK = myRayleighDampingK; }
    
    /// Set and Get RayleighDampingM coefficient.
    double GetRayleighDampingM() const { return RayleighDampingM; }
    void SetRayleighDampingM(double myRayleighDampingM) { RayleighDampingM = myRayleighDampingM; }
    
    /// Return the tensile unloading.
    double GetCoupleMultiplier() const { return mcouple_mult; }
    void SetCoupleMultiplier(double couple_mult) { mcouple_mult = couple_mult; }
	
	/// Return the tensile unloading.
    double GetElasticAnalysisFlag() const { return m_ElasticAnalysis; }
    void SetElasticAnalysisFlag(double ElasticAnalysis) { m_ElasticAnalysis = ElasticAnalysis; }	
	
	
	/// Return the tensile unloading.
    double GetCoupleContrib2EquStrainFlag() const { return m_CoupleContrib2EquStrain; }
    void SetCoupleContrib2EquStrainFlag(double CoupleContrib2EquStrain) { m_CoupleContrib2EquStrain = CoupleContrib2EquStrain; }	
    /// Compute stresses from given strains and state variables.
    void ComputeStress(ChVectorDynamic<>& mstrain, ChVectorDynamic<>& curvature, double &len, double& epsV, ChVectorDynamic<>& statev, double& area, double& width, double& height, ChVectorDynamic<>& mstress, ChVectorDynamic<>& mcouple);
    //
	void ComputeStress(ChVectorDynamic<>& mstrain, ChVectorDynamic<>& curvature, ChVectorDynamic<>& eigenstrain, double &len, double& epsV, ChVectorDynamic<>& statev, double& area, double& width, double& height, ChVectorDynamic<>& mstress, ChVectorDynamic<>& mcouple);
    
    double FractureBC(ChVectorDynamic<>& mstrain, double& len, double& epsQ, double& epsT, ChVectorDynamic<>& statev);

    //double CompressBC(ChVectorDynamic<>& mstrain, double& epsV, ChVectorDynamic<>& statev);
	double CompressBC(ChVectorDynamic<>& mstrain, double& len, double& epsQ, double& epsT, ChVectorDynamic<>& statev);

    std::pair<double, double> ShearBC(ChVectorDynamic<>& mstrain, ChVectorDynamic<>& statev);
	
	std::vector<double> ShearBC(ChVectorDynamic<>& mstrain, ChVectorDynamic<>& curvature, double rN2, ChVectorDynamic<>& statev);
    
  private:
    
    double m_rho;               ///< density
    double m_E0;    			///< mesoscale modulus of elasticity
    double m_alpha; 			///< mesoscale Poisson like parameter
    double m_sigmat;            ///< tensile strength
    double m_sigmas;            ///< shear strength
    double m_nt;                ///< softening exponent
    //double m_Gt;                ///< fracture energy
    double m_lt;                ///< tensile characteristic length
    double m_Ed;                ///< densified normal modulus
    double m_sigmac0;           ///< compressive yield strength
    double m_beta;              ///< volunmetric deviatoric coupling
    double m_Hc0;               ///< initial hardening modulus
    double m_Hc1;               ///< final hardening modulus
    double m_kc0;               ///< transitional strain ratio
    double m_kc1;               ///< deviatoric strain threshold ratio
    double m_kc2;               ///< deviatoric damage parameter
    double m_kc3;               ///< material parameter, 0.1
    double m_mu0;               ///< initial friction
    double m_muinf;             ///< asymptotic friction
    double m_sigmaN0;           ///< transitional stress.
    double m_kt;                ///< tensile unloading
    double RayleighDampingK=0;
    double RayleighDampingM=0;
    double mcouple_mult=0;
	bool m_ElasticAnalysis=false;
	bool m_CoupleContrib2EquStrain=false;
  //public:
  //EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/// @} wood_elements

}  // end of namespace wood
}  // end of namespace chrono

#endif
