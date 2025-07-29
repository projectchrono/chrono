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
// Authors: Erol Lale, Wisdom Akpan, Ke Yu, Jibril B. Coulibaly
// =============================================================================
// Material class for LDPM and CSL elements 
//
// A description of the material parameter can be found in: https://doi.org/10.1016/j.cemconcomp.2011.02.011
// =============================================================================

#ifndef CHWOODMATERIALVECT_H
#define CHWOODMATERIALVECT_H

#include "chrono/core/ChVector3.h"
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
    using StateVarVector = ChVectorN<double, 18>;

    /// Construct an isotropic elastic material.
    ChWoodMaterialVECT(double rho,  		///< material density
                       double E0,    	///< Mesoscale Young's modulus
                       double alpha,   	///< Mesoscale Poisson like parameter
                       double sigmat, double sigmac, double sigmas,
                       double nt, double lt, double rs);
    
    ChWoodMaterialVECT();

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

    /// Return the compressive yield strength.
    double Get_sigmac() const { return m_sigmac; }
    void Set_sigmac(double sigmac) { m_sigmac = sigmac; }

    /// Return the shear strength.
    double Get_sigmas() const { return m_sigmas; }
    void Set_sigmas(double sigmas) { m_sigmas = sigmas; }

    /// Return the softening exponent.
    double Get_nt() const { return m_nt; }
    void Set_nt(double nt) { m_nt = nt; }

    /// Return the tensile characteristic length.
    double Get_lt() const { return m_lt; }
    void Set_lt(double lt) { m_lt = lt; }

    /// Return the shear softening modulus ratio.
    double Get_rs() const { return m_rs; }
    void Set_rs(double rs) { m_rs = rs; }
    
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
    void ComputeStress(ChVector3d& mstrain, ChVector3d& curvature, double &len, StateVarVector& statev, double& area, double& width, double& height, double& random_field, ChVector3d& mstress, ChVector3d& mcouple);
    //void ComputeStress_NEW(ChVector3d& strain_incr, ChVector3d& curvature_incr, double &length, StateVarVector& statev, double& width, double& height, double& random_field, ChVector3d& stress, ChVector3d& surfacic_couple);
    //
	void ComputeStress(ChVector3d& mstrain, ChVector3d& curvature, ChVectorDynamic<>& eigenstrain, double &len, StateVarVector& statev, double& area, double& width, double& height, double& random_field, ChVector3d& mstress, ChVector3d& mcouple);
    
    double FractureBC(ChVector3d& mstrain, double& random_field, double& len, double& epsQ, double& epsQN,  double& epsT, StateVarVector& statev);

	double CompressBC(ChVector3d& mstrain, double& random_field, double& len, double& epsQ, double& epsT, double& epsQN, StateVarVector& statev);
    
  private:
    
    double m_rho;               ///< density
    double m_E0;    			///< mesoscale modulus of elasticity
    double m_alpha; 			///< mesoscale Poisson like parameter
    double m_sigmat;            ///< tensile strength
    double m_sigmac;            ///< compressive yield strength
    double m_sigmas;            ///< shear strength
    double m_nt;                ///< softening exponent
    double m_lt;                ///< tensile characteristic length
    double m_rs;                ///< shear softening modulus ratio
    double RayleighDampingK=0;
    double RayleighDampingM=0;
    double mcouple_mult=0; ///<  influence factor for rotational DOF: beta_N = beta_M = beta_B
	bool m_ElasticAnalysis=false;
	bool m_CoupleContrib2EquStrain=false;
  //public:
  //EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/// @} wood_elements

}  // end of namespace wood
}  // end of namespace chrono

#endif
