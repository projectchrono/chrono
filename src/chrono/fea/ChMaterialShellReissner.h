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
// Authors: Alessandro Tasora
// =============================================================================
// Material for 6-field Reissner shells
// =============================================================================

#ifndef CHMATERIALSHELLREISSNER_H
#define CHMATERIALSHELLREISSNER_H

#include <array>
#include <vector>

#include "chrono/fea/ChElementShell.h"

namespace chrono {
namespace fea {

/// @addtogroup fea_elements
/// @{


//  forward
class ChMaterialShellReissner;



/// Base interface for elasticity of 6-field Reissner-Mindlin shells (kinematically-exact shell theory
/// as in Witkowski et al.) to be used in a ChMaterialShellReissner.
/// Children classes must implement the ComputeStress function to get
///    {n_u,n_v,m_u,m_v}=f({e_u,e_v,k_u,k_v})
/// Inherited materials do not define any thickness, which should be
/// a property of the element or its layer(s) using this material.

class ChApi ChElasticityReissner {
  public:
    ChElasticityReissner() : section(nullptr) {}

    virtual ~ChElasticityReissner() {}

    /// Compute the generalized force and torque, given actual deformation and curvature.
	/// This MUST be implemented by subclasses.
    virtual void ComputeStress(ChVector<>& n_u,          ///< forces along \e u direction (per unit length)
                               ChVector<>& n_v,          ///< forces along \e v direction (per unit length)
                               ChVector<>& m_u,          ///< torques along \e u direction (per unit length)
                               ChVector<>& m_v,          ///< torques along \e v direction (per unit length)
                               const ChVector<>& eps_u,  ///< strains along \e u direction
                               const ChVector<>& eps_v,  ///< strains along \e v direction
                               const ChVector<>& kur_u,  ///< curvature along \e u direction
                               const ChVector<>& kur_v,  ///< curvature along \e v direction
                               const double z_inf,       ///< layer lower z value (along thickness coord)
                               const double z_sup,       ///< layer upper z value (along thickness coord)
                               const double angle        ///< layer angle respect to x (if needed)
    ) = 0;

    /// Compute the 12x12 stiffness matrix [Km] , that is [ds/de], the tangent of the constitutive relation
    /// stresses/strains. 
    /// By default, it is computed by backward differentiation from the ComputeStress() function,
    /// but inherited classes should better provide an analytical form, if possible.
    virtual void ComputeStiffnessMatrix(ChMatrixRef mC,           ///< tangent matrix
                                 const ChVector<>& eps_u,  ///< strains along \e u direction
                                 const ChVector<>& eps_v,  ///< strains along \e v direction
                                 const ChVector<>& kur_u,  ///< curvature along \e u direction
                                 const ChVector<>& kur_v,  ///< curvature along \e v direction
                                 const double z_inf,       ///< layer lower z value (along thickness coord)
                                 const double z_sup,       ///< layer upper z value (along thickness coord)
                                 const double angle        ///< layer angle respect to x (if needed)
    );

    ChMaterialShellReissner* section;
};



/// Elasticity of 6-field Reissner-Mindlin shells (kinematically-exact shell theory
/// as in Witkowski et al.) to be used in a ChMaterialShellReissner.
/// This class implements material properties for a layer from the Reissner theory,
/// for the case of isotropic linear linear elastic material.
/// This is probably the material that you need most often when using 6-field shells.
/// Previously: ChMaterialShellReissnerIsothropic

class ChApi ChElasticityReissnerIsothropic : public ChElasticityReissner {
  public:
    /// Construct an isotropic material.
    ChElasticityReissnerIsothropic   (double E,            ///< Young's modulus
                                      double nu,           ///< Poisson ratio
                                      double alpha = 1.0,  ///< shear factor
                                      double beta = 0.1    ///< torque factor
    );

    /// Return the elasticity moduli
    double Get_E() const { return m_E; }
    /// Return the Poisson ratio
    double Get_nu() const { return m_nu; }
    /// Return the shear factor
    double Get_alpha() const { return m_alpha; }
    /// Return the torque factor
    double Get_beta() const { return m_beta; }

    /// The FE code will evaluate this function to compute
    /// per-unit-length forces/torques given the u,v strains/curvatures.
    virtual void ComputeStress(
        ChVector<>& n_u,          ///< forces along \e u direction (per unit length)
        ChVector<>& n_v,          ///< forces along \e v direction (per unit length)
        ChVector<>& m_u,          ///< torques along \e u direction (per unit length)
        ChVector<>& m_v,          ///< torques along \e v direction (per unit length)
        const ChVector<>& eps_u,  ///< strains along \e u direction
        const ChVector<>& eps_v,  ///< strains along \e v direction
        const ChVector<>& kur_u,  ///< curvature along \e u direction
        const ChVector<>& kur_v,  ///< curvature along \e v direction
        const double z_inf,       ///< layer lower z value (along thickness coord)
        const double z_sup,       ///< layer upper z value (along thickness coord)
        const double angle        ///< layer angle respect to x (if needed) -not used in this, isotropic
    );

    /// Compute 12x12 stiffness matrix [Km] , that is [ds/de], the tangent of the constitutive relation
    /// per-unit-length forces/torques vs generalized strains.
    virtual void ComputeStiffnessMatrix(
        ChMatrixRef mC,           ///< tangent matrix
        const ChVector<>& eps_u,  ///< strains along \e u direction
        const ChVector<>& eps_v,  ///< strains along \e v direction
        const ChVector<>& kur_u,  ///< curvature along \e u direction
        const ChVector<>& kur_v,  ///< curvature along \e v direction
        const double z_inf,       ///< layer lower z value (along thickness coord)
        const double z_sup,       ///< layer upper z value (along thickness coord)
        const double angle        ///< layer angle respect to x (if needed) -not used in this, isotropic
    );

  private:
    double m_E;      ///< elasticity moduli
    double m_nu;     ///< Poisson ratio
    double m_alpha;  ///< shear factor
    double m_beta;   ///< torque factor
};


/// Elasticity of 6-field Reissner-Mindlin shells (kinematically-exact shell theory
/// as in Witkowski et al.) to be used in a ChMaterialShellReissner.
/// This class implements material properties for a layer from the Reissner theory,
/// for the case of orthotropic linear elastic material.
/// This is useful for laminated shells. One direction can be made softer than the other.
/// Note that the angle and the thickness are defined when adding a material with this elasticity to
/// a shell finite element (ex. ChElementShellReissner4) as a layer.
/// Previously: ChMaterialShellReissnerOrthotropic

class ChApi ChElasticityReissnerOrthotropic : public ChElasticityReissner {
  public:
    /// Construct an orthotropic material
    ChElasticityReissnerOrthotropic   (double m_E_x,    ///< Young's modulus on x
                                       double m_E_y,    ///< Young's modulus on y
                                       double m_nu_xy,  ///< Poisson ratio xy (for yx it holds: nu_yx*E_x = nu_xy*E_y)
                                       double m_G_xy,   ///< Shear modulus, in plane
                                       double m_G_xz,   ///< Shear modulus, transverse
                                       double m_G_yz,   ///< Shear modulus, transverse
                                       double m_alpha = 1.0,  ///< shear factor
                                       double m_beta = 0.1    ///< torque factor
    );
    /// Construct an orthotropic material as sub case isotropic
    ChElasticityReissnerOrthotropic   (double m_E,            ///< Young's modulus on x
                                       double m_nu,           ///< Poisson ratio
                                       double m_alpha = 1.0,  ///< shear factor
                                       double m_beta = 0.1    ///< torque factor
    );

    /// Return the elasticity moduli, on x
    double Get_E_x() const { return E_x; }
    /// Return the elasticity moduli, on y
    double Get_E_y() const { return E_y; }
    /// Return the Poisson ratio, for xy
    double Get_nu_xy() const { return nu_xy; }
    /// Return the Poisson ratio, for yx (follows xy as it must be nu_yx*E_x = nu_xy*E_y)
    double Get_nu_yx() const { return nu_xy * (E_y / E_x); }
    /// Return the shear mod, in plane
    double Get_G_xy() const { return G_xy; }
    /// Return the shear mod, transverse
    double Get_G_xz() const { return G_xz; }
    /// Return the shear mod, transverse
    double Get_G_yz() const { return G_yz; }
    /// Return the shear factor
    double Get_alpha() const { return alpha; }
    /// Return the torque factor
    double Get_beta() const { return beta; }

    /// The FE code will evaluate this function to compute
    /// per-unit-length forces/torques  given the u,v strains/curvatures.
    virtual void ComputeStress(
        ChVector<>& n_u,          ///< forces along \e u direction (per unit length)
        ChVector<>& n_v,          ///< forces along \e v direction (per unit length)
        ChVector<>& m_u,          ///< torques along \e u direction (per unit length)
        ChVector<>& m_v,          ///< torques along \e v direction (per unit length)
        const ChVector<>& eps_u,  ///< strains along \e u direction
        const ChVector<>& eps_v,  ///< strains along \e v direction
        const ChVector<>& kur_u,  ///< curvature along \e u direction
        const ChVector<>& kur_v,  ///< curvature along \e v direction
        const double z_inf,       ///< layer lower z value (along thickness coord)
        const double z_sup,       ///< layer upper z value (along thickness coord)
        const double angle        ///< layer angle respect to x (if needed) -not used in this, isotropic
    );

    /// /// Compute the 12x12 stiffness matrix [Km] , that is [ds/de], the tangent of the constitutive relation
    /// stresses/strains. 
	virtual void ComputeStiffnessMatrix(
        ChMatrixRef mC,           ///< tangent matrix
        const ChVector<>& eps_u,  ///< strains along \e u direction
        const ChVector<>& eps_v,  ///< strains along \e v direction
        const ChVector<>& kur_u,  ///< curvature along \e u direction
        const ChVector<>& kur_v,  ///< curvature along \e v direction
        const double z_inf,       ///< layer lower z value (along thickness coord)
        const double z_sup,       ///< layer upper z value (along thickness coord)
        const double angle        ///< layer angle respect to x (if needed) -not used in this, isotropic
    );

  private:
    double E_x;    ///< elasticity moduli
    double E_y;    ///< elasticity moduli
    double nu_xy;  ///< Poisson ratio
    double G_xy;   ///< Shear factor, in plane
    double G_xz;   ///< Shear factor, out of plane
    double G_yz;   ///< Shear factor, out of plane
    double alpha;  ///< shear factor
    double beta;   ///< torque factor
};

// ----------------------------------------------------------------------------

/// Base class for internal variables of Reissner shells materials.
/// Especially useful for plasticity, where internal variables are used
/// to carry information on plastic flow, accumulated flow, etc.
class ChApi ChShellReissnerInternalData {
  public:
    ChShellReissnerInternalData() : p_strain_acc(0) {}

    virtual ~ChShellReissnerInternalData(){};

    virtual void Copy(const ChShellReissnerInternalData& other) { p_strain_acc = other.p_strain_acc; }

    double p_strain_acc;  // accumulated flow,  \overbar\eps^p  in Neto-Owen book
};

/// Base interface for plasticity of 6-field Reissner-Mindlin shells (kinematically-exact shell theory
/// as in Witkowski et al.) to be used in a ChMaterialShellReissner.
/// Children classes must implement the ComputeStressWithReturnMapping to compute
/// effective stress and strain given a tentative strain that might violate the yeld function.
/// Inherited materials do not define any thickness, which should be
/// a property of the element or its layer(s) using this material.
class ChApi ChPlasticityReissner {
  public:
    ChPlasticityReissner();

    virtual ~ChPlasticityReissner() {}

    /// Given a trial strain, it computes the effective stress and strain by
    /// clamping against the yeld surface. An implicit return mapping integration
    /// step is computed automatically per each call of this function.
    /// Note: for the elastic part, it must use the elasticity model in this->section->elasticity.
    /// If not beyond yeld, simply:
    ///      elastic strain = tot strain - plastic strain
    /// If it is beyond yeld:
    ///      elastic strain is computed by fully implicit strain integration with return mapping,
    ///      and plastic strains in "data_new" are updated.
    /// Returns true if it had to do return mapping, false if it was in elastic regime
	/// This MUST be implemented by subclasses.
    virtual bool ComputeStressWithReturnMapping(
		ChVector<>& n_u,          ///< forces along \e u direction (per unit length)
        ChVector<>& n_v,          ///< forces along \e v direction (per unit length)
        ChVector<>& m_u,          ///< torques along \e u direction (per unit length)
        ChVector<>& m_v,          ///< torques along \e v direction (per unit length)
        ChShellReissnerInternalData& data_new,  ///< updated material internal variables, at this point, including {p_strain_e, p_strain_k, p_strain_acc}
        const ChVector<>& eps_u_trial,  ///< trial strains along \e u direction
        const ChVector<>& eps_v_trial,  ///< trial strains along \e v direction
        const ChVector<>& kur_u_trial,  ///< trial curvature along \e u direction
        const ChVector<>& kur_v_trial,  ///< trial curvature along \e v direction
        const ChShellReissnerInternalData& data,  ///< trial material internal variables, at this point, including {p_strain_e, p_strain_k, p_strain_acc}
		const double z_inf,       ///< layer lower z value (along thickness coord)
        const double z_sup,       ///< layer upper z value (along thickness coord)
        const double angle        ///< layer angle respect to x (if needed) 
        ) = 0;

    /// Compute the 12x12 tangent material stiffness matrix [Km]=d\sigma/d\epsilon,
    /// given actual internal data and deformation and curvature (if needed). If in
    /// plastic regime, uses elastoplastic matrix, otherwise uses elastic.
    /// This must be overridden by subclasses if an analytical solution is
    /// known (preferred for high performance), otherwise the base behaviour here is to compute
    /// [Km] by numerical differentiation calling ComputeStressWithReturnMapping() multiple times.
    virtual void ComputeStiffnessMatrixElastoplastic(
        ChMatrixRef K,        ///< 12x12 material elastoplastic stiffness matrix values here
        const ChVector<>& eps_u,  ///< strains along \e u direction
        const ChVector<>& eps_v,  ///< strains along \e v direction
        const ChVector<>& kur_u,  ///< curvature along \e u direction
        const ChVector<>& kur_v,  ///< curvature along \e v direction
		const ChShellReissnerInternalData& data,  ///< updated material internal variables, at this point including {p_strain_e, p_strain_k, p_strain_acc}
        const double z_inf,       ///< layer lower z value (along thickness coord)
        const double z_sup,       ///< layer upper z value (along thickness coord)
        const double angle        ///< layer angle respect to x (if needed) 
    );

    // Populate a vector with the appropriate ChBeamSectionPlasticity data structures.
    // Children classes may override this. By default uses ChBeamMaterialInternalData for basic plasticity.
    // Thanks to unique_ptr there is no need to call delete for the pointed objects.
    virtual void CreatePlasticityData(int numpoints,
                                      std::vector<std::unique_ptr<ChShellReissnerInternalData>>& plastic_data);


    ChMaterialShellReissner* section;

    double nr_yeld_tolerance;
    int nr_yeld_maxiters;
};

/// @} fea_elements

}  // end of namespace fea
}  // end of namespace chrono

#endif
