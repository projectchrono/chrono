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

/// @} fea_elements

}  // end of namespace fea
}  // end of namespace chrono

#endif
