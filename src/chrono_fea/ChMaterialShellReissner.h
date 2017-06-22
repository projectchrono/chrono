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
// Authors: Alessandro Tasora
// =============================================================================
// Material for 6-field Reissner shells
// =============================================================================

#ifndef CHMATERIALSHELLREISSNER_H
#define CHMATERIALSHELLREISSNER_H

#include <array>
#include <vector>

#include "chrono_fea/ChApiFEA.h"
#include "chrono_fea/ChElementShell.h"

namespace chrono {
namespace fea {

/// @addtogroup fea_elements
/// @{

// ----------------------------------------------------------------------------

/// Base class for all materials to be used for
/// 6-field Reissner-Mindlin shells (kinematically-exact shell theory)
/// as in Witkowski et al.
/// Inherited materials do not define any thickness, which should be
/// a property of the element or its layer(s) using this material.
class ChApiFea ChMaterialShellReissner {
  public:
    /// Construct an isotropic material.
    ChMaterialShellReissner(){};

    /// Return the material density.
    double Get_rho() const { return m_rho; }

    /// The FE code will evaluate this function to compute
    /// u,v stresses/torques given the u,v strains/curvatures.
    /// Inherited classes MUST implement this.
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

    /// Compute [C] , that is [ds/de], the tangent of the constitutive relation
    /// stresses/strains. In many cases this is a constant matrix, but it could
    /// change for instance in case of hardening or softening materials, etc.
    /// By default, it is computed by backward differentiation from the ComputeStress() function,
    /// but inherited classes should better provide an analytical form, if possible.
    virtual void ComputeTangentC(ChMatrix<>& mC,           ///< tangent matrix
                                 const ChVector<>& eps_u,  ///< strains along \e u direction
                                 const ChVector<>& eps_v,  ///< strains along \e v direction
                                 const ChVector<>& kur_u,  ///< curvature along \e u direction
                                 const ChVector<>& kur_v,  ///< curvature along \e v direction
                                 const double z_inf,       ///< layer lower z value (along thickness coord)
                                 const double z_sup,       ///< layer upper z value (along thickness coord)
                                 const double angle        ///< layer angle respect to x (if needed)
    );

  protected:
    double m_rho;  ///< density
};

/// Material definition.
/// This class implements material properties for a layer from the Reissner theory,
/// for the case of isotropic linear linear elastic material.
/// This is probably the material that you need most often when using 6-field shells.
class ChApiFea ChMaterialShellReissnerIsothropic : public ChMaterialShellReissner {
  public:
    /// Construct an isotropic material.
    ChMaterialShellReissnerIsothropic(double rho,          ///< material density
                                      double E,            ///< Young's modulus
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

    /// Compute [C] , that is [ds/de], the tangent of the constitutive relation
    /// per-unit-length forces/torques vs strains.
    virtual void ComputeTangentC(
        ChMatrix<>& mC,           ///< tangent matrix
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

/// Material definition.
/// This class implements material properties for a layer from the Reissner theory,
/// for the case of orthotropic linear elastic material.
/// This is useful for laminated shells. One direction can be made softer than the other.
/// Note that the angle and the thickness are defined when adding this material to
/// a finite element as a layer.
class ChApiFea ChMaterialShellReissnerOrthotropic : public ChMaterialShellReissner {
  public:
    /// Construct an orthotropic material
    ChMaterialShellReissnerOrthotropic(double m_rho,    ///< material density
                                       double m_E_x,    ///< Young's modulus on x
                                       double m_E_y,    ///< Young's modulus on y
                                       double m_nu_xy,  ///< Poisson ratio xy (for yx it holds: nu_yx*E_x = nu_xy*E_y)
                                       double m_G_xy,   ///< Shear modulus, in plane
                                       double m_G_xz,   ///< Shear modulus, transverse
                                       double m_G_yz,   ///< Shear modulus, transverse
                                       double m_alpha = 1.0,  ///< shear factor
                                       double m_beta = 0.1    ///< torque factor
    );
    /// Construct an orthotropic material as sub case isotropic
    ChMaterialShellReissnerOrthotropic(double m_rho,          ///< material density
                                       double m_E,            ///< Young's modulus on x
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

    /// Compute [C] , that is [ds/de], the tangent of the constitutive relation
    /// per-unit-length forces/torques  vs strains.
    virtual void ComputeTangentC(
        ChMatrix<>& mC,           ///< tangent matrix
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
