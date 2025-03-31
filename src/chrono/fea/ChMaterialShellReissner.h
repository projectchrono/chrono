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
    virtual void ComputeStress(ChVector3d& n_u,          ///< forces along \e u direction (per unit length)
                               ChVector3d& n_v,          ///< forces along \e v direction (per unit length)
                               ChVector3d& m_u,          ///< torques along \e u direction (per unit length)
                               ChVector3d& m_v,          ///< torques along \e v direction (per unit length)
                               const ChVector3d& eps_u,  ///< strains along \e u direction
                               const ChVector3d& eps_v,  ///< strains along \e v direction
                               const ChVector3d& kur_u,  ///< curvature along \e u direction
                               const ChVector3d& kur_v,  ///< curvature along \e v direction
                               const double z_inf,       ///< layer lower z value (along thickness coord)
                               const double z_sup,       ///< layer upper z value (along thickness coord)
                               const double angle        ///< layer angle respect to x (if needed)
                               ) = 0;

    /// Compute the 12x12 stiffness matrix [Km] , that is [ds/de], the tangent of the constitutive relation
    /// stresses/strains.
    /// By default, it is computed by backward differentiation from the ComputeStress() function,
    /// but inherited classes should better provide an analytical form, if possible.
    virtual void ComputeStiffnessMatrix(ChMatrixRef mC,           ///< tangent matrix
                                        const ChVector3d& eps_u,  ///< strains along \e u direction
                                        const ChVector3d& eps_v,  ///< strains along \e v direction
                                        const ChVector3d& kur_u,  ///< curvature along \e u direction
                                        const ChVector3d& kur_v,  ///< curvature along \e v direction
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
    ChElasticityReissnerIsothropic(double E,            ///< Young's modulus
                                   double nu,           ///< Poisson ratio
                                   double alpha = 1.0,  ///< shear factor
                                   double beta = 0.1    ///< torque factor
    );

    /// Return the elasticity moduli
    double GetYoungModulus() const { return m_E; }
    /// Return the Poisson ratio
    double GetPoissonRatio() const { return m_nu; }
    /// Return the shear factor
    double GetShearFactor() const { return m_alpha; }
    /// Return the torque factor
    double GetTorqueFactor() const { return m_beta; }

    /// The FE code will evaluate this function to compute
    /// per-unit-length forces/torques given the u,v strains/curvatures.
    virtual void ComputeStress(
        ChVector3d& n_u,          ///< forces along \e u direction (per unit length)
        ChVector3d& n_v,          ///< forces along \e v direction (per unit length)
        ChVector3d& m_u,          ///< torques along \e u direction (per unit length)
        ChVector3d& m_v,          ///< torques along \e v direction (per unit length)
        const ChVector3d& eps_u,  ///< strains along \e u direction
        const ChVector3d& eps_v,  ///< strains along \e v direction
        const ChVector3d& kur_u,  ///< curvature along \e u direction
        const ChVector3d& kur_v,  ///< curvature along \e v direction
        const double z_inf,       ///< layer lower z value (along thickness coord)
        const double z_sup,       ///< layer upper z value (along thickness coord)
        const double angle        ///< layer angle respect to x (if needed) -not used in this, isotropic
    );

    /// Compute 12x12 stiffness matrix [Km] , that is [ds/de], the tangent of the constitutive relation
    /// per-unit-length forces/torques vs generalized strains.
    virtual void ComputeStiffnessMatrix(
        ChMatrixRef mC,           ///< tangent matrix
        const ChVector3d& eps_u,  ///< strains along \e u direction
        const ChVector3d& eps_v,  ///< strains along \e v direction
        const ChVector3d& kur_u,  ///< curvature along \e u direction
        const ChVector3d& kur_v,  ///< curvature along \e v direction
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
    ChElasticityReissnerOrthotropic(double m_E_x,    ///< Young's modulus on x
                                    double m_E_y,    ///< Young's modulus on y
                                    double m_nu_xy,  ///< Poisson ratio xy (for yx it holds: nu_yx*E_x = nu_xy*E_y)
                                    double m_G_xy,   ///< Shear modulus, in plane
                                    double m_G_xz,   ///< Shear modulus, transverse
                                    double m_G_yz,   ///< Shear modulus, transverse
                                    double m_alpha = 1.0,  ///< shear factor
                                    double m_beta = 0.1    ///< torque factor
    );
    /// Construct an orthotropic material as sub case isotropic
    ChElasticityReissnerOrthotropic(double m_E,            ///< Young's modulus on x
                                    double m_nu,           ///< Poisson ratio
                                    double m_alpha = 1.0,  ///< shear factor
                                    double m_beta = 0.1    ///< torque factor
    );

    /// Return the elasticity moduli, on x
    double GetYoungModulusX() const { return E_x; }
    /// Return the elasticity moduli, on y
    double GetYoungModulusY() const { return E_y; }
    /// Return the Poisson ratio, for xy
    double GetPoissonRatioXY() const { return nu_xy; }
    /// Return the Poisson ratio, for yx (follows xy as it must be nu_yx*E_x = nu_xy*E_y)
    double GetPoissonRatioYX() const { return nu_xy * (E_y / E_x); }
    /// Return the shear mod, in plane
    double GetShearModulusXY() const { return G_xy; }
    /// Return the shear mod, transverse
    double GetShearModulusXZ() const { return G_xz; }
    /// Return the shear mod, transverse
    double GetShearModulusYZ() const { return G_yz; }
    /// Return the shear factor
    double GetShearFactor() const { return alpha; }
    /// Return the torque factor
    double GetTorqueFactor() const { return beta; }

    /// The FE code will evaluate this function to compute
    /// per-unit-length forces/torques  given the u,v strains/curvatures.
    virtual void ComputeStress(
        ChVector3d& n_u,          ///< forces along \e u direction (per unit length)
        ChVector3d& n_v,          ///< forces along \e v direction (per unit length)
        ChVector3d& m_u,          ///< torques along \e u direction (per unit length)
        ChVector3d& m_v,          ///< torques along \e v direction (per unit length)
        const ChVector3d& eps_u,  ///< strains along \e u direction
        const ChVector3d& eps_v,  ///< strains along \e v direction
        const ChVector3d& kur_u,  ///< curvature along \e u direction
        const ChVector3d& kur_v,  ///< curvature along \e v direction
        const double z_inf,       ///< layer lower z value (along thickness coord)
        const double z_sup,       ///< layer upper z value (along thickness coord)
        const double angle        ///< layer angle respect to x (if needed) -not used in this, isotropic
    );

    // Compute the 12x12 stiffness matrix [Km] , that is [ds/de], the tangent of the constitutive relation
    // stresses/strains.
    virtual void ComputeStiffnessMatrix(
        ChMatrixRef mC,           ///< tangent matrix
        const ChVector3d& eps_u,  ///< strains along \e u direction
        const ChVector3d& eps_v,  ///< strains along \e v direction
        const ChVector3d& kur_u,  ///< curvature along \e u direction
        const ChVector3d& kur_v,  ///< curvature along \e v direction
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

/// Generic linear elasticity for 6-field Reissner-Mindlin shells (kinematically-exact shell theory
/// as in Witkowski et al.) to be used in a ChMaterialShellReissner.
/// This uses a 12x12 matrix [E] from user-input data. The [E] matrix can be
/// computed from a preprocessing stage using a FEA analysis over a detailed 3D model
/// of a slab of shell, hence recovering the 6x6 matrix in the linear mapping:
/// {n,m}=[E]{e,k}.

class ChApi ChElasticityReissnerGeneric : public ChElasticityReissner {
  public:
    ChElasticityReissnerGeneric();

    virtual ~ChElasticityReissnerGeneric() {}

    /// Access the 12x12 [E] matrix, for getting/setting its values.
    /// This is the matrix that defines the linear elastic constitutive model
    /// as it maps  yxz displacements "e" and xyz rotations "k"
    /// to the "n" force and  "m" torque as in
    ///   {n_u,n_v,m_u,m_v}=[E]{e_u,e_v,k_u,k_v}.
    ChMatrixNM<double, 12, 12>& Ematrix() { return this->mE; }

    /// The FE code will evaluate this function to compute
    /// per-unit-length forces/torques  given the u,v strains/curvatures.
    virtual void ComputeStress(
        ChVector3d& n_u,          ///< forces along \e u direction (per unit length)
        ChVector3d& n_v,          ///< forces along \e v direction (per unit length)
        ChVector3d& m_u,          ///< torques along \e u direction (per unit length)
        ChVector3d& m_v,          ///< torques along \e v direction (per unit length)
        const ChVector3d& eps_u,  ///< strains along \e u direction
        const ChVector3d& eps_v,  ///< strains along \e v direction
        const ChVector3d& kur_u,  ///< curvature along \e u direction
        const ChVector3d& kur_v,  ///< curvature along \e v direction
        const double z_inf,       ///< layer lower z value (along thickness coord)
        const double z_sup,       ///< layer upper z value (along thickness coord)
        const double angle        ///< layer angle respect to x (if needed) -not used in this, isotropic
        ) override;

    /// /// Compute the 12x12 stiffness matrix [Km] , that is [ds/de], the tangent of the constitutive relation
    /// stresses/strains.
    virtual void ComputeStiffnessMatrix(
        ChMatrixRef mC,           ///< tangent matrix
        const ChVector3d& eps_u,  ///< strains along \e u direction
        const ChVector3d& eps_v,  ///< strains along \e v direction
        const ChVector3d& kur_u,  ///< curvature along \e u direction
        const ChVector3d& kur_v,  ///< curvature along \e v direction
        const double z_inf,       ///< layer lower z value (along thickness coord)
        const double z_sup,       ///< layer upper z value (along thickness coord)
        const double angle        ///< layer angle respect to x (if needed) -not used in this, isotropic
        ) override;

  private:
    ChMatrixNM<double, 12, 12> mE;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
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
        ChVector3d& n_u,                          ///< forces along \e u direction (per unit length)
        ChVector3d& n_v,                          ///< forces along \e v direction (per unit length)
        ChVector3d& m_u,                          ///< torques along \e u direction (per unit length)
        ChVector3d& m_v,                          ///< torques along \e v direction (per unit length)
        ChShellReissnerInternalData& data_new,    ///< updated material internal variables, at this point, including
                                                  ///< {p_strain_e, p_strain_k, p_strain_acc}
        const ChVector3d& eps_u_trial,            ///< trial strains along \e u direction
        const ChVector3d& eps_v_trial,            ///< trial strains along \e v direction
        const ChVector3d& kur_u_trial,            ///< trial curvature along \e u direction
        const ChVector3d& kur_v_trial,            ///< trial curvature along \e v direction
        const ChShellReissnerInternalData& data,  ///< trial material internal variables, at this point, including
                                                  ///< {p_strain_e, p_strain_k, p_strain_acc}
        const double z_inf,                       ///< layer lower z value (along thickness coord)
        const double z_sup,                       ///< layer upper z value (along thickness coord)
        const double angle                        ///< layer angle respect to x (if needed)
        ) = 0;

    /// Compute the 12x12 tangent material stiffness matrix [Km] = d&sigma;/d&epsilon;,
    /// given actual internal data and deformation and curvature (if needed). If in
    /// plastic regime, uses elastoplastic matrix, otherwise uses elastic.
    /// This must be overridden by subclasses if an analytical solution is
    /// known (preferred for high performance), otherwise the base behaviour here is to compute
    /// [Km] by numerical differentiation calling ComputeStressWithReturnMapping() multiple times.
    virtual void ComputeStiffnessMatrixElastoplastic(
        ChMatrixRef K,                            ///< 12x12 material elastoplastic stiffness matrix values here
        const ChVector3d& eps_u,                  ///< strains along \e u direction
        const ChVector3d& eps_v,                  ///< strains along \e v direction
        const ChVector3d& kur_u,                  ///< curvature along \e u direction
        const ChVector3d& kur_v,                  ///< curvature along \e v direction
        const ChShellReissnerInternalData& data,  ///< updated material internal variables, at this point including
                                                  ///< {p_strain_e, p_strain_k, p_strain_acc}
        const double z_inf,                       ///< layer lower z value (along thickness coord)
        const double z_sup,                       ///< layer upper z value (along thickness coord)
        const double angle                        ///< layer angle respect to x (if needed)
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

// ----------------------------------------------------------------------------

/// Base interface for damping of 6-field Reissner-Mindlin shells (kinematically-exact shell theory
/// as in Witkowski et al.) to be used in a ChMaterialShellReissner.
/// Children classes should implement a ComputeStress function that returns generalized stresses
/// given time derivatives of strains as:
///   {n_u,n_v,m_u.m_v}=f({e_u',e_v',k_u',k_v'})

class ChApi ChDampingReissner {
  public:
    ChDampingReissner() : section(nullptr) {}

    virtual ~ChDampingReissner() {}

    /// Compute the generalized cut force and cut torque, caused by structural damping,
    /// given actual deformation speed and curvature speed.
    /// This MUST be implemented by subclasses.
    virtual void ComputeStress(ChVector3d& n_u,           ///< forces along \e u direction (per unit length)
                               ChVector3d& n_v,           ///< forces along \e v direction (per unit length)
                               ChVector3d& m_u,           ///< torques along \e u direction (per unit length)
                               ChVector3d& m_v,           ///< torques along \e v direction (per unit length)
                               const ChVector3d& deps_u,  ///< time derivative of strains along \e u direction
                               const ChVector3d& deps_v,  ///< time derivative of strains along \e v direction
                               const ChVector3d& dkur_u,  ///< time derivative of curvature along \e u direction
                               const ChVector3d& dkur_v,  ///< time derivative of curvature along \e v direction
                               const double z_inf,        ///< layer lower z value (along thickness coord)
                               const double z_sup,        ///< layer upper z value (along thickness coord)
                               const double angle         ///< layer angle respect to x (if needed)
                               ) = 0;

    /// Compute the 12x12 tangent material damping matrix, ie the jacobian [Rm]=dstress/dstrainspeed.
    /// This must be overridden by subclasses if an analytical solution is
    /// known (preferred for high performance), otherwise the base behaviour here is to compute
    /// [Rm] by numerical differentiation calling ComputeStress() multiple times.
    virtual void ComputeDampingMatrix(
        ChMatrixRef R,             ///< 12x12 material damping matrix values here
        const ChVector3d& deps_u,  ///< time derivative of strains along \e u direction
        const ChVector3d& deps_v,  ///< time derivative of strains along \e v direction
        const ChVector3d& dkur_u,  ///< time derivative of curvature along \e u direction
        const ChVector3d& dkur_v,  ///< time derivative of curvature along \e v direction
        const double z_inf,        ///< layer lower z value (along thickness coord)
        const double z_sup,        ///< layer upper z value (along thickness coord)
        const double angle         ///< layer angle respect to x (if needed) -not used in this, isotropic
    );

    ChMaterialShellReissner* section;
};

/// Simple Rayleight damping of a Reissner-mindlin shell,
/// where damping is proportional to stiffness via a beta coefficient.
/// In order to generalize it also in case of nonlinearity, the full
/// element tangent stiffness matrix cannot be used (it may contain negative eigenvalues)
/// and it can't be used to recover instant nodal caused by damping as F=beta*K*q_dt
/// so it is generalized to the following implementation at the material stress level
///   <pre>
///   {n,m}=beta*[E]*{e',k'}
///   </pre>
/// where
/// - beta is the 2nd Rayleigh damping parameter
/// - [E] is the 6x6 shell stiffness matrix at the undeformed unstressed case (hence assumed constant)
/// - {e',k'} is the speed of deformation/curvature
/// Note that the alpha mass-proportional parameter (the first of the alpha,beta parameters of the original
/// Rayleigh model) is not supported.

class ChApi ChDampingReissnerRayleigh : public ChDampingReissner {
  public:
    /// Construct the Rayleigh damping model from the stiffness model used by the shell layer.
    /// This is important because the Rayleigh damping is proportional to the stiffness,
    /// so the model must know which is the stiffness matrix of the material.
    /// Note: melasticity must be alreay set with proper values: its [E] stiffness matrix will be
    /// fetched just once for all.
    ChDampingReissnerRayleigh(std::shared_ptr<ChElasticityReissner> melasticity, const double& mbeta = 0);

    virtual ~ChDampingReissnerRayleigh() {}

    /// Compute the generalized cut force and cut torque, caused by structural damping,
    /// given actual deformation speed and curvature speed.
    virtual void ComputeStress(ChVector3d& n_u,           ///< forces along \e u direction (per unit length)
                               ChVector3d& n_v,           ///< forces along \e v direction (per unit length)
                               ChVector3d& m_u,           ///< torques along \e u direction (per unit length)
                               ChVector3d& m_v,           ///< torques along \e v direction (per unit length)
                               const ChVector3d& deps_u,  ///< time derivative of strains along \e u direction
                               const ChVector3d& deps_v,  ///< time derivative of strains along \e v direction
                               const ChVector3d& dkur_u,  ///< time derivative of curvature along \e u direction
                               const ChVector3d& dkur_v,  ///< time derivative of curvature along \e v direction
                               const double z_inf,        ///< layer lower z value (along thickness coord)
                               const double z_sup,        ///< layer upper z value (along thickness coord)
                               const double angle         ///< layer angle respect to x (if needed)
    );

    /// Compute the 6x6 tangent material damping matrix, ie the jacobian [Rm]=dstress/dstrainspeed.
    /// In this model, it is beta*[E] where [E] is the 12x12 stiffness matrix at material level, assumed constant
    virtual void ComputeDampingMatrix(
        ChMatrixRef R,             ///< 12x12 material damping matrix values here
        const ChVector3d& deps_u,  ///< time derivative of strains along \e u direction
        const ChVector3d& deps_v,  ///< time derivative of strains along \e v direction
        const ChVector3d& dkur_u,  ///< time derivative of curvature along \e u direction
        const ChVector3d& dkur_v,  ///< time derivative of curvature along \e v direction
        const double z_inf,        ///< layer lower z value (along thickness coord)
        const double z_sup,        ///< layer upper z value (along thickness coord)
        const double angle         ///< layer angle respect to x (if needed) -not used in this, isotropic
    );

    /// Get the beta Rayleigh parameter (stiffness proportional damping)
    double GetBeta() { return beta; }
    /// Set the beta Rayleigh parameter (stiffness proportional damping)
    void SetBeta(const double mbeta) { beta = mbeta; }

  private:
    std::shared_ptr<ChElasticityReissner> section_elasticity;
    ChMatrixNM<double, 12, 12>
        E_const;  // to store the precomputed stiffness matrix at undeformed unstressed initial state
    double beta;
    bool updated;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// ----------------------------------------------------------------------------

/// Material for a single layer of a 6-field Reissner-Mindlin shells
/// (kinematically-exact shell theory as in Witkowski et al).
/// This base implementation assumes that one creates a ChMaterialShellReissner
/// by providing three components:
///
/// - an elasticity model (from ChElasticityReissner classes)
/// - a plasticity model (optional, from ChPlasticityReissner classes)
/// - a damping model (optional, from ChDampingReissner classes)
///
/// Thickness is defined when adding a ChMaterialShellReissner material as a layer
/// in a shell finite element (ex. ChElementShellReissner4).
/// A material can be shared between multiple layers.

class ChApi ChMaterialShellReissner {
  public:
    ChMaterialShellReissner(std::shared_ptr<ChElasticityReissner> melasticity  ///< elasticity model
    );

    ChMaterialShellReissner(std::shared_ptr<ChElasticityReissner> melasticity,  ///< elasticity model
                            std::shared_ptr<ChPlasticityReissner> mplasticity   ///< plasticity model, if any
    );

    ChMaterialShellReissner(std::shared_ptr<ChElasticityReissner> melasticity,  ///< elasticity model
                            std::shared_ptr<ChPlasticityReissner> mplasticity,  ///< plasticity model, if any
                            std::shared_ptr<ChDampingReissner> mdamping         ///< damping model, if any
    );

    virtual ~ChMaterialShellReissner() {}

    /// Compute the generalized cut force and cut torque, given the actual generalized section strain
    /// expressed as deformation vector e and curvature k, that is: {n_u,n_v,m_u,m_v}=f({e_u,e_v,k_u,k_v}), and
    /// given the actual material state required for plasticity if any (but if mdata=nullptr,
    /// computes only the elastic force).
    /// If there is plasticity, the stress is clamped by automatically performing an implicit return mapping.
    /// In sake of generality, if possible this is the function that should be used by beam finite elements
    /// to compute internal forces, ex.by some Gauss quadrature.
    virtual void ComputeStress(
        ChVector3d& n_u,                                    ///< forces along \e u direction (per unit length)
        ChVector3d& n_v,                                    ///< forces along \e v direction (per unit length)
        ChVector3d& m_u,                                    ///< torques along \e u direction (per unit length)
        ChVector3d& m_v,                                    ///< torques along \e v direction (per unit length)
        const ChVector3d& eps_u,                            ///< strains along \e u direction
        const ChVector3d& eps_v,                            ///< strains along \e v direction
        const ChVector3d& kur_u,                            ///< curvature along \e u direction
        const ChVector3d& kur_v,                            ///< curvature along \e v direction
        const double z_inf,                                 ///< layer lower z value (along thickness coord)
        const double z_sup,                                 ///< layer upper z value (along thickness coord)
        const double angle,                                 ///< layer angle respect to x (if needed)
        ChShellReissnerInternalData* mdata_new = nullptr,   ///< updated material internal variables, at this
                                                            ///< point, including {p_strain_e, p_strain_k, p_strain_acc}
        const ChShellReissnerInternalData* mdata = nullptr  ///< current material internal variables, at this point,
                                                            ///< including {p_strain_e, p_strain_k, p_strain_acc}
    );

    /// Compute the 6x6 tangent material stiffness matrix [Km] = d&sigma;/d&epsilon;
    /// at a given strain state, and at given internal data state (if mdata=nullptr,
    /// computes only the elastic tangent stiffenss, regardless of plasticity).
    virtual void ComputeStiffnessMatrix(
        ChMatrixRef K,                                      ///< 12x12 stiffness matrix
        const ChVector3d& eps_u,                            ///< strains along \e u direction
        const ChVector3d& eps_v,                            ///< strains along \e v direction
        const ChVector3d& kur_u,                            ///< curvature along \e u direction
        const ChVector3d& kur_v,                            ///< curvature along \e v direction
        const double z_inf,                                 ///< layer lower z value (along thickness coord)
        const double z_sup,                                 ///< layer upper z value (along thickness coord)
        const double angle,                                 ///< layer angle respect to x (if needed)
        const ChShellReissnerInternalData* mdata = nullptr  ///< material internal variables, at this point, if any,
                                                            ///< including {p_strain_e, p_strain_k, p_strain_acc}
    );

    /// Set the elasticity model for this section.
    /// By default it uses a simple centered linear elastic model, but you can set more complex models.
    void SetElasticity(std::shared_ptr<ChElasticityReissner> melasticity);

    /// Get the elasticity model for this section.
    /// Use this function to access parameters such as stiffness, Young modulus, etc.
    /// By default it uses a simple centered linear elastic model.
    std::shared_ptr<ChElasticityReissner> GetElasticity() { return this->elasticity; }

    /// Set the plasticity model for this section.
    /// This is independent from the elasticity model.
    /// Note that by default there is no plasticity model,
    /// so by default plasticity never happens.
    void SetPlasticity(std::shared_ptr<ChPlasticityReissner> mplasticity);

    /// Get the elasticity model for this section, if any.
    /// Use this function to access parameters such as yeld limit, etc.
    std::shared_ptr<ChPlasticityReissner> GetPlasticity() { return this->plasticity; }

    /// Set the damping model for this section.
    /// By default no damping.
    void SetDamping(std::shared_ptr<ChDampingReissner> mdamping);

    /// Get the damping model for this section.
    /// By default no damping.
    std::shared_ptr<ChDampingReissner> GetDamping() { return this->damping; }

    /// Set the density of the shell (kg/m^3)
    void SetDensity(double md) { this->density = md; }
    double GetDensity() const { return this->density; }

  private:
    std::shared_ptr<ChElasticityReissner> elasticity;
    std::shared_ptr<ChPlasticityReissner> plasticity;
    std::shared_ptr<ChDampingReissner> damping;

    double density;
};

//------------------------------------------------------------------

//
// ONLY FOR BACKWARD COMPATIBILITY
//

/// For backward compatibility only!
/// New approach: create a ChElasticityReissnerOrthotropic and create a ChMaterialShellReissner by passing the
/// elasticity as a parameter.

class ChApi ChMaterialShellReissnerIsothropic : public ChMaterialShellReissner {
  public:
    /// Construct an isotropic material.
    ChMaterialShellReissnerIsothropic(double mdensity,     ///< material density
                                      double E,            ///< Young's modulus
                                      double nu,           ///< Poisson ratio
                                      double alpha = 1.0,  ///< shear factor
                                      double beta = 0.1    ///< torque factor
                                      )
        : ChMaterialShellReissner(chrono_types::make_shared<ChElasticityReissnerIsothropic>(E, nu, alpha, beta)) {
        this->SetDensity(mdensity);
    }
};

/// For backward compatibility only!
/// New approach: create a ChElasticityReissnerOrthotropic and create a ChMaterialShellReissner by passing the
/// elasticity as a parameter.

class ChApi ChMaterialShellReissnerOrthotropic : public ChMaterialShellReissner {
  public:
    /// Construct an orthotropic material
    ChMaterialShellReissnerOrthotropic(double mdensity,  ///< material density
                                       double m_E_x,     ///< Young's modulus on x
                                       double m_E_y,     ///< Young's modulus on y
                                       double m_nu_xy,   ///< Poisson ratio xy (for yx it holds: nu_yx*E_x = nu_xy*E_y)
                                       double m_G_xy,    ///< Shear modulus, in plane
                                       double m_G_xz,    ///< Shear modulus, transverse
                                       double m_G_yz,    ///< Shear modulus, transverse
                                       double m_alpha = 1.0,  ///< shear factor
                                       double m_beta = 0.1    ///< torque factor
                                       )
        : ChMaterialShellReissner(chrono_types::make_shared<ChElasticityReissnerOrthotropic>(m_E_x,
                                                                                             m_E_y,
                                                                                             m_nu_xy,
                                                                                             m_G_xy,
                                                                                             m_G_xz,
                                                                                             m_G_yz,
                                                                                             m_alpha,
                                                                                             m_beta)) {
        this->SetDensity(mdensity);
    }

    /// Construct an orthotropic material as sub case isotropic
    ChMaterialShellReissnerOrthotropic(double mdensity,       ///< material density
                                       double m_E,            ///< Young's modulus on x
                                       double m_nu,           ///< Poisson ratio
                                       double m_alpha = 1.0,  ///< shear factor
                                       double m_beta = 0.1    ///< torque factor
                                       )
        : ChMaterialShellReissner(
              chrono_types::make_shared<ChElasticityReissnerOrthotropic>(m_E, m_nu, m_alpha, m_beta)) {
        this->SetDensity(mdensity);
    }
};

/// @} fea_elements

}  // end of namespace fea
}  // end of namespace chrono

#endif
