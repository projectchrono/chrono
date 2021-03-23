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
// Material for Kirchhoff-Love shells (thin shells)
// =============================================================================

#ifndef CHMATERIALSHELLKIRCHHOFF_H
#define CHMATERIALSHELLKIRCHHOFF_H

#include <array>
#include <vector>

#include "chrono/fea/ChElementShell.h"

namespace chrono {
namespace fea {

/// @addtogroup fea_elements
/// @{


//  forward
class ChMaterialShellKirchhoff;



/// Base interface for elasticity of thin shells (Kirchoff-Love shell theory,
/// without shear effects) to be used in a ChMaterialShellKirchhoff.
/// Children classes must implement the ComputeStress function to get
///    {n,m}=f({e,k})
/// that is per-unit-length forces/torques n_11,n22,n12,m_11,m_22,m_12 given the 
/// strains/curvatures e_11 e_22 e_12 k_11 k_22 k_12.
/// Inherited materials do not define any thickness, which should be
/// a property of the element or its layer(s) using this material.

class ChApi ChElasticityKirchhoff {
  public:
    ChElasticityKirchhoff() : section(nullptr) {}

    virtual ~ChElasticityKirchhoff() {}

    /// Compute the generalized force and torque, given actual deformation and curvature.
	/// This MUST be implemented by subclasses.
    virtual void ComputeStress(ChVector<>& n,            ///< forces  n_11, n_22, n_12 (per unit length)
                               ChVector<>& m,            ///< torques m_11, m_22, m_12 (per unit length)
                               const ChVector<>& eps,    ///< strains   e_11, e_22, e_12
                               const ChVector<>& kur,    ///< curvature k_11, k_22, k_12
                               const double z_inf,       ///< layer lower z value (along thickness coord)
                               const double z_sup,       ///< layer upper z value (along thickness coord)
                               const double angle        ///< layer angle respect to x (if needed)
    ) = 0;

    /// Compute the 6x6 stiffness matrix [Km] , that is [ds/de], the tangent of the constitutive relation
    /// stresses/strains. 
    /// By default, it is computed by backward differentiation from the ComputeStress() function,
    /// but inherited classes should better provide an analytical form, if possible.
    virtual void ComputeStiffnessMatrix(ChMatrixRef mC,    ///< tangent matrix
                                 const ChVector<>& eps,    ///< strains   e_11, e_22, e_12
                                 const ChVector<>& kur,    ///< curvature k_11, k_22, k_12
                                 const double z_inf,       ///< layer lower z value (along thickness coord)
                                 const double z_sup,       ///< layer upper z value (along thickness coord)
                                 const double angle        ///< layer angle respect to x (if needed)
    );

    ChMaterialShellKirchhoff* section;
};



/// Isothropic elasticity for thin shells (Kirchoff-Love shell theory,
/// without shear effects) to be used in a ChMaterialShellKirchhoff.
/// This class implements material properties for a layer of a Kirchhoff thin shell,
/// for the case of isotropic linear elastic material.
/// This is probably the material that you need most often when using thin shells.

class ChApi ChElasticityKirchhoffIsothropic : public ChElasticityKirchhoff {
  public:
    /// Construct an isotropic material.
    ChElasticityKirchhoffIsothropic  (double E,            ///< Young's modulus
                                      double nu            ///< Poisson ratio
    );

    /// Return the elasticity moduli
    double Get_E() const { return m_E; }
    /// Return the Poisson ratio
    double Get_nu() const { return m_nu; }
    
    /// The FE code will evaluate this function to compute
    /// per-unit-length forces/torques given the strains/curvatures.
    virtual void ComputeStress(
        ChVector<>& n,            ///< forces  n_11, n_22, n_12 (per unit length)
        ChVector<>& m,            ///< torques m_11, m_22, m_12 (per unit length)
        const ChVector<>& eps,    ///< strains   e_11, e_22, e_12
        const ChVector<>& kur,    ///< curvature k_11, k_22, k_12
        const double z_inf,       ///< layer lower z value (along thickness coord)
        const double z_sup,       ///< layer upper z value (along thickness coord)
        const double angle        ///< layer angle respect to x (if needed) -not used in this, isotropic
    ) override;

    /// Compute 12x12 stiffness matrix [Km] , that is [ds/de], the tangent of the constitutive relation
    /// per-unit-length forces/torques vs generalized strains.
    virtual void ComputeStiffnessMatrix(
        ChMatrixRef mC,           ///< tangent matrix
        const ChVector<>& eps,    ///< strains   e_11, e_22, e_12
        const ChVector<>& kur,    ///< curvature k_11, k_22, k_12
        const double z_inf,       ///< layer lower z value (along thickness coord)
        const double z_sup,       ///< layer upper z value (along thickness coord)
        const double angle        ///< layer angle respect to x (if needed)
    ) override;

  private:
    double m_E;      ///< elasticity moduli
    double m_nu;     ///< Poisson ratio
};


/// Orthotropic elasticity for thin shells (Kirchoff-Love shell theory,
/// without shear effects) to be used in a ChMaterialShellKirchhoff.
/// This class implements material properties for a layer of a Kirchhoff thin shell,
/// for the case of orthotropic linear elastic material.
/// This is useful for laminated shells. One direction can be made softer than the other.
/// Note that the angle and the thickness are defined when adding a material with this elasticity to
/// a shell finite element as a layer.

class ChApi ChElasticityKirchhoffOrthotropic : public ChElasticityKirchhoff {
  public:
    /// Construct an orthotropic material
    ChElasticityKirchhoffOrthotropic  (double m_E_x,    ///< Young's modulus on x
                                       double m_E_y,    ///< Young's modulus on y
                                       double m_nu_xy,  ///< Poisson ratio xy (for yx it holds: nu_yx*E_x = nu_xy*E_y)
                                       double m_G_xy    ///< Shear modulus, in plane
    );
    /// Construct an orthotropic material as sub case isotropic
    ChElasticityKirchhoffOrthotropic  (double m_E,            ///< Young's modulus on x
                                       double m_nu            ///< Poisson ratio
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

    /// The FE code will evaluate this function to compute
    /// per-unit-length forces/torques given the strains/curvatures.
    virtual void ComputeStress(
        ChVector<>& n,            ///< forces  n_11, n_22, n_12 (per unit length)
        ChVector<>& m,            ///< torques m_11, m_22, m_12 (per unit length)
        const ChVector<>& eps,    ///< strains   e_11, e_22, e_12
        const ChVector<>& kur,    ///< curvature k_11, k_22, k_12
        const double z_inf,       ///< layer lower z value (along thickness coord)
        const double z_sup,       ///< layer upper z value (along thickness coord)
        const double angle        ///< layer angle respect to x (if needed) -not used in this, isotropic
    ) override;

    /// Compute 6x6 stiffness matrix [Km] , that is [ds/de], the tangent of the constitutive relation
    /// per-unit-length forces/torques vs generalized strains.
    virtual void ComputeStiffnessMatrix(
        ChMatrixRef mC,           ///< tangent matrix
        const ChVector<>& eps,    ///< strains   e_11, e_22, e_12
        const ChVector<>& kur,    ///< curvature k_11, k_22, k_12
        const double z_inf,       ///< layer lower z value (along thickness coord)
        const double z_sup,       ///< layer upper z value (along thickness coord)
        const double angle        ///< layer angle respect to x (if needed)
    ) override;

  private:
    double E_x;    ///< elasticity moduli
    double E_y;    ///< elasticity moduli
    double nu_xy;  ///< Poisson ratio
    double G_xy;   ///< Shear factor, in plane
    ////double G_xz;   ///< Shear factor, out of plane
    ////double G_yz;   ///< Shear factor, out of plane
};


/// Generic linear elasticity for thin shells (Kirchoff-Love shell theory,
/// without shear effects) to be used in a ChMaterialShellKirchhoff. 
/// This uses a 6x6 matrix [E] from user-input data. The [E] matrix can be 
/// computed from a preprocessing stage using a FEA analysis over a detailed 3D model
/// of a slab of shell, hence recovering the 6x6 matrix in the linear mapping:
/// {n,m}=[E]{e,k}.

class ChApi ChElasticityKirchhoffGeneric : public ChElasticityKirchhoff {
  public:
    ChElasticityKirchhoffGeneric();

    virtual ~ChElasticityKirchhoffGeneric() {}

    /// Access the E matrix, for getting/setting its values.
    /// This is the matrix that defines the linear elastic constitutive model
    /// as it maps  yxz displacements "e" and xyz rotations "k"
    /// to the "n" force and  "m" torque as in
    ///   {n,m}=[E]{e,k}.
    ChMatrixNM<double, 6, 6>& Ematrix() { return this->mE; }


    /// The FE code will evaluate this function to compute
    /// per-unit-length forces/torques given the strains/curvatures.
    virtual void ComputeStress(
        ChVector<>& n,            ///< forces  n_11, n_22, n_12 (per unit length)
        ChVector<>& m,            ///< torques m_11, m_22, m_12 (per unit length)
        const ChVector<>& eps,    ///< strains   e_11, e_22, e_12
        const ChVector<>& kur,    ///< curvature k_11, k_22, k_12
        const double z_inf,       ///< layer lower z value (along thickness coord)
        const double z_sup,       ///< layer upper z value (along thickness coord)
        const double angle        ///< layer angle respect to x (if needed) -not used in this, isotropic
    ) override;

    /// Compute 6x6 stiffness matrix [Km] , that is [ds/de], the tangent of the constitutive relation
    /// per-unit-length forces/torques vs generalized strains.
    virtual void ComputeStiffnessMatrix(
        ChMatrixRef mC,           ///< tangent matrix
        const ChVector<>& eps,    ///< strains   e_11, e_22, e_12
        const ChVector<>& kur,    ///< curvature k_11, k_22, k_12
        const double z_inf,       ///< layer lower z value (along thickness coord)
        const double z_sup,       ///< layer upper z value (along thickness coord)
        const double angle        ///< layer angle respect to x (if needed)
    ) override;

  private:
    ChMatrixNM<double, 6, 6> mE;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};



// ----------------------------------------------------------------------------

/// Base class for internal variables of Kirchhoff thin shells materials.
/// Especially useful for plasticity, where internal variables are used
/// to carry information on plastic flow, accumulated flow, etc.
class ChApi ChShellKirchhoffInternalData {
  public:
    ChShellKirchhoffInternalData() : p_strain_acc(0) {}

    virtual ~ChShellKirchhoffInternalData(){};

    virtual void Copy(const ChShellKirchhoffInternalData& other) { p_strain_acc = other.p_strain_acc; }

    double p_strain_acc;  // accumulated flow,  \overbar\eps^p  in Neto-Owen book
};

/// Base interface for plasticity of thin shells (Kirchoff-Love shell theory,
/// without shear effects) to be used in a ChMaterialShellKirchhoff.
/// Children classes must implement the ComputeStressWithReturnMapping to compute
/// effective stress and strain given a tentative strain that might violate the yeld function.
/// Inherited materials do not define any thickness, which should be
/// a property of the element or its layer(s) using this material.
class ChApi ChPlasticityKirchhoff {
  public:
    ChPlasticityKirchhoff();

    virtual ~ChPlasticityKirchhoff() {}

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
		ChVector<>& n,            ///< forces  n_11, n_22, n_12 (per unit length)
        ChVector<>& m,            ///< torques m_11, m_22, m_12 (per unit length)
        ChShellKirchhoffInternalData& data_new,  ///< updated material internal variables, at this point, including {p_strain_e, p_strain_k, p_strain_acc}
        const ChVector<>& eps_trial,  ///< trial strains    e_11, e_22, e_12 
        const ChVector<>& kur_trial,  ///< trial curvature  k_11, k_22, k_12 
        const ChShellKirchhoffInternalData& data,  ///< trial material internal variables, at this point, including {p_strain_e, p_strain_k, p_strain_acc}
		const double z_inf,       ///< layer lower z value (along thickness coord)
        const double z_sup,       ///< layer upper z value (along thickness coord)
        const double angle        ///< layer angle respect to x (if needed) 
        ) = 0;

    /// Compute the 6x6 tangent material stiffness matrix [Km] = d&sigma;/d&epsilon;,
    /// given actual internal data and deformation and curvature (if needed). If in
    /// plastic regime, uses elastoplastic matrix, otherwise uses elastic.
    /// This must be overridden by subclasses if an analytical solution is
    /// known (preferred for high performance), otherwise the base behaviour here is to compute
    /// [Km] by numerical differentiation calling ComputeStressWithReturnMapping() multiple times.
    virtual void ComputeStiffnessMatrixElastoplastic(
        ChMatrixRef K,        ///< 12x12 material elastoplastic stiffness matrix values here
        const ChVector<>& eps,    ///< strains   e_11, e_22, e_12
        const ChVector<>& kur,    ///< curvature k_11, k_22, k_12
		const ChShellKirchhoffInternalData& data,  ///< updated material internal variables, at this point including {p_strain_e, p_strain_k, p_strain_acc}
        const double z_inf,       ///< layer lower z value (along thickness coord)
        const double z_sup,       ///< layer upper z value (along thickness coord)
        const double angle        ///< layer angle respect to x (if needed) 
    );

    // Populate a vector with the appropriate ChBeamSectionPlasticity data structures.
    // Children classes may override this. By default uses ChBeamMaterialInternalData for basic plasticity.
    // Thanks to unique_ptr there is no need to call delete for the pointed objects.
    virtual void CreatePlasticityData(int numpoints,
                                      std::vector<std::unique_ptr<ChShellKirchhoffInternalData>>& plastic_data);


    ChMaterialShellKirchhoff* section;

    double nr_yeld_tolerance;
    int nr_yeld_maxiters;
};


// ----------------------------------------------------------------------------


/// Base interface for damping of thin shells (Kirchoff-Love shell theory,
/// without shear effects) to be used in a ChMaterialShellKirchhoff.
/// Children classes should implement a ComputeStress function that returns generalized stresses
/// given time derivatives of strains as:
///   {n,m}=f({e',k'})

class ChApi ChDampingKirchhoff {
  public:
    ChDampingKirchhoff() : section(nullptr) {}

    virtual ~ChDampingKirchhoff() {}

    /// Compute the generalized cut force and cut torque, caused by structural damping,
    /// given actual deformation speed and curvature speed.
    /// This MUST be implemented by subclasses.
    virtual void ComputeStress(
        ChVector<>& n,            ///< forces  n_11, n_22, n_12 (per unit length)
        ChVector<>& m,            ///< torques m_11, m_22, m_12 (per unit length)
        const ChVector<>& deps,   ///< time derivative of strains   de_11/dt, de_22/dt, de_12/dt
        const ChVector<>& dkur,   ///< time derivative of curvature dk_11/dt, dk_22/dt, dk_12/dt
        const double z_inf,       ///< layer lower z value (along thickness coord)
        const double z_sup,       ///< layer upper z value (along thickness coord)
        const double angle        ///< layer angle respect to x (if needed)
        ) = 0;

    /// Compute the 6x6 tangent material damping matrix, ie the jacobian [Rm]=dstress/dstrainspeed.
    /// This must be overridden by subclasses if an analytical solution is
    /// known (preferred for high performance), otherwise the base behaviour here is to compute
    /// [Rm] by numerical differentiation calling ComputeStress() multiple times.
    virtual void ComputeDampingMatrix(	ChMatrixRef R,			  ///< 6x6 material damping matrix values here
										const ChVector<>& deps,   ///< time derivative of strains   de_11/dt, de_22/dt, de_12/dt
										const ChVector<>& dkur,   ///< time derivative of curvature dk_11/dt, dk_22/dt, dk_12/dt
										const double z_inf,       ///< layer lower z value (along thickness coord)
										const double z_sup,       ///< layer upper z value (along thickness coord)
										const double angle        ///< layer angle respect to x (if needed) -not used in this, isotropic
    );

    ChMaterialShellKirchhoff* section;
};




/// Simple Rayleight damping of a Kirchhoff shell layer,
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

class ChApi ChDampingKirchhoffRayleigh : public ChDampingKirchhoff {
  public:
		/// Construct the Rayleigh damping model from the stiffness model used by the shell layer.
		/// This is important because the Rayleigh damping is proportional to the stiffness,
		/// so the model must know which is the stiffness matrix of the material.
	    /// Note: melasticity must be alreay set with proper values: its [E] stiffness matrix will be
		/// fetched just once for all.
	ChDampingKirchhoffRayleigh(std::shared_ptr<ChElasticityKirchhoff> melasticity, const double& mbeta = 0);

	virtual ~ChDampingKirchhoffRayleigh() {}

	/// Compute the generalized cut force and cut torque, caused by structural damping,
    /// given actual deformation speed and curvature speed.
	virtual void ComputeStress(
		ChVector<>& n,            ///< forces  n_11, n_22, n_12 (per unit length)
		ChVector<>& m,            ///< torques m_11, m_22, m_12 (per unit length)
		const ChVector<>& deps,   ///< time derivative of strains   de_11/dt, de_22/dt, de_12/dt
		const ChVector<>& dkur,   ///< time derivative of curvature dk_11/dt, dk_22/dt, dk_12/dt
		const double z_inf,       ///< layer lower z value (along thickness coord)
		const double z_sup,       ///< layer upper z value (along thickness coord)
		const double angle        ///< layer angle respect to x (if needed)
	);

    /// Compute the 6x6 tangent material damping matrix, ie the jacobian [Rm]=dstress/dstrainspeed.
    /// In this model, it is beta*[E] where [E] is the 6x6 stiffness matrix at material level, assumed constant
    virtual void ComputeDampingMatrix(	ChMatrixRef R,			  ///< 6x6 material damping matrix values here
										const ChVector<>& deps,   ///< time derivative of strains   de_11/dt, de_22/dt, de_12/dt
										const ChVector<>& dkur,   ///< time derivative of curvature dk_11/dt, dk_22/dt, dk_12/dt
										const double z_inf,       ///< layer lower z value (along thickness coord)
										const double z_sup,       ///< layer upper z value (along thickness coord)
										const double angle        ///< layer angle respect to x (if needed) -not used in this, isotropic
    );

	/// Get the beta Rayleigh parameter (stiffness proportional damping)
    double GetBeta() { return beta; }
    /// Set the beta Rayleigh parameter (stiffness proportional damping)
	void SetBeta(const double mbeta) { beta = mbeta; }


  private:
	std::shared_ptr<ChElasticityKirchhoff> section_elasticity;
    ChMatrixNM<double, 6, 6> E_const; // to store the precomputed stiffness matrix at undeformed unstressed initial state
	double beta;
	bool updated;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};



// ----------------------------------------------------------------------------

/// Material for a single layer of a thin shell (Kirchoff-Love shell theory,
/// i.e. shells without shear effects). 
/// This base implementation assumes that one creates a ChMaterialShellKirchhoff
/// by providing three components: 
///
/// - an elasticity model (from ChElasticityKirchhoff classes)
/// - a plasticity model (optional, from ChPlasticityKirchhoff classes)
/// - a damping model (optional, from ChDampingKirchhoff classes)
///
/// Thickness is defined when adding a ChMaterialShellKirchhoff material as a layer 
/// in a shell finite element.
/// A material can be shared between multiple layers.

class ChApi ChMaterialShellKirchhoff  {
  public:
    ChMaterialShellKirchhoff(std::shared_ptr<ChElasticityKirchhoff> melasticity  ///< elasticity model 
    );

    ChMaterialShellKirchhoff(
        std::shared_ptr<ChElasticityKirchhoff> melasticity,  ///< elasticity model 
        std::shared_ptr<ChPlasticityKirchhoff> mplasticity   ///< plasticity model, if any
    );

    ChMaterialShellKirchhoff(
        std::shared_ptr<ChElasticityKirchhoff> melasticity,  ///< elasticity model
        std::shared_ptr<ChPlasticityKirchhoff> mplasticity,  ///< plasticity model, if any
        std::shared_ptr<ChDampingKirchhoff>    mdamping      ///< damping model, if any
    );

    virtual ~ChMaterialShellKirchhoff() {}

    /// Compute the generalized cut force and cut torque, given the actual generalized section strain
    /// expressed as deformation vector e and curvature k, that is: {n,m}=f({e,k}), and
    /// given the actual material state required for plasticity if any (but if mdata=nullptr,
    /// computes only the elastic force).
    /// If there is plasticity, the stress is clamped by automatically performing an implicit return mapping.
    /// In sake of generality, if possible this is the function that should be used by beam finite elements
    /// to compute internal forces, ex.by some Gauss quadrature.
    virtual void ComputeStress(
        ChVector<>& n,            ///< forces  n_11, n_22, n_12 (per unit length)
        ChVector<>& m,            ///< torques m_11, m_22, m_12 (per unit length)
        const ChVector<>& eps,    ///< strains   e_11, e_22, e_12
        const ChVector<>& kur,    ///< curvature k_11, k_22, k_12
        const double z_inf,       ///< layer lower z value (along thickness coord)
        const double z_sup,       ///< layer upper z value (along thickness coord)
        const double angle,        ///< layer angle respect to x (if needed)
        ChShellKirchhoffInternalData* mdata_new = nullptr,   ///< updated material internal variables, at this
                                                            ///< point, including {p_strain_e, p_strain_k, p_strain_acc}
        const ChShellKirchhoffInternalData* mdata = nullptr  ///< current material internal variables, at this point,
                                                            ///< including {p_strain_e, p_strain_k, p_strain_acc}
    ); 

    /// Compute the 6x6 tangent material stiffness matrix [Km] = d&sigma;/d&epsilon;
    /// at a given strain state, and at given internal data state (if mdata=nullptr,
    /// computes only the elastic tangent stiffenss, regardless of plasticity).
    virtual void ComputeStiffnessMatrix(
        ChMatrixRef K,			  ///< 12x12 stiffness matrix
        const ChVector<>& eps,    ///< strains   e_11, e_22, e_12
        const ChVector<>& kur,    ///< curvature k_11, k_22, k_12
        const double z_inf,       ///< layer lower z value (along thickness coord)
        const double z_sup,       ///< layer upper z value (along thickness coord)
        const double angle,       ///< layer angle respect to x (if needed) 
        const ChShellKirchhoffInternalData* mdata = nullptr  ///< material internal variables, at this point, if any,
                                                            ///< including {p_strain_e, p_strain_k, p_strain_acc}
    );

    /// Set the elasticity model for this section.
    /// By default it uses a simple centered linear elastic model, but you can set more complex models.
    void SetElasticity(std::shared_ptr<ChElasticityKirchhoff> melasticity);

    /// Get the elasticity model for this section.
    /// Use this function to access parameters such as stiffness, Young modulus, etc.
    /// By default it uses a simple centered linear elastic model.
    std::shared_ptr<ChElasticityKirchhoff> GetElasticity() { return this->elasticity; }

    /// Set the plasticity model for this section.
    /// This is independent from the elasticity model.
    /// Note that by default there is no plasticity model,
    /// so by default plasticity never happens.
    void SetPlasticity(std::shared_ptr<ChPlasticityKirchhoff> mplasticity);

    /// Get the elasticity model for this section, if any.
    /// Use this function to access parameters such as yeld limit, etc.
    std::shared_ptr<ChPlasticityKirchhoff> GetPlasticity() { return this->plasticity; }

    /// Set the damping model for this section.
    /// By default no damping.
    void SetDamping(std::shared_ptr<ChDampingKirchhoff> mdamping);

    /// Get the damping model for this section.
    /// By default no damping.
    std::shared_ptr<ChDampingKirchhoff> GetDamping() { return this->damping; }

	/// Set the density of the shell (kg/m^3)
    void SetDensity(double md) { this->density = md; }
    double GetDensity() const { return this->density; }

  private:

    std::shared_ptr<ChElasticityKirchhoff> elasticity;
    std::shared_ptr<ChPlasticityKirchhoff> plasticity;
    std::shared_ptr<ChDampingKirchhoff>    damping;

	double density;
};



/// @} fea_elements

}  // end of namespace fea
}  // end of namespace chrono

#endif
