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

#ifndef CHBEAMSECTION_H
#define CHBEAMSECTION_H

#include "chrono_fea/ChApiFEA.h"
#include "chrono/motion_functions/ChFunction.h"

namespace chrono {
namespace fea {





//  forward 
class ChBeamSectionTimoshenko;

/// Base class for elasticity of beam sections of Timoshenko type,
/// where {F,M}=f({epsilon,kappa}) is a 6-dimensional function of
/// epsilon traction/shear and curvatures kappa.
/// This can be shared between multiple beams.

class ChApiFea ChElasticityTimoshenko {
public:
	

	ChElasticityTimoshenko() {
		section = nullptr;
	}

	virtual ~ChElasticityTimoshenko() {}


		/// Compute the generalized cut force and cut torque, 
		/// given actual deformation and curvature. 
		/// This MUST be implemented by subclasses.
	virtual void ComputeStress(
		ChVector<>& stress_n,      ///< return the local stress (generalized force), x component = traction along beam
		ChVector<>& stress_m,      ///< return the local stress (generalized torque), x component = torsion torque along beam
		const ChVector<>& strain_e, ///< the local strain (deformation part): x= elongation, y and z are shear
		const ChVector<>& strain_k  ///< the local strain (curvature part), x= torsion, y and z are line curvatures
	) = 0;

		/// Compute the 6x6 tangent material stiffness matrix [Km]=d\sigma/d\epsilon, 
		/// given actual deformation and curvature (if needed).
		/// This must be overridden by subclasses if an analytical solution is
		/// known (preferred for high performance), otherwise the base behaviour here is to compute
		/// [Km] by numerical differentiation calling ComputeStress() multiple times.
	virtual void ComputeStiffnessMatrix(
		ChMatrixDynamic<>& K,       ///< return the 6x6 material stiffness matrix values here
		const ChVector<>& strain_e, ///< the local strain (deformation part): x= elongation, y and z are shear
		const ChVector<>& strain_k  ///< the local strain (curvature part), x= torsion, y and z are line curvatures
	) {
		double epsi = 1e-6;
		double invepsi = 1.0 / epsi;
		ChVector<> astress_n;
		ChVector<> astress_m;
		ChVector<> bstress_n;
		ChVector<> bstress_m;
		ChVector<> strain_e_inc = strain_e;
		ChVector<> strain_k_inc = strain_k;
		this->ComputeStress(astress_n, astress_m, strain_e, strain_k);
		for (int i = 0;i < 2; ++i) {
			strain_e_inc[i] += epsi;
			this->ComputeStress(bstress_n, bstress_m, strain_e_inc, strain_k_inc);
			K.PasteVector((bstress_n - astress_n)*invepsi, 0, i);
			K.PasteVector((bstress_m - astress_m)*invepsi, 3, i);
			strain_e_inc[i] -= epsi;
		}
		for (int i = 0;i < 2; ++i) {
			strain_k_inc[i] += epsi;
			this->ComputeStress(bstress_n, bstress_m, strain_e_inc, strain_k_inc);
			K.PasteVector((bstress_n - astress_n)*invepsi, 0, i+3);
			K.PasteVector((bstress_m - astress_m)*invepsi, 3, i+3);
			strain_k_inc[i] -= epsi;
		}
	}


	/// Shortcut: set parameters at once, given the y and z widths of the beam assumed
	/// with rectangular shape.
	virtual void SetAsRectangularSection(double width_y, double width_z) = 0;

	/// Shortcut: set parameters at once, given the diameter of the beam assumed
	/// with circular shape.
	virtual void SetAsCircularSection(double diameter) = 0;


	ChBeamSectionTimoshenko* section;
};


/// Simple linear elasticity model for a Timoshenko beam, using basic material
/// properties (zz and yy moments of inertia, area, Young modulus, etc.)
/// This can be shared between multiple beams.

class ChApiFea ChElasticityTimoshenkoSimple : public ChElasticityTimoshenko {
public:
	double Iyy;
	double Izz;
	double J;
	double G;
	double E;
	
	double rdamping;
	double Ks_y;
	double Ks_z;

	ChElasticityTimoshenkoSimple() {
		E = 0.01e9;                 // default E stiffness: (almost rubber)
		SetGwithPoissonRatio(0.3);  // default G (low poisson ratio)

		SetAsRectangularSection(0.01, 0.01);  // defaults Area, Ixx, Iyy, Ks_y, Ks_z, J

		rdamping = 0.01;  // default Rayleigh damping.
	}

	virtual ~ChElasticityTimoshenkoSimple() {}

	/// Set the Iyy moment of inertia of the beam (for flexion about y axis)
	/// Note: some textbook calls this Iyy as Iz
	void SetIyy(double ma) { this->Iyy = ma; }
	double GetIyy() const { return this->Iyy; }

	/// Set the Izz moment of inertia of the beam (for flexion about z axis)
	/// Note: some textbook calls this Izz as Iy
	void SetIzz(double ma) { this->Izz = ma; }
	double GetIzz() const { return this->Izz; }

	/// Set the J torsion constant of the beam (for torsion about x axis)
	void SetJ(double ma) { this->J = ma; }
	double GetJ() const { return this->J; }

	/// Set the Timoshenko shear coefficient Ks for y shear, usually about 0.8,
	/// (for elements that use this, ex. the Timoshenko beams, or Reddy's beams)
	void SetKsy(double ma) { this->Ks_y = ma; }
	double GetKsy() const { return this->Ks_y; }

	/// Set the Timoshenko shear coefficient Ks for z shear, usually about 0.8,
	/// (for elements that use this, ex. the Timoshenko beams, or Reddy's beams)
	void SetKsz(double ma) { this->Ks_z = ma; }
	double GetKsz() const { return this->Ks_z; }

	/// Shortcut: set Area, Ixx, Iyy, Ksy, Ksz and J torsion constant
	/// at once, given the y and z widths of the beam assumed
	/// with rectangular shape.
	virtual void SetAsRectangularSection(double width_y, double width_z) override;

	/// Shortcut: set Area, Ixx, Iyy, Ksy, Ksz and J torsion constant
	/// at once, given the diameter of the beam assumed
	/// with circular shape.
	virtual void SetAsCircularSection(double diameter) override;

	/// Set E, the Young elastic modulus (N/m^2)
	void SetYoungModulus(double mE) { this->E = mE; }
	double GetYoungModulus() const { return this->E; }

	/// Set G, the shear modulus
	void SetGshearModulus(double mG) { this->G = mG; }
	double GetGshearModulus() const { return this->G; }

	/// Set G, the shear modulus, given current E and the specified Poisson ratio
	void SetGwithPoissonRatio(double mpoisson) { this->G = this->E / (2.0 * (1.0 + mpoisson)); }

	/// Set the Rayleigh damping ratio r (as in: R = r * K ), to do: also mass-proportional term
	void SetBeamRaleyghDamping(double mr) { this->rdamping = mr; }
	double GetBeamRaleyghDamping() { return this->rdamping; }

	// Interface to base:

	/// Compute the generalized cut force and cut torque. 
	virtual void ComputeStress(
		ChVector<>& stress_n,      ///< return the local stress (generalized force), x component = traction along beam
		ChVector<>& stress_m,      ///< return the local stress (generalized torque), x component = torsion torque along beam
		const ChVector<>& strain_e, ///< the local strain (deformation part): x= elongation, y and z are shear
		const ChVector<>& strain_k  ///< the local strain (curvature part), x= torsion, y and z are line curvatures
	) override;

	/// Compute the 6x6 tangent material stiffness matrix [Km] =d\sigma/d\epsilon
	virtual void ComputeStiffnessMatrix(
		ChMatrixDynamic<>& K,       ///< return the 6x6 stiffness matrix
		const ChVector<>& strain_e, ///< the local strain (deformation part): x= elongation, y and z are shear
		const ChVector<>& strain_k  ///< the local strain (curvature part), x= torsion, y and z are line curvatures
	) override;
};



/// Elasticity for a beam section in 3D, along with basic material
/// properties. It also supports the advanced case of
/// Iyy and Izz axes rotated respect reference, centroid with offset
/// from reference, and shear center with offset from reference.
/// This material can be shared between multiple beams.
/// The linear elasticity is uncoupled between shear terms S and axial terms A
/// as to have this stiffness matrix pattern:
///
///  Fx   [A       A A ]   e_x
///  Fy   [  S S S     ]   e_y
///  Fz = [  S S S     ] * e_z
///  Mx   [  S S S     ]   k_x
///  My   [A       A A ]   k_y
///  Mz   [A       A A ]   k_z
///

class ChApiFea ChElasticityTimoshenkoAdvanced : public ChElasticityTimoshenkoSimple {
public:
	double alpha;  // Rotation of Izz Iyy respect to reference section, centered on line x
	double Cy;     // Centroid, respect to reference section (elastic center, tension center)
	double Cz;
	double beta;   // Rotation of shear reference section, centered on line x
	double Sy;     // Shear center, respect to reference section
	double Sz;

	ChElasticityTimoshenkoAdvanced() {
		alpha = 0;
		Cy = 0;
		Cz = 0;
		beta = 0;
		Sy = 0;
		Sz = 0;
	}

	virtual ~ChElasticityTimoshenkoAdvanced() {}

	/// "Elastic reference": set alpha, the rotation of the section for which the Iyy Izz are
	/// defined, respect to the reference section coordinate system.
	void SetSectionRotation(double ma) { this->alpha = ma; }
	double GetSectionRotation() { return this->alpha; }

	/// "Elastic reference": set the displacement of the centroid C (i.e. the elastic center,
	/// or tension center) respect to the reference section coordinate system.
	void SetCentroid(double my, double mz) {
		this->Cy = my;
		this->Cz = mz;
	}
	double GetCentroidY() { return this->Cy; }
	double GetCentroidZ() { return this->Cz; }

	/// "Shear reference": set beta, the rotation of the section for shear decoupling, respect to
	/// the reference section coordinate system. Usually it is same as alpha.
	void SetShearRotation(double mb) { this->beta = mb; }
	double GetShearRotation() { return this->beta; }

	/// "Shear reference": set the displacement of the shear center S
	/// respect to the reference beam line. For shapes like rectangles,
	/// rotated rectangles, etc., it corresponds to the centroid C, but
	/// for "L" shaped or "U" shaped beams this is not always true, and
	/// the shear center accounts for torsion effects when a shear force is applied.
	void SetShearCenter(double my, double mz) {
		this->Sy = my;
		this->Sz = mz;
	}
	double GetShearCenterY() { return this->Sy; }
	double GetShearCenterZ() { return this->Sz; }

	// Interface to base:

	/// Compute the generalized cut force and cut torque. 
	virtual void ComputeStress(
		ChVector<>& stress_n,      ///< return the local stress (generalized force), x component = traction along beam
		ChVector<>& stress_m,      ///< return the local stress (generalized torque), x component = torsion torque along beam
		const ChVector<>& strain_e, ///< the local strain (deformation part): x= elongation, y and z are shear
		const ChVector<>& strain_k  ///< the local strain (curvature part), x= torsion, y and z are line curvatures
	) override;

	/// Compute the 6x6 tangent material stiffness matrix [Km] =d\sigma/d\epsilon
	virtual void ComputeStiffnessMatrix(
		ChMatrixDynamic<>& K,       ///< return the 6x6 stiffness matrix
		const ChVector<>& strain_e, ///< the local strain (deformation part): x= elongation, y and z are shear
		const ChVector<>& strain_k  ///< the local strain (curvature part), x= torsion, y and z are line curvatures
	) override;
};





/// Base class for internal variables of materials. 
/// Especially useful for plasticity, where internal variables are used 
/// to carry information on plastic flow, accumulated flow, etc.

class ChApiFea ChBeamMaterialInternalData {
public:
	ChBeamMaterialInternalData() :
		p_strain_acc(0)
	{};

	virtual ~ChBeamMaterialInternalData() {};

	virtual void Copy(const ChBeamMaterialInternalData& other) {
		p_strain_acc = other.p_strain_acc;
	}

	double p_strain_acc;  // accumulated flow,  \overbar\eps^p  in Neto-Owen book
};


/// Base class for plasticity of beam sections of Timoshenko type.
/// This can be shared between multiple beams.

class ChApiFea ChPlasticityTimoshenko {
public:

	ChPlasticityTimoshenko() {
		section = nullptr;
		nr_yeld_tolerance = 1e-7;
		nr_yeld_maxiters = 5;
	}

	virtual ~ChPlasticityTimoshenko() {}

	// Given a trial strain, it computes the effective stress and strain by 
	// clamping against the yeld surface. An implicit return mapping integration
	// step is computed automatically per each call of this function.
	// Note: for the elastic part, it must use the elasticity model in this->section->elasticity.
	// If not beyond yeld, simply: 
	//      elastic strain = tot strain - plastic strain
	// If its beyond yeld:
	//      elastic strain is computed by fully implicit strain integration with return mapping,
	//      and plastic strains in "data_new" are updated.
	// Returns true if it had to do return mapping, false if it was in elastic regime
	virtual bool ComputeStressWithReturnMapping(
		ChVector<>& stress_n,			 ///< return the local stress (generalized force), x component = traction along beam
		ChVector<>& stress_m,			 ///< return the local stress (generalized torque), x component = torsion torque along beam
		ChVector<>& e_strain_e_new,      ///< return updated elastic strain (deformation part)
		ChVector<>& e_strain_k_new,      ///< return updated elastic strain (curvature part)
		ChBeamMaterialInternalData& data_new,///< return updated material internal variables, at this point, including {p_strain_e, p_strain_k, p_strain_acc}
		const ChVector<>& tot_strain_e,  ///< trial tot strain (deformation part): x= elongation, y and z are shear
		const ChVector<>& tot_strain_k,  ///< trial tot strain (curvature part), x= torsion, y and z are line curvatures
		const ChBeamMaterialInternalData& data ///< trial material internal variables, at this point, including {p_strain_e, p_strain_k, p_strain_acc}
	) = 0;

	/// Compute the 6x6 tangent material stiffness matrix [Km]=d\sigma/d\epsilon, 
	/// given actual internal data and deformation and curvature (if needed). If in 
	/// plastic regime, uses elastoplastic matrix, otherwise uses elastic. 
	/// This must be overridden by subclasses if an analytical solution is
	/// known (preferred for high performance), otherwise the base behaviour here is to compute
	/// [Km] by numerical differentiation calling ComputeStressWithReturnMapping() multiple times.
	virtual void ComputeStiffnessMatrixElastoplastic(
		ChMatrixDynamic<>& K,       ///< return the 6x6 material stiffness matrix values here
		const ChVector<>& strain_e, ///< tot strain (deformation part): x= elongation, y and z are shear
		const ChVector<>& strain_k, ///< tot strain (curvature part), x= torsion, y and z are line curvatures
		const ChBeamMaterialInternalData& data ///< get & return updated material internal variables, at this point, including {p_strain_e, p_strain_k, p_strain_acc}
	);

	// Populate a vector with the appropriate ChBeamSectionPlasticity data structures.
	// Children classes may override this. By default uses ChBeamMaterialInternalData for basic plasticity.
	// Thanks to unique_ptr there is no need to call delete for the pointed objects.
	virtual void CreatePlasticityData(
		int numpoints,
		std::vector< std::unique_ptr<ChBeamMaterialInternalData> >& plastic_data
	) {
		plastic_data.resize(numpoints);
		for (int i = 0; i < numpoints; ++i) {
			plastic_data[i] = std::unique_ptr<ChBeamMaterialInternalData>(new ChBeamMaterialInternalData());
		}
	};


	/// Shortcut: set parameters at once, given the y and z widths of the beam assumed
	/// with rectangular shape.
	virtual void SetAsRectangularSection(double width_y, double width_z) = 0;

	/// Shortcut: set parameters at once, given the diameter of the beam assumed
	/// with circular shape.
	virtual void SetAsCircularSection(double diameter) = 0;

	ChBeamSectionTimoshenko* section;
	double nr_yeld_tolerance;
	int nr_yeld_maxiters;
};




/// Internal variables for basic lumped plasticity in Timoshenko beams
class ChApiFea ChInternalDataLumpedTimoshenko : public ChBeamMaterialInternalData {
public:
	ChInternalDataLumpedTimoshenko()
	{};

	virtual ~ChInternalDataLumpedTimoshenko() {};

	virtual void Copy(const ChBeamMaterialInternalData& other) override {
		ChBeamMaterialInternalData::Copy(other);

		if (auto mother = dynamic_cast<const ChInternalDataLumpedTimoshenko*>(&other)) {
			p_strain_e = mother->p_strain_e;
			p_strain_k = mother->p_strain_k;
		}
	}


	ChVector<> p_strain_e;
	ChVector<> p_strain_k;
};


/// Lumped plasticity of Timoshenko-type beams.
/// This can be shared between multiple beams.

class ChApiFea ChPlasticityTimoshenkoLumped : public ChPlasticityTimoshenko {
public:

	ChPlasticityTimoshenkoLumped() {
		// Default: linear isotropic constant hardening
		n_yeld_x = std::make_shared<ChFunction_Const>(1000);
		n_beta_x = std::make_shared<ChFunction_Const>(0);
	}

	virtual ~ChPlasticityTimoshenkoLumped() {}

	// Given a trial strain, it computes the effective stress and strain by 
	// clamping against the yeld surface. An implicit return mapping integration
	// step is computed automatically per each call of this function.
	// Returns true if it had to do return mapping, false if it was in elastic regime
	virtual bool ComputeStressWithReturnMapping(
		ChVector<>& stress_n,			 ///< return the local stress (generalized force), x component = traction along beam
		ChVector<>& stress_m,			 ///< return the local stress (generalized torque), x component = torsion torque along beam
		ChVector<>& e_strain_e_new,      ///< return updated elastic strain (deformation part)
		ChVector<>& e_strain_k_new,      ///< return updated elastic strain (curvature part)
		ChBeamMaterialInternalData& data_new,///< return updated material internal variables, at this point, including {p_strain_e, p_strain_k, p_strain_acc}
		const ChVector<>& tot_strain_e,  ///< trial tot strain (deformation part): x= elongation, y and z are shear
		const ChVector<>& tot_strain_k,  ///< trial tot strain (curvature part), x= torsion, y and z are line curvatures
		const ChBeamMaterialInternalData& data ///< current material internal variables, at this point, including {p_strain_e, p_strain_k, p_strain_acc}
	) override;

	/// Compute the 6x6 tangent material stiffness matrix [Km]=d\sigma/d\epsilon, 
	/// given actual internal data and deformation and curvature (if needed). If in 
	/// plastic regime, uses elastoplastic matrix, otherwise uses elastic. 
	/*
	virtual void ComputeStiffnessMatrixElastoplastic(
		ChMatrixDynamic<>& K,       ///< return the 6x6 material stiffness matrix values here
		const ChVector<>& strain_e, ///< tot strain (deformation part): x= elongation, y and z are shear
		const ChVector<>& strain_k, ///< tot strain (curvature part), x= torsion, y and z are line curvatures
		ChBeamMaterialInternalData& data ///< get & return updated material internal variables, at this point, including {p_strain_e, p_strain_k, p_strain_acc}
	) override {
		...
	}
	*/

	// Populate a vector with the appropriate ChBeamSectionPlasticity data structures.
	// Children classes may override this. By default uses ChBeamMaterialInternalData for basic plasticity.
	// Thanks to unique_ptr there is no need to call delete for the pointed objects.
	virtual void CreatePlasticityData(
		int numpoints,
		std::vector< std::unique_ptr<ChBeamMaterialInternalData> >& plastic_data
	) {
		plastic_data.resize(numpoints);
		for (int i = 0; i < numpoints; ++i) {
			plastic_data[i] = std::unique_ptr<ChBeamMaterialInternalData>(new ChInternalDataLumpedTimoshenko());
		}
	};


	virtual void SetAsRectangularSection(double width_y, double width_z) override {};

	virtual void SetAsCircularSection(double diameter) override {};


	std::shared_ptr<ChFunction> n_yeld_x; // sigma_y(p_strain_acc)
	std::shared_ptr<ChFunction> n_beta_x; // beta(p_strain_acc)
};






/// Base class for properties of all beam sections.
/// A beam section can be shared between multiple beams.
/// A beam section contains the models for elasticity, plasticity, damping, etc.

class ChApiFea ChBeamSectionProperties {
public:
	double y_drawsize;
	double z_drawsize;
	bool is_circular;

	double Area;
	double density;


	ChBeamSectionProperties() {
		this->y_drawsize = 0.01;
		this->z_drawsize = 0.01;
		this->is_circular = 0;

		this->Area = 1;
		this->density = 1000;
	}

	virtual ~ChBeamSectionProperties() {}

	/// Sets the rectangular thickness of the beam on y and z directions,
	/// only for drawing/rendering purposes (these thickness values do NOT
	/// have any meaning at a physical level, use SetAsRectangularSection()
	////instead if you want to affect also the inertias of the beam section).
	void SetDrawThickness(double thickness_y, double thickness_z) {
		this->y_drawsize = thickness_y;
		this->z_drawsize = thickness_z;
	}
	double GetDrawThicknessY() { return this->y_drawsize; }
	double GetDrawThicknessZ() { return this->z_drawsize; }

	/// Tells if the section must be drawn as a circular
	/// section instead than default rectangular
	bool IsCircular() { return is_circular; }
	/// Set if the section must be drawn as a circular
	/// section instead than default rectangular
	void SetCircular(bool ic) { is_circular = ic; }

	/// Sets the radius of the beam if in 'circular section' draw mode,
	/// only for drawing/rendering purposes (this radius value do NOT
	/// have any meaning at a physical level, use ChBeamSectionBasic::SetAsCircularSection()
	////instead if you want to affect also the inertias of the beam section).
	void SetDrawCircularRadius(double draw_rad) { this->y_drawsize = draw_rad; }
	double GetDrawCircularRadius() { return this->y_drawsize; }


	/// Set the cross sectional area A of the beam (m^2)
	void SetArea(const double ma) { this->Area = ma; }
	double GetArea() const { return this->Area; }

	/// Set the density of the beam (kg/m^3)
	void SetDensity(double md) { this->density = md; }
	double GetDensity() const { return this->density; }


	/// Shortcut: set elastic and plastic constants
	/// at once, given the y and z widths of the beam assumed
	/// with rectangular shape.
	virtual void SetAsRectangularSection(double width_y, double width_z) = 0;

	/// Shortcut: set elastic and plastic constants
	/// at once, given the diameter of the beam assumed
	/// with circular shape.
	virtual void SetAsCircularSection(double diameter) = 0;

};





/// Base class for properties of beam sections of Timoshenko type (with shear too).
/// A beam section can be shared between multiple beams.
/// A beam section contains the models for elasticity, plasticity, damping, etc.

class ChApiFea ChBeamSectionTimoshenko : public ChBeamSectionProperties {
  public:

	  ChBeamSectionTimoshenko(
					std::shared_ptr<ChElasticityTimoshenko> melasticity,  /// the elasticity model for this section, ex.ChElasticityTimoshenkoSimple 
					std::shared_ptr<ChPlasticityTimoshenko> mplasticity = nullptr /// the plasticity model for this section, if any
	  ) {
		this->SetElasticity(melasticity);
		
		if (mplasticity)
			this->SetPlasticity(mplasticity);
    }

    virtual ~ChBeamSectionTimoshenko() {}


	/// Compute the generalized cut force and cut torque, given the actual generalized section strain
	/// expressed as deformation vector e and curvature k, that is: {F,M}=f({e,k}), and 
	/// given the actual material state required for plasticity if any (but if mdata=nullptr, 
	/// computes only the elastic force).
	/// If there is plasticity, the stress is clamped by automatically performing an implicit return mapping.
	/// In sake of generality, if possible this is the function that should be used by beam finite elements 
	/// to compute internal forces, ex.by some Gauss quadrature. 
	virtual void ComputeStress(
		ChVector<>& stress_n, ///< return the stress (generalized force F), x component = traction along beam
		ChVector<>& stress_m, ///< return the stress (generalized torque M), x component = torsion torque along beam
		const ChVector<>& strain_e, ///< strain (deformation part e): x= elongation, y and z are shear
		const ChVector<>& strain_k, ///< strain (curvature part k), x= torsion, y and z are line curvatures
		ChBeamMaterialInternalData* mdata_new = nullptr,  ///< return updated material internal variables, at this point, including {p_strain_e, p_strain_k, p_strain_acc}
		const ChBeamMaterialInternalData* mdata = nullptr ///< current material internal variables, at this point, including {p_strain_e, p_strain_k, p_strain_acc}
	) {
		if (!plasticity || !mdata || !mdata_new)
			this->elasticity->ComputeStress(stress_n, stress_m, strain_e, strain_k);
		else {
			ChVector<> e_strain_e; // probably not needed as computable later as e_strain_e = strain_e - data.p_strain_e
			ChVector<> e_strain_k; // probably not needed   "  "
			this->plasticity->ComputeStressWithReturnMapping(stress_n, stress_m, e_strain_e, e_strain_k, *mdata_new, strain_e, strain_k, *mdata);
		}
	}

	/// Compute the 6x6 tangent material stiffness matrix [Km] =d\sigma/d\epsilon
	/// at a given strain state, and at given internal data state (if mdata=nullptr, 
	/// computes only the elastic tangent stiffenss, regardless of plasticity).
	virtual void ComputeStiffnessMatrix(
		ChMatrixDynamic<>& K, ///< returns the 6x6 stiffness matrix
		const ChVector<>& strain_e, ///< strain (deformation part): x= elongation, y and z are shear
		const ChVector<>& strain_k, ///< strain (curvature part), x= torsion, y and z are line curvatures
		const ChBeamMaterialInternalData* mdata = nullptr ///< material internal variables, at this point, if any, including {p_strain_e, p_strain_k, p_strain_acc}
	) {
		if (!plasticity || !mdata)
			this->elasticity->ComputeStiffnessMatrix(K, strain_e, strain_k);
		else {
			this->plasticity->ComputeStiffnessMatrixElastoplastic(K, strain_e, strain_k, *mdata);
		}
	}


	/// Set the elasticity model for this section.
	/// By default it uses a simple centered linear elastic model, but you can set more complex models.
	void SetElasticity(std::shared_ptr<ChElasticityTimoshenko> melasticity) {
		elasticity = melasticity;
		elasticity->section = this;
	}

	/// Get the elasticity model for this section. 
	/// Use this function to access parameters such as stiffness, Young modulus, etc. 
	/// By default it uses a simple centered linear elastic model.
	std::shared_ptr<ChElasticityTimoshenko> GetElasticity() {
		return this->elasticity;
	}


	/// Set the plasticity model for this section.
	/// This is independent from the elasticity model.
	/// Note that by default there is no plasticity model, 
	/// so by default plasticity never happens.
	void SetPlasticity(std::shared_ptr<ChPlasticityTimoshenko> mplasticity) {
		plasticity = mplasticity;
		mplasticity->section = this;
	}

	/// Get the elasticity model for this section, if any.
	/// Use this function to access parameters such as yeld limit, etc.
	std::shared_ptr<ChPlasticityTimoshenko> GetPlasticity() {
		return this->plasticity;
	}


	/// Shortcut: set elastic and plastic constants
	/// at once, given the y and z widths of the beam assumed
	/// with rectangular shape.
	virtual void SetAsRectangularSection(double width_y, double width_z) {
		this->Area = width_y * width_z;
		this->is_circular = false;
		this->y_drawsize = width_y;
		this->z_drawsize = width_z;

		if (this->elasticity)
			this->elasticity->SetAsRectangularSection(width_y, width_z);
		if (this->plasticity)
			this->plasticity->SetAsRectangularSection(width_y, width_z);
	}

	/// Shortcut: set elastic and plastic constants
	/// at once, given the diameter of the beam assumed
	/// with circular shape.
	virtual void SetAsCircularSection(double diameter) {
		this->Area = CH_C_PI * pow((0.5 * diameter), 2);
		this->is_circular = true;
		this->SetDrawCircularRadius(diameter / 2);

		if (this->elasticity)
			this->elasticity->SetAsCircularSection(diameter);
		if (this->plasticity)
			this->plasticity->SetAsCircularSection(diameter);
	}


private:
	std::shared_ptr<ChElasticityTimoshenko> elasticity;
	std::shared_ptr<ChPlasticityTimoshenko> plasticity;
};







//
// OLD CLASSES FOR BEAM MATERIALS
//


/// Base class for properties of beam sections.
/// A beam section can be shared between multiple beams.
/// A beam section contains the models for elasticity, plasticity, damping, etc.

class ChApiFea ChBeamSection {
public:
	double y_drawsize;
	double z_drawsize;
	bool is_circular;

	ChBeamSection() {
		this->y_drawsize = 0.01;
		this->z_drawsize = 0.01;
		this->is_circular = 0;
	}

	virtual ~ChBeamSection() {}

	/// Sets the rectangular thickness of the beam on y and z directions,
	/// only for drawing/rendering purposes (these thickness values do NOT
	/// have any meaning at a physical level, use ChBeamSectionBasic::SetAsRectangularSection()
	////instead if you want to affect also the inertias of the beam section).
	void SetDrawThickness(double thickness_y, double thickness_z) {
		this->y_drawsize = thickness_y;
		this->z_drawsize = thickness_z;
	}
	double GetDrawThicknessY() { return this->y_drawsize; }
	double GetDrawThicknessZ() { return this->z_drawsize; }

	/// Tells if the section must be drawn as a circular
	/// section instead than default rectangular
	bool IsCircular() { return is_circular; }
	/// Set if the section must be drawn as a circular
	/// section instead than default rectangular
	void SetCircular(bool ic) { is_circular = ic; }

	/// Sets the radius of the beam if in 'circular section' draw mode,
	/// only for drawing/rendering purposes (this radius value do NOT
	/// have any meaning at a physical level, use ChBeamSectionBasic::SetAsCircularSection()
	////instead if you want to affect also the inertias of the beam section).
	void SetDrawCircularRadius(double draw_rad) { this->y_drawsize = draw_rad; }
	double GetDrawCircularRadius() { return this->y_drawsize; }

};




/// Basic geometry for a beam section in 3D, along with basic material
/// properties (zz and yy moments of inertia, area, Young modulus, etc.)
/// This material can be shared between multiple beams.
class ChApiFea ChBeamSectionBasic : public ChBeamSection {
  public:
    double Area;
    double Iyy;
    double Izz;
    double J;
    double G;
    double E;
    double density;
    double rdamping;
    double Ks_y;
    double Ks_z;

    ChBeamSectionBasic() {
        E = 0.01e9;                 // default E stiffness: (almost rubber)
        SetGwithPoissonRatio(0.3);  // default G (low poisson ratio)

        SetAsRectangularSection(0.01, 0.01);  // defaults Area, Ixx, Iyy, Ks_y, Ks_z, J

        density = 1000;   // default density: water
        rdamping = 0.01;  // default Rayleigh damping.
    }

    virtual ~ChBeamSectionBasic() {}

    /// Set the cross sectional area A of the beam (m^2)
    void SetArea(const double ma) { this->Area = ma; }
    double GetArea() const { return this->Area; }

    /// Set the Iyy moment of inertia of the beam (for flexion about y axis)
    /// Note: some textbook calls this Iyy as Iz
    void SetIyy(double ma) { this->Iyy = ma; }
    double GetIyy() const { return this->Iyy; }

    /// Set the Izz moment of inertia of the beam (for flexion about z axis)
    /// Note: some textbook calls this Izz as Iy
    void SetIzz(double ma) { this->Izz = ma; }
    double GetIzz() const { return this->Izz; }

    /// Set the J torsion constant of the beam (for torsion about x axis)
    void SetJ(double ma) { this->J = ma; }
    double GetJ() const { return this->J; }

    /// Set the Timoshenko shear coefficient Ks for y shear, usually about 0.8,
    /// (for elements that use this, ex. the Timoshenko beams, or Reddy's beams)
    void SetKsy(double ma) { this->Ks_y = ma; }
    double GetKsy() const { return this->Ks_y; }

    /// Set the Timoshenko shear coefficient Ks for z shear, usually about 0.8,
    /// (for elements that use this, ex. the Timoshenko beams, or Reddy's beams)
    void SetKsz(double ma) { this->Ks_z = ma; }
    double GetKsz() const { return this->Ks_z; }

    /// Shortcut: set Area, Ixx, Iyy, Ksy, Ksz and J torsion constant
    /// at once, given the y and z widths of the beam assumed
    /// with rectangular shape.
    void SetAsRectangularSection(double width_y, double width_z) {
        this->Area = width_y * width_z;
        this->Izz = (1.0 / 12.0) * width_z * pow(width_y, 3);
        this->Iyy = (1.0 / 12.0) * width_y * pow(width_z, 3);

        // use Roark's formulas for torsion of rectangular sect:
        double t = ChMin(width_y, width_z);
        double b = ChMax(width_y, width_z);
        this->J = b * pow(t, 3) * ((1.0 / 3.0) - 0.210 * (t / b) * (1.0 - (1.0 / 12.0) * pow((t / b), 4)));

        // set Ks using Timoshenko-Gere formula for solid rect.shapes
        double poisson = this->E / (2.0 * this->G) - 1.0;
        this->Ks_y = 10.0 * (1.0 + poisson) / (12.0 + 11.0 * poisson);
        this->Ks_z = this->Ks_y;

        this->is_circular = false;
        this->y_drawsize = width_y;
        this->z_drawsize = width_z;
    }

    /// Shortcut: set Area, Ixx, Iyy, Ksy, Ksz and J torsion constant
    /// at once, given the diameter of the beam assumed
    /// with circular shape.
    void SetAsCircularSection(double diameter) {
        this->Area = CH_C_PI * pow((0.5 * diameter), 2);
        this->Izz = (CH_C_PI / 4.0) * pow((0.5 * diameter), 4);
        this->Iyy = Izz;

        // exact expression for circular beam J = Ixx ,
        // where for polar theorem Ixx = Izz+Iyy
        this->J = Izz + Iyy;

        // set Ks using Timoshenko-Gere formula for solid circular shape
        double poisson = this->E / (2.0 * this->G) - 1.0;
        this->Ks_y = 6.0 * (1.0 + poisson) / (7.0 + 6.0 * poisson);
        this->Ks_z = this->Ks_y;

        this->is_circular = true;
        this->SetDrawCircularRadius(diameter / 2);
    }

    /// Set the density of the beam (kg/m^3)
    void SetDensity(double md) { this->density = md; }
    double GetDensity() const { return this->density; }

    /// Set E, the Young elastic modulus (N/m^2)
    void SetYoungModulus(double mE) { this->E = mE; }
    double GetYoungModulus() const { return this->E; }

    /// Set G, the shear modulus
    void SetGshearModulus(double mG) { this->G = mG; }
    double GetGshearModulus() const { return this->G; }

    /// Set G, the shear modulus, given current E and the specified Poisson ratio
    void SetGwithPoissonRatio(double mpoisson) { this->G = this->E / (2.0 * (1.0 + mpoisson)); }

    /// Set the Rayleigh damping ratio r (as in: R = r * K ), to do: also mass-proportional term
    void SetBeamRaleyghDamping(double mr) { this->rdamping = mr; }
    double GetBeamRaleyghDamping() { return this->rdamping; }
};

/// Geometry for a beam section in 3D, along with basic material
/// properties. It also supports the advanced case of
/// Iyy and Izz axes rotated respect reference, centroid with offset
/// from reference, and shear center with offset from reference.
/// This material can be shared between multiple beams.
class ChApiFea ChBeamSectionAdvanced : public ChBeamSectionBasic {
  public:
    double alpha;  // Rotation of Izz Iyy respect to reference line x
    double Cy;     // Centroid (elastic center, tension center)
    double Cz;
    double Sy;  // Shear center
    double Sz;

    ChBeamSectionAdvanced() {
        alpha = 0;
        Cy = 0;
        Cz = 0;
        Sy = 0;
        Sz = 0;
    }

    virtual ~ChBeamSectionAdvanced() {}

    /// Set the rotation of the section for which the Iyy Izz are
    /// defined.
    void SetSectionRotation(double ma) { this->alpha = ma; }
    double GetSectionRotation() { return this->alpha; }

    /// Set the displacement of the centroid C (i.e. the elastic center,
    /// or tension center) respect to the reference beam line.
    void SetCentroid(double my, double mz) {
        this->Cy = my;
        this->Cz = mz;
    }
    double GetCentroidY() { return this->Cy; }
    double GetCentroidZ() { return this->Cz; }

    /// Set the displacement of the shear center S
    /// respect to the reference beam line. For shapes like rectangles,
    /// rotated rectangles, etc., it corresponds to the centroid C, but
    /// for "L" shaped or "U" shaped beams this is not always true, and
    /// the shear center accounts for torsion effects when a shear force is applied.
    void SetShearCenter(double my, double mz) {
        this->Sy = my;
        this->Sz = mz;
    }
    double GetShearCenterY() { return this->Sy; }
    double GetShearCenterZ() { return this->Sz; }
};

/// Simplified geometry for a 'cable' beam section in 3D, that is a beam
/// without torsional stiffness and with circular section (i.e.same Ixx and Iyy properties).
/// This material can be shared between multiple beams.
class ChApiFea ChBeamSectionCable : public ChBeamSection {
  public:
    double Area;
    double I;
    double E;
    double density;
    double rdamping;

    ChBeamSectionCable() {
        E = 0.01e9;  // default E stiffness: (almost rubber)

        SetDiameter(0.01);  // defaults Area, I

        density = 1000;   // default density: water
        rdamping = 0.01;  // default Rayleigh damping.
    }

    virtual ~ChBeamSectionCable() {}

    /// Set the cross sectional area A of the beam (m^2)
    void SetArea(const double ma) { this->Area = ma; }
    double GetArea() const { return this->Area; }

    /// Set the I moment of inertia of the beam (for flexion about y axis or z axis)
    /// Note: since this simple section assumes circular section, Iyy=Izz=I
    void SetI(double ma) { this->I = ma; }
    double GetI() const { return this->I; }

    /// Shortcut: set Area and I inertia at once,
    /// given the diameter of the beam assumed
    /// with circular shape.
    void SetDiameter(double diameter) {
        this->Area = CH_C_PI * pow((0.5 * diameter), 2);
        this->I = (CH_C_PI / 4.0) * pow((0.5 * diameter), 4);

        this->is_circular = true;
        this->SetDrawCircularRadius(diameter / 2);
    }

    /// Set the density of the beam (kg/m^3)
    void SetDensity(double md) { this->density = md; }
    double GetDensity() const { return this->density; }

    /// Set E, the Young elastic modulus (N/m^2)
    void SetYoungModulus(double mE) { this->E = mE; }
    double GetYoungModulus() const { return this->E; }

    /// Set the Rayleigh damping ratio r (as in: R = r * K ), to do: also mass-proportional term
    void SetBeamRaleyghDamping(double mr) { this->rdamping = mr; }
    double GetBeamRaleyghDamping() { return this->rdamping; }
};

}  // end namespace fea
}  // end namespace chrono

#endif
