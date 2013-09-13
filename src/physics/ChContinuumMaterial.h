#ifndef CHCONTINUUMMATERIAL_H
#define CHCONTINUUMMATERIAL_H

//////////////////////////////////////////////////
//  
//   ChContinuumMaterial.h
//
//   Class for material of elastoplatic continuum
//
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "core/ChApiCE.h"
#include "core/ChMath.h"
#include "core/ChShared.h"
#include "physics/ChTensors.h"

namespace chrono 
{
namespace fem
{



/// Base class for properties of materials 
/// in a continuum. 

class ChApi ChContinuumMaterial : public ChShared
{
private:
	double density;

public:

	ChContinuumMaterial(double mdensity=1000) : density(mdensity) {};

	virtual ~ChContinuumMaterial() {};
	
			/// Set the density of the material, in kg/m^2.
	void   Set_density (double m_density) {density = m_density;}
			/// Get the density of the material, in kg/m^2.
	double Get_density () {return density;}

				/// Method to allow deserializing
	void StreamIN(ChStreamInBinary& mstream);

				/// Method to allow serializing 
	void StreamOUT(ChStreamOutBinary& mstream);

};



/// Class for the basic properties of materials 
/// in an elastic continuum. 
/// This is a base material with isothropic hookean 
/// elasticity.

class ChApi ChContinuumElastic : public ChContinuumMaterial
{
private:

	double E;			// Young Modulus
	double v;			// Poisson ratio
	double G;			// shear modulus
	double l;			// Lame's modulus
	ChMatrixDynamic<> StressStrainMatrix;		//Elasticity (stiffness) matrix		σ = [E] ε

public:

			/// Create a continuum isothropic hookean material. 
			/// Default value for Young elastic modulus is low (like a 
			/// rubber-type material), and same for density.
	ChContinuumElastic(double myoung = 10000000, double mpoisson=0.4, double mdensity=1000);

	virtual ~ChContinuumElastic();
	

			/// Set the Young E elastic modulus, in Pa (N/m^2), as the ratio of the uniaxial 
			/// stress over the uniaxial strain, for hookean materials. Intuitively, the 
			/// tensile pressure on a side of a parallelepiped in order to double its length.
			/// Note that most metal materials require very high values, ex. steel 
			/// has E=210GPa (E=210e9), aluminium E=69e9, and this can cause numerical 
			/// problems if you do not set up the simulation integrator properly.
	void   Set_E (double m_E);
			/// Get the Young E elastic modulus, in Pa (N/m^2).
	double Get_E () {return E;}


			/// Set the Poisson v ratio, as v=-transverse_strain/axial_strain, so
			/// takes into account the 'squeezing' effect of materials that are pulled (so,
			/// if zero, when you push the two sizes of a cube, it won't inflate). Most
			/// materials have some 0<v<0.5, for example steel has v=0.27..0.30, aluminium v=0.33,
			/// rubber=0.49, etc. Note! v=0.5 means perfectly incompressible material, that 
			/// could give problems with some type of solvers.
			/// Setting v also changes G.
	void   Set_v (double m_v);
			/// Get the Young v ratio, as v=-transverse_strain/axial_strain.
	double Get_v () {return v;}

			/// Set the shear modulus G, in Pa (N/m^2), as the ratio of shear stress to 
			/// the shear strain. Setting G also changes Poisson ratio v.
	void   Set_G (double m_G);
			/// Get the shear modulus G, in Pa (N/m^2)
	double Get_G () {return G;}

			/// Get Lamé first parameter (the second is shear modulus, so Get_G() )
	double Get_l () {return l;}

			/// Get bulk modulus (increase of pressure for decrease of volume), in Pa. 
	double Get_BulkModulus () {return E/(3.*(1.-2.*v));}

			/// Get P-wave modulus (if V=speed of propagation of a P-wave, then (M/density)=V^2 ) 
	double Get_WaveModulus () {return E*((1.-v)/(1.+v)*(1.-2.*v));}


			/// Computes Elasticity matrix and stores the value in this->StressStrainMatrix
			/// Note: is performed every time you change a material parameter
	void ComputeStressStrainMatrix();
			/// Get the Elasticity matrix 
	ChMatrixDynamic<> Get_StressStrainMatrix () {return StressStrainMatrix;}


			/// Compute elastic stress from elastic strain
			/// (using column tensors, in Voight notation)
	void ComputeElasticStress(ChStressTensor<>& mstress, const ChStrainTensor<>& mstrain) const
				{
					mstress.XX() = mstrain.XX() * (l+2*G) + mstrain.YY() * l       + mstrain.ZZ() * l;
					mstress.YY() = mstrain.XX() * l		  + mstrain.YY() * (l+2*G) + mstrain.ZZ() * l;
					mstress.ZZ() = mstrain.XX() * l       + mstrain.YY() * l	   + mstrain.ZZ() * (l+2*G);
					mstress.XY() = mstrain.XY() * 2*G;
					mstress.XZ() = mstrain.XZ() * 2*G;
					mstress.YZ() = mstrain.YZ() * 2*G;
				}

			/// Compute elastic strain from elastic stress
			/// (using column tensors, in Voight notation)
	void ComputeElasticStrain(ChStrainTensor<>& mstrain, const ChStressTensor<>& mstress) const
				{
					double invE = 1./E;
					double invhG = 0.5/G;
					mstrain.XX() = invE*( mstress.XX()		- mstress.YY() * v	- mstress.ZZ() * v);
					mstrain.YY() = invE*(-mstress.XX() * v	+ mstress.YY()		- mstress.ZZ() * v);
					mstrain.ZZ() = invE*(-mstress.XX() * v	- mstress.YY() * v	+ mstress.ZZ()    );
					mstrain.XY() = mstress.XY() * invhG;
					mstrain.XZ() = mstress.XZ() * invhG;
					mstrain.YZ() = mstress.YZ() * invhG;
				}


			//
			// STREAMING
			//


				/// Method to allow deserializing a persistent binary archive (ex: a file)
				/// into transient data.
	void StreamIN(ChStreamInBinary& mstream);

				/// Method to allow serializing transient data into a persistent
				/// binary archive (ex: a file).
	void StreamOUT(ChStreamOutBinary& mstream);


};





/// Class for all elastic materials that can undergo plastic flow
/// Defines simply some interface functions.

class ChApi ChContinuumElastoplastic : public ChContinuumElastic
{
public:
	ChContinuumElastoplastic(double myoung = 10000000, double mpoisson=0.4, double mdensity=1000) : ChContinuumElastic(myoung,mpoisson,mdensity) {};

			/// Return a scalar value that is 0 on the yeld surface, <0 inside (elastic), >0 outside (incompatible->plastic flow)
	virtual double ComputeYeldFunction(const ChStressTensor<>& mstress) const = 0;

			/// Compute plastic strain flow (flow derivative dE_plast/dt) from strain,
			/// according to VonMises strain yeld theory. 
	virtual void ComputePlasticStrainFlow(ChStrainTensor<>& mplasticstrainflow, const ChStrainTensor<>& mtotstrain) const =0;

			/// Correct the strain-stress by enforcing that elastic stress must remain on the yeld 
			/// surface, computing a plastic flow to be added to plastic strain while integrating.
	virtual void ComputeReturnMapping(ChStrainTensor<>& mplasticstrainflow, 
									const ChStrainTensor<>&	mincrementstrain, 
									const ChStrainTensor<>& mlastelasticstrain,
									const ChStrainTensor<>& mlastplasticstrain) const = 0;

			/// Set the plastic flow rate, i.e. the 'creep' speed. The lower the value, the slower 
			/// the plastic flow during dynamic simulations, with delayed plasticity. 
	virtual void   Set_flow_rate (double mflow_rate)=0;
			/// Set the plastic flow rate.
	virtual double Get_flow_rate ()=0;
};





/// Class for the basic properties of materials 
/// in an elastoplastic continuum, with strain yeld limit
/// based on Von Mises yeld

class ChApi ChContinuumPlasticVonMises : public ChContinuumElastoplastic
{
private:

	double elastic_yeld;
	double plastic_yeld;
	
	double flow_rate;

public:

			/// Create a continuum isothropic elastoplastic material,
			/// where you can define also plastic and elastic max. stress (yeld limits
			/// for transition elastic->blastic and plastic->fracture).
	ChContinuumPlasticVonMises(double myoung = 10000000, double mpoisson=0.4, double mdensity=1000,
							double melastic_yeld = 0.1, double  mplastic_yeld = 0.2);

	virtual ~ChContinuumPlasticVonMises() {};
	

			/// Set the elastic yeld modulus as the maximum VonMises
			/// equivalent strain that can be withstood by material before
			/// starting plastic flow. It defines the transition elastic->plastic.
	void   Set_elastic_yeld (double melastic_yeld) {elastic_yeld = melastic_yeld;};
			/// Get the elastic yeld modulus.
	double Get_elastic_yeld () {return elastic_yeld;}

			/// Set the plastic yeld modulus as the maximum VonMises
			/// equivalent strain that can be withstood by material before
			/// fracture. It defines the transition plastic->fracture.
	void   Set_plastic_yeld (double mplastic_yeld) {plastic_yeld = mplastic_yeld;};
			/// Get the plastic yeld modulus.
	double Get_plastic_yeld () {return plastic_yeld;}

			/// Set the plastic flow rate. The lower the value, the slower 
			/// the plastic flow during dynamic simulations.
	void   Set_flow_rate (double mflow_rate) {flow_rate = mflow_rate;};
			/// Set the plastic flow rate.
	double Get_flow_rate () {return flow_rate;}


	virtual double ComputeYeldFunction(const ChStressTensor<>& mstress) const;

	virtual void ComputeReturnMapping(ChStrainTensor<>& mplasticstrainflow, 
									const ChStrainTensor<>&	mincrementstrain, 
									const ChStrainTensor<>& mlastelasticstrain,
									const ChStrainTensor<>& mlastplasticstrain) const;


			/// Compute plastic strain flow (flow derivative dE_plast/dt) from strain,
			/// according to VonMises strain yeld theory. 
	void ComputePlasticStrainFlow(ChStrainTensor<>& mplasticstrainflow, const ChStrainTensor<>& mestrain) const;


			//
			// STREAMING
			//

				/// Method to allow deserializing a persistent binary archive (ex: a file)
				/// into transient data.
	void StreamIN(ChStreamInBinary& mstream);

				/// Method to allow serializing transient data into a persistent
				/// binary archive (ex: a file).
	void StreamOUT(ChStreamOutBinary& mstream);

};




/// Class for the basic properties of elastoplastic materials 
/// of Drucker-Prager type, that are useful for simulating
/// soils

class ChApi ChContinuumDruckerPrager : public ChContinuumElastoplastic
{
private:

	double elastic_yeld;
	double alpha;
	double dilatancy;
	double hardening_speed;
	double hardening_limit;
	double flow_rate;

public:

			/// Create a continuum isothropic Drucker-Prager material
	ChContinuumDruckerPrager(double myoung = 10000000, double mpoisson=0.4, double mdensity=1000,
							double melastic_yeld = 0.1, double  malpha = 0.5, double mdilatancy = 0);

	virtual ~ChContinuumDruckerPrager() {};
	

			/// Set the D-P yeld modulus C, for Drucker-Prager
			/// yeld. It defines the transition elastic->plastic.
	void   Set_elastic_yeld (double melastic_yeld) {elastic_yeld = melastic_yeld;};
			/// Get the elastic yeld modulus C
	double Get_elastic_yeld () {return elastic_yeld;}

			/// Set the internal friction coefficient A
	void   Set_alpha (double malpha) {alpha = malpha;};
			/// Get the internal friction coefficient A
	double Get_alpha () {return alpha;}	

			/// Sets the C and A parameters of the Drucker-Prager model
			/// starting from more 'practical' values of inner friction angle phi
			/// and cohesion, as used in the faceted Mohr-Coulomb model. 
			/// Use the optional parameter inner_approx to set if the faceted
			/// Mohr-Coulomg must be approximated with D-P inscribed (default) or circumscribed.
	void   Set_from_MohrCoulomb(double phi, double cohesion, bool inner_approx=true);

			/// Set the plastic flow rate multiplier. The lower the value, the slower 
			/// the plastic flow during dynamic simulations.
	void   Set_flow_rate (double mflow_rate) {flow_rate = mflow_rate;};
			/// Get the flow rate multiplier.
	double Get_flow_rate () {return flow_rate;}

			/// Set the internal dilatancy coefficient (usually 0.. < int.friction)
	void   Set_dilatancy (double mdilatancy) {dilatancy = mdilatancy;};
			/// Get the internal dilatancy coefficient 
	double Get_dilatancy () {return dilatancy;}

			/// Set the hardening limit (usually a bit larger than yeld), or softening
	void   Set_hardening_limit (double mhl) {hardening_limit = mhl;};
			/// Get the hardening limit 
	double Get_hardening_limit () {return hardening_limit;}

			/// Set the hardening inverse speed coeff. for exponential hardening 
			/// (the larger, the slower the hardening or softening process that 
			/// will asymptotycally make yeld = hardening_limit )
	void   Set_hardening_speed (double mhl) {hardening_speed = mhl;};
			/// Get the hardening speed
	double Get_hardening_speed () {return hardening_speed;}


	virtual double ComputeYeldFunction(const ChStressTensor<>& mstress) const;

	virtual void ComputeReturnMapping(ChStrainTensor<>& mplasticstrainflow, 
									const ChStrainTensor<>&	mincrementstrain, 
									const ChStrainTensor<>& mlastelasticstrain,
									const ChStrainTensor<>& mlastplasticstrain) const;

			/// Compute plastic strain flow direction from strain
			/// according to Drucker-Prager. 
	void ComputePlasticStrainFlow(ChStrainTensor<>& mplasticstrainflow, const ChStrainTensor<>& mestrain) const;


			//
			// STREAMING
			//

				/// Method to allow deserializing a persistent binary archive (ex: a file)
				/// into transient data.
	void StreamIN(ChStreamInBinary& mstream);

				/// Method to allow serializing transient data into a persistent
				/// binary archive (ex: a file).
	void StreamOUT(ChStreamOutBinary& mstream);

};


} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____
#endif 
