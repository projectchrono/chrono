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


#include <stdlib.h>

#include "core/ChMath.h"


namespace chrono 
{


/// Base class for stress and strain tensors, in compact Voight notation
/// that is with 6 components in a column. This saves some
/// memory when compared to traditional 3D tensors with three
/// rows and three columns, that are symmetric.

template <class Real = double>
class ChVoightTensor : public ChMatrixNM<Real,6,1>
{
public:
					/// Constructors (default empty)
	ChVoightTensor()  {};

					/// Copy constructor, from a typical 3D rank-two stress or strain tensor (as 3x3 matrix)
	template <class RealB>
	inline ChVoightTensor (const ChMatrix33<RealB>& msource) 
					  {
						 this->ConvertFromMatrix(msource);
					  };

	inline Real& XX()   { return ChMatrix<Real>::ElementN(0); };
	inline const Real& XX () const { return ChMatrix<Real>::ElementN(0); };

	inline Real& YY()   { return ChMatrix<Real>::ElementN(1); };
	inline const Real& YY () const { return ChMatrix<Real>::ElementN(1); };

	inline Real& ZZ()   { return ChMatrix<Real>::ElementN(2); };
	inline const Real& ZZ () const { return ChMatrix<Real>::ElementN(2); };

	inline Real& XY()   { return ChMatrix<Real>::ElementN(3); };
	inline const Real& XY () const { return ChMatrix<Real>::ElementN(3); };

	inline Real& XZ()   { return ChMatrix<Real>::ElementN(4); };
	inline const Real& XZ () const { return ChMatrix<Real>::ElementN(4); };

	inline Real& YZ()   { return ChMatrix<Real>::ElementN(5); };
	inline const Real& YZ () const { return ChMatrix<Real>::ElementN(5); };
		
			/// Convert from a typical 3D rank-two stress or strain tensor (a 3x3 matrix)
	template <class RealB>
	void ConvertFromMatrix(const ChMatrix33<RealB>& msource)
					{
						 XX()= (Real)msource(0,0);
						 YY()= (Real)msource(1,1);
 						 ZZ()= (Real)msource(2,2);
						 XY()= (Real)msource(0,1);
						 XZ()= (Real)msource(0,2);
						 YZ()= (Real)msource(1,2);
					};

			/// Convert to a typical 3D rank-two stress or strain tensor (a 3x3 matrix)
	template <class RealB>
	void ConvertToMatrix(ChMatrix33<RealB>& mdest)
					{
						 mdest(0,0) = (RealB)XX();
						 mdest(1,1) = (RealB)YY();
 						 mdest(2,2) = (RealB)ZZ();
						 mdest(0,1) = (RealB)XY();
						 mdest(0,2) = (RealB)XZ();
						 mdest(1,2) = (RealB)YZ();
						 mdest(1,0) = (RealB)XY();
						 mdest(2,0) = (RealB)XZ();
						 mdest(2,1) = (RealB)YZ();
					};

			/// Compute the volumetric part of the tensor, that is 
			/// the trace V =Txx+Tyy+Tzz.
	Real GetVolumetricPart() const
					{
						return XX()+YY()+ZZ();
					}
			/// Compute the deviatoric part of the tensor, storing
			/// it in mdeviatoric
	void GetDeviatoricPart(ChVoightTensor<Real>& mdeviatoric) const
					{
						Real mM = GetVolumetricPart()/3.0;
						mdeviatoric = *this;
						mdeviatoric.XX() -= mM;
						mdeviatoric.YY() -= mM;
						mdeviatoric.ZZ() -= mM;
					}

};

/// Class for stress tensors, in compact Voight notation
/// that is with 6 components in a column. 

template <class Real = double>
class ChStressTensor : public ChVoightTensor<Real>
{
public:
			/// Compute the Von Mises equivalent stress
	double GetVonMisesStressV2() 
			{	
				return sqrt( 0.5*(pow(this->XX()-this->YY(),2.) + pow(this->YY()-this->ZZ(),2.) + pow(this->ZZ()-this->XX(),2.)) + 3.0*( this->XY()*this->XY() + this->XZ()*this->XZ() + this->YZ()*this->YZ()) );
			}
};


/// Class for strain tensors, in compact Voight notation
/// that is with 6 components in a column. 

template <class Real = double>
class ChStrainTensor : public ChVoightTensor<Real>
{
public:
};




/// Class for the basic properties of materials 
/// in an elastoplastic continuum. 
/// This is a base material with isothropic hookean 
/// elasticity.

class ChContinuumMaterial 
{
private:

	double E;			// Young Modulus
	double v;			// Poisson ratio
	double G;			// shear modulus
	double l;			// Lame's modulus

	double density;		// density

public:

			/// Create a continuum isothropic hookean material. 
			/// Default value for Young elastic modulus is low (like a 
			/// rubber-type material), and same for density.
	ChContinuumMaterial(double myoung = 10000000, double mpoisson=0.4, double mdensity=1000);

	virtual ~ChContinuumMaterial();
	

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
	
			/// Set the density of the material, in kg/m^2.
	void   Set_density (double m_density) {density = m_density;}
			/// Get the density of the material, in kg/m^2.
	double Get_density () {return density;}

			/// Get Lamé first parameter (the second is shear modulus, so Get_G() )
	double Get_l () {return l;}

			/// Get bulk modulus (increase of pressure for decrease of volume), in Pa. 
	double Get_BulkModulus () {return E/(3.*(1.-2.*v));}

			/// Get P-wave modulus (if V=speed of propagation of a P-wave, then (M/density)=V^2 ) 
	double Get_WaveModulus () {return E*((1.-v)/(1.+v)*(1.-2.*v));}


			/// Compute elastic stress from elastic strain
			/// (using column tensors, in Voight notation)
	void ComputeElasticStress(ChStressTensor<>& mstress, const ChStrainTensor<>& mstrain) const
				{/*
					mstress.XX() = mstrain.XX() * (l+2*G) + mstrain.XY() * l       + mstrain.XZ() * l;
					mstress.YY() = mstrain.XY() * l		  + mstrain.YY() * (l+2*G) + mstrain.YZ() * l;
					mstress.ZZ() = mstrain.XZ() * l       + mstrain.YZ() * l	   + mstrain.ZZ() * (l+2*G);
					mstress.XY() = mstrain.XY() * 2*G;
					mstress.XZ() = mstrain.XZ() * 2*G;
					mstress.YZ() = mstrain.YZ() * 2*G;
				*/
					mstress.XX() = mstrain.XX() * E;
					mstress.YY() = mstrain.YY() * E;
					mstress.ZZ() = mstrain.ZZ() * E;
				}

			/// Compute elastic strain from elastic stress
			/// (using column tensors, in Voight notation)
	void ComputeElasticStrain(ChStrainTensor<>& mstrain, const ChStressTensor<>& mstress) const
				{
					double invE = 1./E;
					mstrain.XX() = invE*( mstress.XX()		- mstress.XY() * v	- mstress.XZ() * v);
					mstrain.YY() = invE*(-mstress.XY() * v	+ mstress.YY()		- mstress.YZ() * v);
					mstrain.ZZ() = invE*(-mstress.XZ() * v	- mstress.YZ() * v	+ mstress.ZZ()    );
					mstrain.XY() = mstress.XY() * 0.5*G;
					mstrain.XZ() = mstress.XZ() * 0.5*G;
					mstrain.YZ() = mstress.YZ() * 0.5*G;
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



} // END_OF_NAMESPACE____
#endif 
