//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHTENSORS_H
#define CHTENSORS_H

//////////////////////////////////////////////////
//  
//   ChTensors.h
//
//   Class for stress & strain tensors
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include <stdlib.h>
#include "core/ChApiCE.h"
#include "core/ChMath.h"
#include "core/ChShared.h"

namespace chrono 
{
namespace fem
{


/// Base class for stress and strain tensors, in compact Voight notation
/// that is with 6 components in a column. This saves some
/// memory when compared to traditional 3D tensors with three
/// rows and three columns, that are symmetric.

template <class Real = double>
class ChApi ChVoightTensor : public ChMatrixNM<Real,6,1>
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

			/// Compute the I1 invariant
	Real GetInvariant_I1() const
			{
				return XX()+YY()+ZZ();
			}

			/// Compute the I2 invariant
	Real GetInvariant_I2() const
			{
				return XX()*YY() + YY()*ZZ() + XX()*ZZ() 
					 - XY()*XY() - YZ()*YZ() - XZ()*XZ();
			}

			/// Compute the I3 invariant
	Real GetInvariant_I3() const
			{
				return XX()*YY()*ZZ() + 2*XY()*YZ()*XZ() 
					  -XY()*XY()*ZZ() - YZ()*YZ()*XX() - XZ()*XZ()*YY();
			}

			/// Compute the J1 invariant of the deviatoric part (that is always 0)
	Real GetInvariant_J1() const { return 0;}
			 
			/// Compute the J2 invariant of the deviatoric part
	Real GetInvariant_J2() const
	{
		return ChMax(0.0,  ((pow(this->GetInvariant_I1(),2))/3.0) -this->GetInvariant_I2() );
	}
			/// Compute the J3 invariant of the deviatoric part
	Real GetInvariant_J3() const
	{
		return ( pow(this->GetInvariant_I1(),3)*(2./27.)
			        -this->GetInvariant_I1()*this->GetInvariant_I2()*(1./3.)
					+this->GetInvariant_I3() );
	}

		  /// FORMULAS THAT ARE USEFUL FOR YELD CRITERIONS: 

			/// Compute the Von Mises equivalent
	double GetEquivalentVonMises() const
			{	
				return sqrt( 0.5*(pow(this->XX()-this->YY(),2.) + pow(this->YY()-this->ZZ(),2.) + pow(this->ZZ()-this->XX(),2.)) + 3.0*( this->XY()*this->XY() + this->XZ()*this->XZ() + this->YZ()*this->YZ()) );
			}

};

/// Class for stress tensors, in compact Voight notation
/// that is with 6 components in a column. 

template <class Real = double>
class ChApi ChStressTensor : public ChVoightTensor<Real>
{
public:

};


/// Class for strain tensors, in compact Voight notation
/// that is with 6 components in a column. 

template <class Real = double>
class ChApi ChStrainTensor : public ChVoightTensor<Real>
{
public:

};






} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____
#endif 
