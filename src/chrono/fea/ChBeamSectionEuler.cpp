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


#include "chrono/fea/ChBeamSectionEuler.h"


namespace chrono {
namespace fea {


	void ChBeamSectionEulerSimple::ComputeInertiaMatrix(ChMatrixNM<double, 6, 6>& M) {
		M.setZero();
		M(0, 0) = this->Area * this->density;
		M(1, 1) = this->Area * this->density;
		M(2, 2) = this->Area * this->density;
		M(3, 3) = (this->Iyy + this->Izz) * this->density;
		// M(4, 4) M(5, 5)the following are zero in Euler theory  
		//..or just make a tiny nonzero value to avoid singularity
		//M(4, 4) = 1./500. * M(0, 0);
		//M(5, 5) = 1./500. * M(0, 0);
		// .. but Rayleigh beam theory may add it, etc
		//M(4, 4) = this->Jyy;
		//M(5, 5) = this->Jzz;
		//M(4, 5) = -this->Jyz;
		//M(5, 4) = -this->Jyz;
	}

	void ChBeamSectionEulerSimple::ComputeQuadraticTerms(ChVector<>& mF, ChVector<>& mT, const ChVector<>& mW) {
		mF = VNULL;
		mT = VNULL;
	}


	void  ChBeamSectionEulerAdvancedGeneric::ComputeInertiaMatrix(ChMatrixNM<double, 6, 6>& M) {
		M.setZero();
		M(0, 0) = this->mu;
		M(1, 1) = this->mu;
		M(2, 2) = this->mu;

		M(3, 1) = - this->mu * this->Mz;
		M(3, 2) =   this->mu * this->My;
		M(4, 0) =   this->mu * this->Mz;
		M(5, 0) = - this->mu * this->My;

		M(1, 3) = - this->mu * this->Mz;
		M(2, 3) =   this->mu * this->My;
		M(0, 4) =   this->mu * this->Mz;
		M(0, 5) = - this->mu * this->My;

		M(3, 3) = this->Jxx;
		// M(4, 4) M(5, 5) are zero in Euler theory, as Jzz = 0 Jyy = 0 
		// .. but just make them a tiny nonzero value to avoid singularity, if small JzzJyy_factor
		M(4, 4) = JzzJyy_factor * M(0, 0);
		M(5, 5) = JzzJyy_factor * M(0, 0);
		// .. but Rayleigh beam theory may add it as:
		//M(4, 4) = this->Jyy;
		//M(5, 5) = this->Jzz;
		//M(4, 5) = -this->Jyz;
		//M(5, 4) = -this->Jyz;
	}

	void ChBeamSectionEulerAdvancedGeneric::ComputeQuadraticTerms(ChVector<>& mF,   ///< centrifugal term (if any) returned here
		ChVector<>& mT,                ///< gyroscopic term  returned here
		const ChVector<>& mW           ///< current angular velocity of section, in material frame
	) {
		// F_centrifugal = density_per_unit_length w X w X c 
		mF = this->mu * Vcross(mW,Vcross(mW,ChVector<>(0,My,Mz)));

		// unroll the product [J] * w  in the expression w X [J] * w  as 8 values of [J] are zero anyway
		mT = Vcross(mW, ChVector<>( this->GetInertiaJxxPerUnitLength()*mW.x(),
									0,
									0 )  );
	}


	ChBeamSectionEulerEasyRectangular::ChBeamSectionEulerEasyRectangular(double width_y, double width_z, double myE,  double myG, double mydensity)
	{
		this->SetYoungModulus(myE);
		this->SetGshearModulus(myG);
		this->SetDensity(mydensity);
		this->SetAsRectangularSection(width_y, width_z);
	}

	ChBeamSectionEulerEasyCircular::ChBeamSectionEulerEasyCircular(double diameter, double myE,  double myG, double mydensity)
	{
		this->SetYoungModulus(myE);
		this->SetGshearModulus(myG);
		this->SetDensity(mydensity);
		this->SetAsCircularSection(diameter);
	}



}  // end namespace fea
}  // end namespace chrono

