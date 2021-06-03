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
		// M(4, 4) M(5, 5) are zero in Euler theory, as Jzz = 0 Jyy = 0 
		// .. but just make them a tiny nonzero value to avoid singularity, if small JzzJyy_factor
		M(4, 4) = JzzJyy_factor * M(0, 0);
		M(5, 5) = JzzJyy_factor * M(0, 0);
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
		// M(4, 4) M(5, 5) are zero in Euler theory, as Jzz = 0 Jyy = 0  (for no My Mz offsets)
		// .. but just make them a tiny nonzero value to avoid singularity, if small JzzJyy_factor
		M(4, 4) = JzzJyy_factor * M(0, 0) + this->mu * this->Mz * this->Mz;
		M(5, 5) = JzzJyy_factor * M(0, 0) + this->mu * this->My * this->My;
		M(4, 5) = - this->mu * this->My * this->Mz;
		M(5, 4) = - this->mu * this->My * this->Mz;
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




	void ChBeamSectionRayleighSimple::ComputeInertiaMatrix(ChMatrixNM<double, 6, 6>& M)
	{
		// inherit 
		ChBeamSectionEulerSimple::ComputeInertiaMatrix(M);

		// add Rayleigh terms
		M(4, 4) += this->Iyy * this->density;
		M(5, 5) += this->Izz * this->density;
	}



	ChBeamSectionRayleighEasyRectangular::ChBeamSectionRayleighEasyRectangular(double mwidth_y, double mwidth_z, double mE, double mG, double mdensity)
	{
		this->SetYoungModulus(mE);
		this->SetGshearModulus(mG);
		this->SetDensity(mdensity);
		this->SetAsRectangularSection(mwidth_y,mwidth_z);
	}

	ChBeamSectionRayleighEasyCircular::ChBeamSectionRayleighEasyCircular(double diameter, double mE, double mG, double mdensity)
	{
		this->SetYoungModulus(mE);
		this->SetGshearModulus(mG);
		this->SetDensity(mdensity);
		this->SetAsCircularSection(diameter);
	}

	
	





	void ChBeamSectionRayleighAdvancedGeneric::SetInertiasPerUnitLength(const double mJyy, const double mJzz, const double mJyz) {
			this->Jyy = mJyy;
			this->Jzz = mJzz; 
			this->Jyz = mJyz;
			// automatically set parent Jxx value
			this->Jxx = (this->Jyy + this->Jzz);
	}


	void ChBeamSectionRayleighAdvancedGeneric::SetMainInertiasInMassReference(double Jmyy, double Jmzz, double phi) {
		double cc = pow(cos(-phi), 2);
		double ss = pow(sin(-phi), 2);
		double cs = cos(-phi) * sin(-phi);
		// generic 2x2 tensor rotation
		double Tyy_rot = cc * Jmyy + ss * Jmzz; // + 2 * Jmyz * cs; //TODO: it seems the commented term has an opposite sign
		double Tzz_rot = ss * Jmyy + cc * Jmzz; // - 2 * Jmyz * cs;  //TODO: it seems the commented term has an opposite sign
		double Tyz_rot = (Jmzz - Jmyy) * cs; // +Jmyz * (cc - ss);   //TODO: it seems the commented term has an opposite sign
		// add inertia transport 
		this->Jyy = Tyy_rot +  this->mu * this->Mz * this->Mz;
		this->Jzz = Tzz_rot +  this->mu * this->My * this->My;
		this->Jyz = -(Tyz_rot -  this->mu * this->Mz * this->My); // note minus, per definition of Jyz
		// automatically set parent Jxx value
		this->Jxx = (this->Jyy + this->Jzz);
	}

	void ChBeamSectionRayleighAdvancedGeneric::GetMainInertiasInMassReference(double& Jmyy, double& Jmzz, double& phi) {
		// remove inertia transport
		double Tyy_rot  =  this->Jyy - this->mu * this->Mz * this->Mz;
		double Tzz_rot  =  this->Jzz - this->mu * this->My * this->My;
		double Tyz_rot  = -this->Jyz + this->mu * this->Mz * this->My;
		// tensor de-rotation up to principal axes
		double argum = pow((Tyy_rot - Tzz_rot) * 0.5, 2) + pow(Tyz_rot, 2);
		if (argum <= 0) {
			phi = 0;
			Jmyy = 0.5 * (Tzz_rot + Tyy_rot);
			Jmzz = 0.5 * (Tzz_rot + Tyy_rot);
			return;
		}
		double discr = sqrt( pow((Tyy_rot - Tzz_rot)*0.5,2) + pow(Tyz_rot, 2) );
		phi = - 0.5* atan2(Tyz_rot / discr, (Tzz_rot - Tyy_rot) / (2. * discr));
		Jmyy = 0.5 * (Tzz_rot + Tyy_rot) - discr;
		Jmzz = 0.5 * (Tzz_rot + Tyy_rot) + discr;
	}
	
	
	void ChBeamSectionRayleighAdvancedGeneric::ComputeInertiaMatrix(ChMatrixNM<double, 6, 6>& M)
	{
		// inherit 
		ChBeamSectionEulerAdvancedGeneric::ComputeInertiaMatrix(M);

		// overwrite rotational part using Rayleigh terms, similarly to the Cosserat beam.
		//Also add the JzzJyy_factor*M(0,0) term as in Euler beams - a term that avoids zero on diagonal and that can be turned off. 
		M(3, 3) = this->Jyy+this->Jzz;
		M(4, 4) = this->Jyy     + JzzJyy_factor * M(0, 0);
		M(5, 5) = this->Jzz     + JzzJyy_factor * M(0, 0);
		M(4, 5) = -this->Jyz;
		M(5, 4) = -this->Jyz;
	}


	void ChBeamSectionRayleighAdvancedGeneric::ComputeQuadraticTerms(
        ChVector<>& mF,       ///< centrifugal term (if any) returned here
        ChVector<>& mT,       ///< gyroscopic term  returned here
        const ChVector<>& mW  ///< current angular velocity of section, in material frame
    ) {
        // F_centrifugal = density_per_unit_length w X w X c
        mF = this->mu * Vcross(mW, Vcross(mW, ChVector<>(0, My, Mz)));

        // unroll the product [J] * w  in the expression w X [J] * w  as 4 values of [J] are zero anyway
        mT = Vcross(mW, ChVector<>(this->GetInertiaJxxPerUnitLength() * mW.x(), 
								   this->Jyy * mW.y() - this->Jyz * mW.z(),
                                   this->Jzz * mW.z() - this->Jyz * mW.y()));
    }



}  // end namespace fea
}  // end namespace chrono

