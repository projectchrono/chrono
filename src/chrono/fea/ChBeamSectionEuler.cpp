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
#include "chrono/core/ChMatrixMBD.h"
#include "chrono/core/ChMatrix33.h"


namespace chrono {
namespace fea {


	void ChBeamSectionEuler::ComputeInertiaDampingMatrix(ChMatrixNM<double, 6, 6>& Ri,  ///< 6x6 sectional inertial-damping (gyroscopic damping) matrix values here
    const ChVector<>& mW    ///< current angular velocity of section, in material frame
	) {
		double Delta = 1e-8; // magic number, todo: parametrize or #define
		Ri.setZero();

		if (compute_inertia_damping_matrix == false) 
			return;

		// Fi=Fia+Fiv, where Fia depends on acceleration only, so restrict to Fiv quadratic terms for numerical differentiation.
		// Also we assume first three columns of Ri are null because Fiv does not depend on linear velocity.
		// Quadratic terms (gyro, centrifugal) at current state:
		ChVectorN<double,6> Fi0;
		ChVector<> mF, mT;
		this->ComputeQuadraticTerms(mF, mT, mW);  
		Fi0.segment(0, 3) = mF.eigen();
		Fi0.segment(3, 3) = mT.eigen();
		// dw_x 
		ChVectorN<double,6> Fi_dw;
		this->ComputeQuadraticTerms(mF, mT, mW + ChVector<>(Delta,0,0)); 
		Fi_dw.segment(0, 3) = mF.eigen();
		Fi_dw.segment(3, 3) = mT.eigen();
		Ri.block(0,3, 6,1) = (Fi_dw - Fi0) * (1.0 / Delta);
		// dw_y 
		this->ComputeQuadraticTerms(mF, mT, mW + ChVector<>(0,Delta,0)); 
		Fi_dw.segment(0, 3) = mF.eigen();
		Fi_dw.segment(3, 3) = mT.eigen();
		Ri.block(0,4, 6,1) = (Fi_dw - Fi0) * (1.0 / Delta);
		// dw_z 
		this->ComputeQuadraticTerms(mF, mT, mW + ChVector<>(0,0,Delta)); 
		Fi_dw.segment(0, 3) = mF.eigen();
		Fi_dw.segment(3, 3) = mT.eigen();
		Ri.block(0,5, 6,1) = (Fi_dw - Fi0) * (1.0 / Delta);
	}


	void ChBeamSectionEuler::ComputeInertiaStiffnessMatrix(ChMatrixNM<double, 6, 6>& Ki, ///< 6x6 sectional inertial-stiffness matrix values here
		const ChVector<>& mWvel,      ///< current angular velocity of section, in material frame
		const ChVector<>& mWacc,      ///< current angular acceleration of section, in material frame
		const ChVector<>& mXacc       ///< current acceleration of section, in material frame (not absolute!)
	){
		double Delta = 1e-8; // magic number, todo: parametrize or #define
		Ki.setZero();

		if (compute_inertia_stiffness_matrix == false) 
			return;

		// We assume first three columns of Ki are null because Fi does not depend on displacement.
		// We compute Ki by numerical differentiation.

    
		ChVector<> mF, mT;
		this->ComputeInertialForce(mF, mT, mWvel, mWacc, mXacc);
		ChVectorN<double,6> Fi0;
		Fi0.segment(0, 3) = mF.eigen();
		Fi0.segment(3, 3) = mT.eigen();

   
		ChVectorN<double,6> Fi_dr;
		ChVectorN<double, 6> drFi;

		// dr_x
		ChStarMatrix33<> rot_lx(ChVector<>(Delta, 0, 0));
		rot_lx.diagonal().setOnes();
		this->ComputeInertialForce(mF, mT, 
			mWvel, // or rot_lx.transpose()*mWvel,  if abs. ang.vel is constant during rot.increments
			mWacc, // or rot_lx.transpose()*mWacc,  if abs. ang.vel is constant during rot.increments
			rot_lx.transpose()*mXacc);
		Fi_dr.segment(0, 3) = mF.eigen();
		Fi_dr.segment(3, 3) = mT.eigen();
		drFi.segment(0 , 3) = rot_lx * Fi0.segment(0, 3);
		drFi.segment(3 , 3) = Fi0.segment(3, 3);
		Ki.block(0, 3, 6, 1) = (Fi_dr - Fi0) * (1.0 / Delta) +(drFi - Fi0) * (1.0 / Delta);

		// dr_y
		ChStarMatrix33<> rot_ly(ChVector<>(0, Delta, 0));
		rot_ly.diagonal().setOnes();
		this->ComputeInertialForce(mF, mT, 
			mWvel, 
			mWacc, 
			rot_ly.transpose()*mXacc);
		Fi_dr.segment(0, 3) = mF.eigen();
		Fi_dr.segment(3, 3) = mT.eigen();
		drFi.segment(0 , 3) = rot_ly * Fi0.segment(0, 3);
		drFi.segment(3 , 3) = Fi0.segment(3, 3);
		Ki.block(0, 4, 6, 1) = (Fi_dr - Fi0) * (1.0 / Delta) +(drFi - Fi0) * (1.0 / Delta);

		// dr_z
		ChStarMatrix33<> rot_lz(ChVector<>(0, 0, Delta));
		rot_lz.diagonal().setOnes();
		this->ComputeInertialForce(mF, mT, 
			mWvel, 
			mWacc, 
			rot_lz.transpose()*mXacc);
		Fi_dr.segment(0, 3) = mF.eigen();
		Fi_dr.segment(3, 3) = mT.eigen();
		drFi.segment(0 , 3) = rot_lz * Fi0.segment(0, 3);
		drFi.segment(3 , 3) = Fi0.segment(3, 3);
		Ki.block(0, 5, 6, 1) = (Fi_dr - Fi0) * (1.0 / Delta) +(drFi - Fi0) * (1.0 / Delta);
	}

	void ChBeamSectionEuler::ComputeInertialForce(
                                    ChVector<>& mFi,          ///< total inertial force returned here
                                    ChVector<>& mTi,          ///< total inertial torque returned here
                                    const ChVector<>& mWvel,  ///< current angular velocity of section, in material frame
                                    const ChVector<>& mWacc,  ///< current angular acceleration of section, in material frame
                                    const ChVector<>& mXacc   ///< current acceleration of section, in material frame (not absolute!)
	) {
		// Default implementation as Fi = [Mi]*{xacc,wacc}+{mF_quadratic,mT_quadratic}
		// but if possible implement it in children classes with ad-hoc faster formulas.
		ChMatrixNM<double, 6, 6> Mi;
		this->ComputeInertiaMatrix(Mi);
		ChVectorN<double, 6> xpp; 
		xpp.segment(0 , 3) = mXacc.eigen();
		xpp.segment(3 , 3) = mWacc.eigen();
		ChVectorN<double, 6> Fipp = Mi * xpp;    // [Mi]*{xacc,wacc}
		ChVector<> mF_quadratic;
		ChVector<> mT_quadratic;
		this->ComputeQuadraticTerms(mF_quadratic, mT_quadratic, mWvel);  // {mF_quadratic,mT_quadratic}
		mFi = ChVector<>(Fipp.segment(0, 3)) + mF_quadratic; 
		mTi = ChVector<>(Fipp.segment(3, 3)) + mT_quadratic;
	}







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

	void ChBeamSectionEulerSimple::ComputeInertiaDampingMatrix(ChMatrixNM<double, 6, 6>& Ri,  ///< 6x6 sectional inertial-damping (gyroscopic damping) matrix values here
		const ChVector<>& mW    ///< current angular velocity of section, in material frame
	) {
		Ri.setZero();
		if (compute_inertia_damping_matrix == false) 
			return;
		if (this->compute_Ri_Ki_by_num_diff)
			return ChBeamSectionEuler::ComputeInertiaDampingMatrix(Ri, mW);

		ChStarMatrix33<> wtilde(mW);   // [w~]
		// [I]  here a diagonal inertia, in Euler it should be Jzz = 0 Jyy = 0, but a tiny nonzero value avoids singularity:
		ChMatrix33<> mI(ChVector<>(
			(this->Iyy + this->Izz) * this->density,
			JzzJyy_factor * this->Area * this->density,
			JzzJyy_factor * this->Area * this->density));  
		Ri.block<3, 3>(3, 3) = wtilde * mI - ChStarMatrix33<>(mI * mW);  // Ri = [0, 0; 0, [w~][I] - [([I]*w)~]  ]
	}

	void ChBeamSectionEulerSimple::ComputeInertiaStiffnessMatrix(ChMatrixNM<double, 6, 6>& Ki, ///< 6x6 sectional inertial-stiffness matrix values here
		const ChVector<>& mWvel,      ///< current angular velocity of section, in material frame
		const ChVector<>& mWacc,      ///< current angular acceleration of section, in material frame
		const ChVector<>& mXacc       ///< current acceleration of section, in material frame
	) {
		Ki.setZero();
		if (compute_inertia_stiffness_matrix == false) 
        return;
		if (this->compute_Ri_Ki_by_num_diff)
			return ChBeamSectionEuler::ComputeInertiaStiffnessMatrix(Ki, mWvel, mWacc, mXacc);
		// null [Ki^] (but only for the case where angular speeds and accelerations are assumed to corotate with local frames). 
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

	void ChBeamSectionEulerAdvancedGeneric::ComputeInertiaDampingMatrix(ChMatrixNM<double, 6, 6>& Ri,  ///< 6x6 sectional inertial-damping (gyroscopic damping) matrix values here
		const ChVector<>& mW    ///< current angular velocity of section, in material frame
	) {
		Ri.setZero();
		if (compute_inertia_damping_matrix == false) 
			return;
		if (this->compute_Ri_Ki_by_num_diff)
			return ChBeamSectionEuler::ComputeInertiaDampingMatrix(Ri, mW);

		ChStarMatrix33<> wtilde(mW);   // [w~]
		ChVector<> mC(0, this->My, this->Mz);
		ChStarMatrix33<> ctilde(mC);   // [c~]
		ChMatrix33<> mI; 
		mI << this->Jxx  ,														    0,			                          					 	  0,
						0,   JzzJyy_factor * this->mu + this->mu * this->Mz * this->Mz,                            - this->mu * this->My * this->Mz,
					    0,                            - this->mu * this->My * this->Mz,   JzzJyy_factor * this->mu + this->mu * this->My * this->My;
		//  Ri = [0,  m*[w~][c~]' + m*[([w~]*c)~]'  ; 0 , [w~][I] - [([I]*w)~]  ]
		Ri.block<3, 3>(0, 3) = this->mu * (wtilde * ctilde.transpose() + (ChStarMatrix33<>( wtilde * mC )).transpose() ); 
		Ri.block<3, 3>(3, 3) = wtilde * mI - ChStarMatrix33<>(mI * mW);  
	}

	void ChBeamSectionEulerAdvancedGeneric::ComputeInertiaStiffnessMatrix(ChMatrixNM<double, 6, 6>& Ki, ///< 6x6 sectional inertial-stiffness matrix values here
		const ChVector<>& mWvel,      ///< current angular velocity of section, in material frame
		const ChVector<>& mWacc,      ///< current angular acceleration of section, in material frame
		const ChVector<>& mXacc       ///< current acceleration of section, in material frame
	) {
		Ki.setZero();
		if (compute_inertia_stiffness_matrix == false) 
			return;
		if (this->compute_Ri_Ki_by_num_diff)
			return ChBeamSectionEuler::ComputeInertiaStiffnessMatrix(Ki, mWvel, mWacc, mXacc);

		ChStarMatrix33<> wtilde(mWvel);   // [w~]
		ChStarMatrix33<> atilde(mWacc);   // [a~]
		ChVector<> mC(0, this->My, this->Mz);
		ChStarMatrix33<> ctilde(mC);   // [c~]
		ChMatrix33<> mI;
		mI << this->Jxx  ,														    0,			                          					 	  0,
						0,   JzzJyy_factor * this->mu + this->mu * this->Mz * this->Mz,                            - this->mu * this->My * this->Mz,
					    0,                            - this->mu * this->My * this->Mz,   JzzJyy_factor * this->mu + this->mu * this->My * this->My;
		// Ki_al = [0, 0; -m*[([a~]c)~] -m*[([w~][w~]c)~] , m*[c~][xpp~] ]
		Ki.block<3, 3>(0, 3) = -this->mu * ChStarMatrix33<>(atilde * mC) 
							   -this->mu * ChStarMatrix33<>(wtilde*(wtilde*mC));
		Ki.block<3, 3>(3, 3) = this->mu * ctilde * ChStarMatrix33<>(mXacc);  
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

	void ChBeamSectionRayleighSimple::ComputeInertiaDampingMatrix(ChMatrixNM<double, 6, 6>& Ri,  ///< 6x6 sectional inertial-damping (gyroscopic damping) matrix values here
		const ChVector<>& mW    ///< current angular velocity of section, in material frame
	) {
		Ri.setZero();
		if (compute_inertia_damping_matrix == false) 
			return;
		if (this->compute_Ri_Ki_by_num_diff)
			return ChBeamSectionEuler::ComputeInertiaDampingMatrix(Ri, mW);

		ChStarMatrix33<> wtilde(mW);   // [w~]
		// [I]  here a diagonal inertia, in Euler it should be Jzz = 0 Jyy = 0, but a tiny nonzero value avoids singularity:
		ChMatrix33<> mI(ChVector<>(
			(this->Iyy + this->Izz) * this->density,
			JzzJyy_factor * this->Area * this->density + this->Iyy * this->density,
			JzzJyy_factor * this->Area * this->density + this->Izz * this->density));  
		Ri.block<3, 3>(3, 3) = wtilde * mI - ChStarMatrix33<>(mI * mW);  // Ri = [0, 0; 0, [w~][I] - [([I]*w)~]  ]
	}

	void ChBeamSectionRayleighSimple::ComputeInertiaStiffnessMatrix(ChMatrixNM<double, 6, 6>& Ki, ///< 6x6 sectional inertial-stiffness matrix values here
		const ChVector<>& mWvel,      ///< current angular velocity of section, in material frame
		const ChVector<>& mWacc,      ///< current angular acceleration of section, in material frame
		const ChVector<>& mXacc       ///< current acceleration of section, in material frame
	) {
		Ki.setZero();
		if (compute_inertia_stiffness_matrix == false) 
			return;
		if (this->compute_Ri_Ki_by_num_diff)
			return ChBeamSectionEuler::ComputeInertiaStiffnessMatrix(Ki, mWvel, mWacc, mXacc);
		// null [Ki^] (but only for the case where angular speeds and accelerations are assumed to corotate with local frames. 
	}

	void ChBeamSectionRayleighSimple::ComputeQuadraticTerms(ChVector<>& mF,   ///< centrifugal term (if any) returned here
		ChVector<>& mT,                ///< gyroscopic term  returned here
		const ChVector<>& mW           ///< current angular velocity of section, in material frame
	) {
		// F_centrifugal = density_per_unit_length w X w X c 
		mF = VNULL;

		// unroll the product [J] * w  in the expression w X [J] * w  as 8 values of [J] are zero anyway
		mT = Vcross(mW, ChVector<>( this->GetInertiaJxxPerUnitLength()*mW.x(),
									(JzzJyy_factor * this->Area * this->density + this->Iyy * this->density) * mW.y(),
									(JzzJyy_factor * this->Area * this->density + this->Izz * this->density) * mW.z())  );
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

	void ChBeamSectionRayleighAdvancedGeneric::ComputeInertiaDampingMatrix(ChMatrixNM<double, 6, 6>& Ri,  ///< 6x6 sectional inertial-damping (gyroscopic damping) matrix values here
		const ChVector<>& mW    ///< current angular velocity of section, in material frame
	) {
		Ri.setZero();
		if (compute_inertia_damping_matrix == false) 
			return;
		if (this->compute_Ri_Ki_by_num_diff)
			return ChBeamSectionEuler::ComputeInertiaDampingMatrix(Ri, mW);

		ChStarMatrix33<> wtilde(mW);   // [w~]
		ChVector<> mC(0, this->My, this->Mz);
		ChStarMatrix33<> ctilde(mC);   // [c~]
		ChMatrix33<> mI; 
		mI << this->Jyy+this->Jzz ,										 0,			         			   	    0,
								 0,   this->Jyy + JzzJyy_factor * this->mu,                            -this->Jyz,
								 0,                             -this->Jyz,  this->Jzz + JzzJyy_factor * this->mu;
		//  Ri = [0,  m*[w~][c~]' + m*[([w~]*c)~]'  ; 0 , [w~][I] - [([I]*w)~]  ]
		Ri.block<3, 3>(0, 3) = this->mu * (wtilde * ctilde.transpose() + (ChStarMatrix33<>( wtilde * mC )).transpose() ); 
		Ri.block<3, 3>(3, 3) = wtilde * mI - ChStarMatrix33<>(mI * mW);  
	}

	void ChBeamSectionRayleighAdvancedGeneric::ComputeInertiaStiffnessMatrix(ChMatrixNM<double, 6, 6>& Ki, ///< 6x6 sectional inertial-stiffness matrix values here
		const ChVector<>& mWvel,      ///< current angular velocity of section, in material frame
		const ChVector<>& mWacc,      ///< current angular acceleration of section, in material frame
		const ChVector<>& mXacc       ///< current acceleration of section, in material frame
	) {
		Ki.setZero();
		if (compute_inertia_stiffness_matrix == false) 
			return;
		if (this->compute_Ri_Ki_by_num_diff)
			return ChBeamSectionEuler::ComputeInertiaStiffnessMatrix(Ki, mWvel, mWacc, mXacc);

		ChStarMatrix33<> wtilde(mWvel);   // [w~]
		ChStarMatrix33<> atilde(mWacc);   // [a~]
		ChVector<> mC(0, this->My, this->Mz);
		ChStarMatrix33<> ctilde(mC);   // [c~]
		ChMatrix33<> mI;
		mI << this->Jyy+this->Jzz ,										 0,			         			   	    0,
								 0,   this->Jyy + JzzJyy_factor * this->mu,                            -this->Jyz,
								 0,                             -this->Jyz,  this->Jzz + JzzJyy_factor * this->mu;
		// Ki_al = [0, 0; -m*[([a~]c)~] -m*[([w~][w~]c)~] , m*[c~][xpp~] ]
		Ki.block<3, 3>(0, 3) = -this->mu * ChStarMatrix33<>(atilde * mC) 
							   -this->mu * ChStarMatrix33<>(wtilde*(wtilde*mC));
		Ki.block<3, 3>(3, 3) = this->mu * ctilde * ChStarMatrix33<>(mXacc); 
	}

	void ChBeamSectionRayleighAdvancedGeneric::ComputeQuadraticTerms(ChVector<>& mF,   ///< centrifugal term (if any) returned here
		ChVector<>& mT,                ///< gyroscopic term  returned here
		const ChVector<>& mW           ///< current angular velocity of section, in material frame
	) {
		 // F_centrifugal = density_per_unit_length w X w X c 
		mF = this->mu * Vcross(mW,Vcross(mW,ChVector<>(0,this->My,this->Mz)));

		// unroll the product [J] * w  in the expression w X [J] * w  as 4 values of [J] are zero anyway
		mT = Vcross(mW, ChVector<>( this->GetInertiaJxxPerUnitLength()*mW.x(),
									(this->Jyy + JzzJyy_factor * this->mu)*mW.y() - this->Jyz*mW.z(),
									(this->Jzz + JzzJyy_factor * this->mu)*mW.z() - this->Jyz*mW.y() )  );
	}



}  // end namespace fea
}  // end namespace chrono

