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
// Authors: Erol Lale,  Wisdom Akpan, Ke Yu, Jibril B. Coulibaly
// =============================================================================
// Material class for LDPM and CSL elements 
//
// A description of the material parameter can be found in: https://doi.org/10.1016/j.cemconcomp.2011.02.011
// =============================================================================

#include "chrono_wood/ChWoodMaterialVECT.h"
#include "chrono/core/ChMatrix.h"
#include "chrono/core/ChVector3.h"

namespace chrono {
namespace wood {

// Construct an isotropic material.

ChWoodMaterialVECT::ChWoodMaterialVECT(double rho,  // material density
                                       double E0,    // Mesoscale Young's modulus
                                       double alpha,   // Mesoscale Poisson ratio                                       
                                       double sigmat, double sigmac, double sigmas,
                                       double nt, double lt, double rs)
    : m_rho(rho), m_E0(E0), m_alpha(alpha), m_sigmat(sigmat), m_sigmas(sigmas), m_sigmac(sigmac), m_nt(nt), m_lt(lt), m_rs(rs) {}

ChWoodMaterialVECT::ChWoodMaterialVECT() {}

ChWoodMaterialVECT::~ChWoodMaterialVECT() {}

// statec(0): normal N strain
// statec(1): shear M strain
// statec(2): shear L strain
// statec(3): normal N stress
// statec(4): shear M stress
// statec(5): shear L stress
// statec(6): maximum normal N strain
// statec(7): maximum shear T strain
// statec(8): effective strain
// statec(9): effective stress
// statec(10): internal work
// statec(11): crack opening

void ChWoodMaterialVECT::ComputeStress(ChVector3d& dmstrain, ChVector3d& dmcurvature,double &len, StateVarVector& statev,  double& area, double& width, double& height, double& random_field, ChVector3d& mstress, ChVector3d& mcouple) {  
    	ChVector3d mstrain;
    	ChVector3d mcurvature;	
	//
	double E0=this->Get_E0();
	double alpha=this->Get_alpha();	
	double sigmat = this->Get_sigmat();	
	double sigmac = this->Get_sigmac();
	if (random_field)
		E0=E0*random_field;
	//std::cout<<"E0: "<<E0<<" alpha: "<<alpha<<std::endl;	
	// 	
	mstrain=statev.segment(0,3)+dmstrain.eigen();	
	//	
	//
	double epsQ, epsQN;
	double epsT;
	double h2=height*height;		
	double w2=width*width;
	
	if (!this->GetCoupleMultiplier()){		
		epsQ = std::sqrt(mstrain[0] * mstrain[0] + alpha * (mstrain[1] * mstrain[1] + mstrain[2] * mstrain[2]));
		epsT = std::sqrt(mstrain[1] * mstrain[1] + mstrain[2] * mstrain[2]);
		epsQN = mstrain[0];
	}else{	
		double multiplier=this->GetCoupleMultiplier()/3.;					
		mcurvature=statev.segment(12,3)+dmcurvature.eigen();
		//std::cout<<"mstrain: "<<mstrain<<" mcurvature: "<<mcurvature<<std::endl;
		//std::cout<<"curvature: "<<mcurvature<<std::endl;		
				
		epsQN = std::sqrt(mstrain[0] * mstrain[0] + multiplier*( mcurvature[1]*mcurvature[1]*w2 + mcurvature[2]*mcurvature[2]*h2));
		epsT = std::sqrt(mstrain[1] * mstrain[1] + mstrain[2] * mstrain[2]+multiplier*mcurvature[0]*mcurvature[0]*(w2+h2));
		
		epsQ = std::sqrt(epsQN * epsQN + alpha * epsT * epsT);
	
	}
	
	if (statev(6) < epsQN) {
		statev(6) = epsQN;
	}
	if (statev(7) < epsT) {
		statev(7) = epsT;
	}
	
	
	if (!GetElasticAnalysisFlag()) {
	//
	// INELASTIC ANALYSIS
	//
	if (epsQ != 0) {
		if (mstrain[0] > 10e-16 || sigmat<sigmac) {     // fracture behaivor
			double strsQ = FractureBC(mstrain, random_field, len, epsQ, epsQN, epsT, statev);
			mstress[0] = strsQ * mstrain[0] / epsQ;
			mstress[1] = alpha * strsQ * mstrain[1] / epsQ;
			mstress[2] = alpha * strsQ * mstrain[2] / epsQ;
			//
			if(this->GetCoupleMultiplier()){
				double multiplier=this->GetCoupleMultiplier()/3.;
				mcouple[0]=multiplier*alpha*strsQ*mcurvature[0]*(w2+h2)/epsQ;
				mcouple[1]=multiplier*strsQ*mcurvature[1]*w2/epsQ;
				mcouple[2]=multiplier*strsQ*mcurvature[2]*h2/epsQ;						
			}
		}
		else {
			
			 double strsQ = CompressBC(mstrain, random_field, len, epsQ, epsT, epsQN, statev);
			 
			 mstress[0] = strsQ * mstrain[0] / epsQ;
			 mstress[1] = alpha * strsQ * mstrain[1] / epsQ;
			 mstress[2] = alpha * strsQ * mstrain[2] / epsQ;
			 //
			 if(this->GetCoupleMultiplier()){
				 double multiplier=this->GetCoupleMultiplier()/3.;
				 mcouple[0]=multiplier*alpha*strsQ*mcurvature[0]*(w2+h2)/epsQ;
				 mcouple[1]=multiplier*strsQ*mcurvature[1]*w2/epsQ;
				 mcouple[2]=multiplier*strsQ*mcurvature[2]*h2/epsQ;						
			 }
		}
		double Wint = len * area * (((mstress[0] + statev(3)) / 2.0) * dmstrain[0] + ((mstress[1] + statev(4)) / 2.0) * dmstrain[1] + ((mstress[2] + statev(5)) / 2.0) * dmstrain[2] + ((mcouple[0] + statev(15)) / 2.0) * dmcurvature[0] + ((mcouple[1] + statev(16)) / 2.0) * dmcurvature[1] + ((mcouple[2] + statev(17)) / 2.0) * dmcurvature[2]);
		double w_N = len * (mstrain[0] - mstress[0] / E0);
		double w_M = len * (mstrain[1] - mstress[1] / (E0*alpha));
		double w_L = len * (mstrain[2] - mstress[2] / (E0 * alpha));

		double w = std::sqrt(w_N * w_N + w_M * w_M + w_L * w_L);

		statev(0) = mstrain[0];
		statev(1) = mstrain[1];
		statev(2) = mstrain[2];
		statev(3) = mstress[0];
		statev(4) = mstress[1];
		statev(5) = mstress[2];
		statev(8) = std::sqrt(statev(0) * statev(0) + alpha * (statev(1) * statev(1) + statev(2) * statev(2))); // TODO JBC: Why doesn't statev(8) take bending into account?
		statev(9) = std::sqrt(statev(3) * statev(3) + (statev(4) * statev(4) + statev(5) * statev(5)) / alpha); // TODO JBC: Why doesn't statev(9) take bending into account?
		statev(10) = Wint + statev(10);
		statev(11) = w;
		//
		if(this->GetCoupleMultiplier()){				
				statev(12) = mcurvature[0];
				statev(13) = mcurvature[1];
				statev(14) = mcurvature[2];	
				//
				statev(15) = mcouple[0];
				statev(16) = mcouple[1];
				statev(17) = mcouple[2];
		}
	}
	else {
		mstress.Set(0.0);
		mcouple.Set(0.0);
	}
	//std::cout << "stress: " << mstress << std::endl;
	//std::cout << statev(3) << ' ' << statev(4) << ' ' << statev(5) << ' ' << statev(0) << ' ' << statev(1) << ' ' << statev(2) << std::endl;
	
	}else{
	
	//
	// ELASTIC ANALYSIS
	//
	if (epsQ!=0) {
	    double strsQ=E0*epsQ;
		mstress[0]=strsQ*mstrain[0]/epsQ;
		mstress[1]=alpha*strsQ*mstrain[1]/epsQ;
		mstress[2]=alpha*strsQ*mstrain[2]/epsQ;		
		
		
		double Wint = len * area * (((mstress[0] + statev(3)) / 2.0) * dmstrain[0] + ((mstress[1] + statev(4)) / 2.0) * dmstrain[1] + ((mstress[2] + statev(5)) / 2.0) * dmstrain[2] + ((mcouple[0] + statev(15)) / 2.0) * dmcurvature[0] + ((mcouple[1] + statev(16)) / 2.0) * dmcurvature[1] + ((mcouple[2] + statev(17)) / 2.0) * dmcurvature[2]);
		double w_N = len * (mstrain[0] - mstress[0] / E0);
		double w_M = len * (mstrain[1] - mstress[1] / (E0*alpha));
		double w_L = len * (mstrain[2] - mstress[2] / (E0 * alpha));

		double w = std::sqrt(w_N * w_N + w_M * w_M + w_L * w_L);
		
		statev(0) = mstrain[0];
		statev(1) = mstrain[1];
		statev(2) = mstrain[2];
		statev(3) = mstress[0];
		statev(4) = mstress[1];
		statev(5) = mstress[2];
		statev(8) = std::sqrt(statev(0) * statev(0) + alpha * (statev(1) * statev(1) + statev(2) * statev(2))); 
		statev(9) = std::sqrt(statev(3) * statev(3) + (statev(4) * statev(4) + statev(5) * statev(5)) / alpha);
		statev(10) = Wint + statev(10);
		statev(11) = w;
		
		
		if(this->GetCoupleMultiplier()){
			double multiplier=this->GetCoupleMultiplier()/3.;
			mcouple[0]=multiplier*alpha*strsQ*mcurvature[0]*(w2+h2)/epsQ;
			mcouple[1]=multiplier*strsQ*mcurvature[1]*w2/epsQ;
			mcouple[2]=multiplier*strsQ*mcurvature[2]*h2/epsQ;
			//
			statev(12) = mcurvature[0];
			statev(13) = mcurvature[1];
			statev(14) = mcurvature[2];
			//std::cout<<"mcurvature: "<<mcurvature.x()<<"\t"<<mcurvature.y()<<"\t"<<mcurvature.z()<<std::endl;
			//std::cout<<"mcouple: "<<mcouple.x()<<"\t"<<mcouple.y()<<"\t"<<mcouple.z()<<std::endl;
			//
			statev(15) = mcouple[0];
			statev(16) = mcouple[1];
			statev(17) = mcouple[2];
		}
		
	}else{
		mstress.Set(0.0);
		mcouple.Set(0.0);
	}
	
	}
	
	//std::cout<<"mstress: "<<mstress<<"  mcouple: "<<mcouple<<" w2, h2 "<<w2<<" "<<h2<<std::endl;
	//if (mstrain[0]<0) 
	//	exit(1);
	//printf("Statev: %f, %f, %f, %f, %f, %f\n",statev(0), statev(1), statev(2), statev(3), statev(4), statev(5));
}

/*
void ChWoodMaterialVECT::ComputeStress_NEW(ChVector3d& strain_incr, ChVector3d& curvature_incr, double &length, StateVarVector& statev, double& width, double& height, double& random_field, ChVector3d& stress, ChVector3d& surfacic_couple) {  
    ChVector3d strain(statev[0] + strain_incr[0], statev[1] + strain_incr[1], statev[2] + strain_incr[2]);
    ChVector3d curvature;

    // TODO JBC: passed argument width and height are actually the HALF WIDTH and HALF HEIGHT...	
    double area = width * height * 4.0;
    double E_N = m_E0;
    double E_T = m_E0 * m_alpha;
    double h2 = height * height;
    double w2 = width * width;
    double epsQ;
    double epsT;
    double sigQ = 0.0;
    double secant_N = 0.0;
    double secant_T = 0.0;
    double Wint = 0.0;
    double w = 0.0;
    constexpr double strain_tol = 1e-10;
    bool energy_and_crack_calculations_need_stress = false;
    
    if (mcouple_mult) {		
        epsQ = std::sqrt(strain[0] * strain[0] + m_alpha * (strain[1] * strain[1] + strain[2] * strain[2]));
        epsT = std::sqrt(strain[1] * strain[1] + strain[2] * strain[2]);
    } else {
        double multiplier=this->GetCoupleMultiplier()/3.;
        curvature.Set(statev[12] + curvature_incr[0], statev[13] + curvature_incr[1], statev[14] + curvature_incr[2]);

        epsQ = sqrt(strain[0] * strain[0] + m_alpha * (strain[1] * strain[1] + strain[2] * strain[2])+
                    multiplier*(m_alpha*curvature[0]*curvature[0]*(w2+h2)+ curvature[1]*curvature[1]*w2+
                    curvature[2]*curvature[2]*h2));
        epsT = sqrt(strain[1] * strain[1] + strain[2] * strain[2]+multiplier*curvature[0]*curvature[0]*(w2+h2));
    }

    if (m_ElasticAnalysis) {
        secant_N = E_N;
        secant_T = E_T;
        sigQ = m_E0 * epsQ;
        Wint = length * area * 0.5 * sigQ * epsQ;
    } else if (epsQ > strain_tol) {
        // JBC: move it in here because We still want to compute and zero the state variable if that happens
        // If for any reason we go from loading to epsQ <= 1e-16, the state variables will otherwise remain where they were, instead of going to zero.
        energy_and_crack_calculations_need_stress = true;
        if (strain[0] > strain_tol) {     // TODO JBC: WHY NOT ZERO ?!?!?! Is there a case where very small positive values should not lead to fracture? 
            sigQ = FractureBC(strain, random_field, length, epsQ, epsT, statev);
        } else {  
            sigQ = CompressBC(strain, random_field, length, epsQ, epsT, statev);
        }
        secant_N = sigQ / epsQ;
        secant_T = m_alpha * secant_N;
    }

    // Compute stress
    stress[0] = secant_N * strain[0];
    stress[1] = secant_T * strain[1];
    stress[2] = secant_T * strain[2];
    // Compute couples
    if(mcouple_mult) { // TODO JBC: this is not a great way of checking if we are computing moments, we should use a bool, there is also a flag in the CBLCON element
        surfacic_couple[0] = mcouple_mult * secant_T * curvature[0] * (w2 + h2) / 3.0;
        surfacic_couple[1] = mcouple_mult * secant_N * curvature[1] * w2 / 3.0;
        surfacic_couple[2] = mcouple_mult * secant_N * curvature[2] * h2 / 3.0;
    }

    // Update state variables
    if (energy_and_crack_calculations_need_stress) {
        Wint = statev(10) + length * area * 0.5 * ((stress[0] + statev(3)) * strain_incr[0] +
                                                   (stress[1] + statev(4)) * strain_incr[1] +
                                                   (stress[2] + statev(5)) * strain_incr[2]); // TODO JBC: What about curvature here ?
        double inelastic_strain_N = strain[0] - stress[0] / E_N;
        double inelastic_strain_M = strain[1] - stress[1] / E_T;
        double inelastic_strain_L = strain[2] - stress[2] / E_T;
        w = length * std::sqrt(inelastic_strain_N * inelastic_strain_N +
                               inelastic_strain_M * inelastic_strain_M +
                               inelastic_strain_L * inelastic_strain_L);
    }
    // TODO JBC: I think updating the state variables in place is wrong by design, and a bug!
    // Overwriting sv here means these change every iteration at a given step, while we would want to use the last converged value instead.
    // This is a much bigger discussiion related to how Chrono core updates elements as well (it updates multiple times in some time steppers, which I think is wrong, so even if we fixed our things this would still not work as it should)
    statev(0) = strain[0];
    statev(1) = strain[1];
    statev(2) = strain[2];
    statev(3) = stress[0];
    statev(4) = stress[1];
    statev(5) = stress[2];
    statev(6) = std::max(strain[0], statev(6));
    statev(7) = std::max(epsT, statev(7));
    statev(8) = epsQ; // TODO JBC: This is a change from the original since epsQ contains bending, while in the original epsQ is recomputed for strain only. Not sure if that makes sense. These sv are not used so the behaviro should be the same
    statev(9) = sigQ;
    statev(10) = Wint;
    statev(11) = w;
    if(mcouple_mult) {				
        statev(12) = curvature[0];
        statev(13) = curvature[1];
        statev(14) = curvature[2];	
        statev(15) = surfacic_couple[0];
        statev(16) = surfacic_couple[1];
        statev(17) = surfacic_couple[2];
    }
}
*/

void ChWoodMaterialVECT::ComputeStress(ChVector3d& dmstrain, ChVector3d& dmcurvature, ChVectorDynamic<>& eigenstrain ,double &len, StateVarVector& statev,  double& area, double& width, double& height, double& random_field, ChVector3d& mstress, ChVector3d& mcouple) {  
    	ChVector3d mstrain;
    	ChVector3d mcurvature;	
	//
	double E0=this->Get_E0();
	double alpha=this->Get_alpha();	
	if (random_field)
		E0=E0*random_field;
	//std::cout<<"E0: "<<E0<<" alpha: "<<alpha<<std::endl;	
	// 	
	mstrain=statev.segment(0,3)+dmstrain.eigen()-eigenstrain;	
	//	
	//
	double epsQ, epsQN;
	double epsT;
	double h2=height*height;		
	double w2=width*width;
	
	if (!this->GetCoupleMultiplier()){		
		epsQ = std::sqrt(mstrain[0] * mstrain[0] + alpha * (mstrain[1] * mstrain[1] + mstrain[2] * mstrain[2]));
		epsT = std::sqrt(mstrain[1] * mstrain[1] + mstrain[2] * mstrain[2]);
	}else{	
		double multiplier=this->GetCoupleMultiplier()/3.;					
		mcurvature=statev.segment(12,3)+dmcurvature.eigen();
		//std::cout<<"mstrain: "<<mstrain<<" mcurvature: "<<mcurvature<<std::endl;
		//std::cout<<"curvature: "<<mcurvature<<std::endl;		
				
		epsQ = std::sqrt(mstrain[0] * mstrain[0] + alpha * (mstrain[1] * mstrain[1] + mstrain[2] * mstrain[2])+
					multiplier*(alpha*mcurvature[0]*mcurvature[0]*(w2+h2)+ mcurvature[1]*mcurvature[1]*w2+
					mcurvature[2]*mcurvature[2]*h2));
		epsT = std::sqrt(mstrain[1] * mstrain[1] + mstrain[2] * mstrain[2]+multiplier*mcurvature[0]*mcurvature[0]*(w2+h2));
	
	}
	
	if (statev(6) < mstrain[0]) {
		statev(6) = mstrain[0];
	}
	if (statev(7) < epsT) {
		statev(7) = epsT;
	}
	
	
	if (!GetElasticAnalysisFlag()) {
	//
	// INELASTIC ANALYSIS
	//
	if (epsQ != 0) {
		if (mstrain[0] > 10e-16) {     // fracture behaivor
			double strsQ = FractureBC(mstrain, random_field, len, epsQ, epsQN, epsT, statev);
			mstress[0] = strsQ * mstrain[0] / epsQ;
			mstress[1] = alpha * strsQ * mstrain[1] / epsQ;
			mstress[2] = alpha * strsQ * mstrain[2] / epsQ;
			//
			if(this->GetCoupleMultiplier()){
				double multiplier=this->GetCoupleMultiplier()/3.;
				mcouple[0]=multiplier*alpha*strsQ*mcurvature[0]*(w2+h2)/epsQ;
				mcouple[1]=multiplier*strsQ*mcurvature[1]*w2/epsQ;
				mcouple[2]=multiplier*strsQ*mcurvature[2]*h2/epsQ;						
			}
		}
		else {
			
			 double strsQ = CompressBC(mstrain, random_field, len, epsQ, epsT, epsQN, statev);;
			 
			 mstress[0] = strsQ * mstrain[0] / epsQ;
			 mstress[1] = alpha * strsQ * mstrain[1] / epsQ;
			 mstress[2] = alpha * strsQ * mstrain[2] / epsQ;
			 //
			 if(this->GetCoupleMultiplier()){
				 double multiplier=this->GetCoupleMultiplier()/3.;
				 mcouple[0]=multiplier*alpha*strsQ*mcurvature[0]*(w2+h2)/epsQ;
				 mcouple[1]=multiplier*strsQ*mcurvature[1]*w2/epsQ;
				 mcouple[2]=multiplier*strsQ*mcurvature[2]*h2/epsQ;						
			 }
		}
		double Wint = len * area * (((mstress[0] + statev(3)) / 2.0) * dmstrain[0] + ((mstress[1] + statev(4)) / 2.0) * dmstrain[1] + ((mstress[2] + statev(5)) / 2.0) * dmstrain[2] + ((mcouple[0] + statev(15)) / 2.0) * dmcurvature[0] + ((mcouple[1] + statev(16)) / 2.0) * dmcurvature[1] + ((mcouple[2] + statev(17)) / 2.0) * dmcurvature[2]);
		double w_N = len * (mstrain[0] - mstress[0] / E0);
		double w_M = len * (mstrain[1] - mstress[1] / (E0*alpha));
		double w_L = len * (mstrain[2] - mstress[2] / (E0 * alpha));

		double w = std::sqrt(w_N * w_N + w_M * w_M + w_L * w_L);

		
		statev(8) = std::sqrt(mstrain[0] * mstrain[0] + alpha * (mstrain[1] * mstrain[1] + mstrain[2] * mstrain[2])); 
		statev(9) = std::sqrt(mstress[0] * mstress[0] + (mstress[1] * mstress[1] + mstress[2] * mstress[2]) / alpha);		
		statev(10) = Wint + statev(10);
		statev(11) = w;
		
		statev(0) = mstrain[0];
		statev(1) = mstrain[1];
		statev(2) = mstrain[2];
		statev(3) = mstress[0];
		statev(4) = mstress[1];
		statev(5) = mstress[2];
		//
		if(this->GetCoupleMultiplier()){				
				statev(12) = mcurvature[0];
				statev(13) = mcurvature[1];
				statev(14) = mcurvature[2];	
				//
				statev(15) = mcouple[0];
				statev(16) = mcouple[1];
				statev(17) = mcouple[2];
		}
	}
	else {
		mstress.Set(0.0);
		mcouple.Set(0.0);
	}
	//std::cout << "stress: " << mstress << std::endl;
	//std::cout << statev(3) << ' ' << statev(4) << ' ' << statev(5) << ' ' << statev(0) << ' ' << statev(1) << ' ' << statev(2) << std::endl;
	
	}else{
	
	//
	// ELASTIC ANALYSIS
	//
	if (epsQ!=0) {
	    double strsQ=E0*epsQ;
		mstress[0]=strsQ*mstrain[0]/epsQ;
		mstress[1]=alpha*strsQ*mstrain[1]/epsQ;
		mstress[2]=alpha*strsQ*mstrain[2]/epsQ;		
		
		
		double Wint = len * area * (((mstress[0] + statev(3)) / 2.0) * dmstrain[0] + ((mstress[1] + statev(4)) / 2.0) * dmstrain[1] + ((mstress[2] + statev(5)) / 2.0) * dmstrain[2] + ((mcouple[0] + statev(15)) / 2.0) * dmcurvature[0] + ((mcouple[1] + statev(16)) / 2.0) * dmcurvature[1] + ((mcouple[2] + statev(17)) / 2.0) * dmcurvature[2]);
		double w_N = len * (mstrain[0] - mstress[0] / E0);
		double w_M = len * (mstrain[1] - mstress[1] / (E0*alpha));
		double w_L = len * (mstrain[2] - mstress[2] / (E0 * alpha));

		double w = std::sqrt(w_N * w_N + w_M * w_M + w_L * w_L);
		
		
		statev(8) = std::sqrt(mstrain[0] * mstrain[0] + alpha * (mstrain[1] * mstrain[1] + mstrain[2] * mstrain[2])); 
		statev(9) = std::sqrt(mstress[0] * mstress[0] + (mstress[1] * mstress[1] + mstress[2] * mstress[2]) / alpha);
		statev(10) = Wint + statev(10);
		statev(11) = w;
		
		statev(0) = mstrain[0]+eigenstrain(0);
		statev(1) = mstrain[1]+eigenstrain(1);
		statev(2) = mstrain[2]+eigenstrain(2);
		statev(3) = mstress[0];
		statev(4) = mstress[1];
		statev(5) = mstress[2];
		
		
		if(this->GetCoupleMultiplier()){
			double multiplier=this->GetCoupleMultiplier()/3.;
			mcouple[0]=multiplier*alpha*strsQ*mcurvature[0]*(w2+h2)/epsQ;
			mcouple[1]=multiplier*strsQ*mcurvature[1]*w2/epsQ;
			mcouple[2]=multiplier*strsQ*mcurvature[2]*h2/epsQ;
			//
			statev(12) = mcurvature[0];
			statev(13) = mcurvature[1];
			statev(14) = mcurvature[2];
			//std::cout<<"mcurvature: "<<mcurvature.x()<<"\t"<<mcurvature.y()<<"\t"<<mcurvature.z()<<std::endl;
			//std::cout<<"mcouple: "<<mcouple.x()<<"\t"<<mcouple.y()<<"\t"<<mcouple.z()<<std::endl;
			//
			statev(15) = mcouple[0];
			statev(16) = mcouple[1];
			statev(17) = mcouple[2];
		}
		
	}else{
		mstress.Set(0.0);
		mcouple.Set(0.0);
		//statev.setZero();		
	}
	
	}
	
	//std::cout<<"mstress: "<<mstress<<"  mcouple: "<<mcouple<<" w2, h2 "<<w2<<" "<<h2<<std::endl;
	//if (mstrain[0]<0) 
	//	exit(1);
	//printf("Statev: %f, %f, %f, %f, %f, %f\n",statev(0), statev(1), statev(2), statev(3), statev(4), statev(5));
}






double ChWoodMaterialVECT::FractureBC(ChVector3d& mstrain, double& random_field, double& len, double& epsQ, double& epsQN,  double& epsT, StateVarVector& statev) {
	//
	double E0 = this->Get_E0();
	double alpha = this->Get_alpha();
	double sigmat = this->Get_sigmat();
	double sigmas = this->Get_sigmas();
	double sigmac = this->Get_sigmac();
	double nt = this->Get_nt();   
	double lt = this->Get_lt();
	if (random_field)
		sigmat=sigmat*random_field;
	//double Gt = this->Get_Gt();
	//
	//double epsQ = pow(mstrain[0] * mstrain[0] + alpha * (mstrain[1] * mstrain[1] + mstrain[2] * mstrain[2]), 0.5);
	//double epsT = pow(mstrain[1] * mstrain[1] + mstrain[2] * mstrain[2], 0.5);

	// calculate stress boudary sigma_bt for tension
	double omega, sinw, cosw, cosw2, sigma0;
	double r_st = sigmas / sigmat;
	omega = atan(epsQN / (sqrt(alpha) * epsT));

	sinw = sin(omega);
	cosw = cos(omega);
	cosw2 = cos(omega) * cos(omega);

	if (cosw2 < 10e-16) {   // epsT == 0
		//omega = CH_PI * 0.5;
		sigma0 = sigmat;
	}
	else {
		//omega = atan(mstrain[0] / (pow(alpha, 0.5) * epsT));
		sigma0 = sigmat * (-sin(omega) + sqrt((sin(omega) * sin(omega) + 4 * alpha * cosw2 / r_st / r_st))) / (2 * alpha * cosw2 / r_st / r_st);

	}

	double eps0 = sigma0 / E0;
	//double lt = 2 * E0 * Gt / sigmat / sigmat;
	double Ht = 2 * E0 / (lt / len - 1);
	double H0 = Ht * pow(2 * omega / CH_PI, nt);

	double eps_max = std::sqrt(statev(6) * statev(6) + alpha * statev(7) * statev(7));
	double sigma_bt = sigma0 * exp(-H0 * std::max((eps_max - eps0), 0.0) / sigma0);

	double strs_ela = E0 * (epsQ - statev(8)) + statev(9);
	double sigma_fr = std::min(std::max(strs_ela, 0.0), sigma_bt);
	return sigma_fr;
}


double ChWoodMaterialVECT::CompressBC(ChVector3d& mstrain, double& random_field, double& len, double& epsQ, double& epsT, double& epsQN, StateVarVector& statev) {
	//
	double E0 = this->Get_E0();
	double alpha = this->Get_alpha();
	double sigmat = this->Get_sigmat();
	double sigmas = this->Get_sigmas();
	double sigmac = this->Get_sigmac();
	double nt = this->Get_nt();   
	double lt = this->Get_lt();
	if (random_field)
		sigmat=sigmat*random_field;

	// calculate stress boudary sigma_bt for tension
	double omega, sinw, cosw, cosw2, beta2, sigma0;
	
	omega = atan(-epsQN / (pow(alpha, 0.5) * epsT));
	
	beta2 = (sigmas * sigmas) / (sigmac * sigmac);

	sinw = sin(omega);
	cosw = cos(omega);
	//sinw_2 = sin(omega) * sin(omega);
	cosw2 = cosw * cosw;

	//if (omega <= 0) {  
		//omega = CH_PI * 0.5;
		//sigma0 = sigmac;
	//}
	//else {
	sigma0 = sigmac / sqrt((sinw * sinw + (alpha * cosw2) / beta2));
	//}

	double eps0 = sigma0 / E0;
	//double lt = 2 * E0 * Gt / sigmat / sigmat;
	double H0 = 0;

	//double eps_max = epsQ;
	double sigma_bt = sigma0;
	//double sigma_bt = sigma0 * exp(-H0 * std::max((eps_max - eps0), 0.0) / sigma0);
	
	

	double strs_ela = E0 * (epsQ - statev(8)) + statev(9); 
	double sigma_fr = std::min(std::max(strs_ela, 0.0), sigma_bt);
	//std::cout<<"epsQ: "<<epsQ<<" epsQ_0: "<<statev(8)<<" sigma_0: "<<statev(9)<<" strs_ela: "<<strs_ela<<"\tsigma_bt: "<<sigma_bt<<"\tsigma_fr: "<<sigma_fr<<std::endl;
	//exit(9);
	return sigma_fr;
}


}  // end of namespace wood
}  // end of namespace chrono
