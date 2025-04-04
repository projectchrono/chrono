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
// Authors: Erol Lale, Ke Yu, Jibril B. Coulibaly
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
                                       double sigmat, double sigmas, double nt, double lt, 
									   double Ed, double sigmac0, double beta, double Hc0,
									   double Hc1, double kc0, double kc1, double kc2, double kc3,
									   double mu0, double muinf, double sigmaN0, double kt)
    : m_rho(rho), m_E0(E0), m_alpha(alpha), m_sigmat(sigmat), m_sigmas(sigmas), m_nt(nt),
		m_lt(lt), m_Ed(Ed), m_sigmac0(sigmac0), m_beta(beta), m_Hc0(Hc0), m_Hc1(Hc1),
		m_kc0(kc0), m_kc1(kc1), m_kc2(kc2), m_kc3(kc3), m_mu0(mu0), m_muinf(muinf), 
		m_sigmaN0(sigmaN0), m_kt(kt) {
        
}


ChWoodMaterialVECT::ChWoodMaterialVECT(){
    /*double m_rho=2.4E-9;
    double m_E0=30000;
    double m_alpha=0.2;*/
};

/*
// Copy constructor
ChWoodMaterialVECT::ChWoodMaterialVECT(const ChWoodMaterialVECT& my_material)
	: m_rho(my_material.m_rho), m_E0(my_material.m_E0),
		m_alpha(my_material.m_alpha)
{};
*/

// Destructor
ChWoodMaterialVECT::~ChWoodMaterialVECT()	{};

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

void ChWoodMaterialVECT::ComputeStress(ChVector3d& dmstrain, ChVector3d& dmcurvature,double &len, double &epsV, StateVarVector& statev,  double& area, double& width, double& height, ChVector3d& mstress, ChVector3d& mcouple) {  
    	ChVector3d mstrain;
    	ChVector3d mcurvature;	
	//
	double E0=this->Get_E0();
	double alpha=this->Get_alpha();	
	//std::cout<<"E0: "<<E0<<" alpha: "<<alpha<<std::endl;	
	// 	
	mstrain=statev.segment(0,3)+dmstrain.eigen();	
	//	
	//
	double epsQ;
	double epsT;
	double h2=height*height;		
	double w2=width*width;
	
	if (!this->GetCoupleMultiplier()){		
		epsQ = pow(mstrain[0] * mstrain[0] + alpha * (mstrain[1] * mstrain[1] + mstrain[2] * mstrain[2]), 0.5);
		epsT = pow(mstrain[1] * mstrain[1] + mstrain[2] * mstrain[2], 0.5);
	}else{	
		double multiplier=this->GetCoupleMultiplier()/3.;					
		mcurvature=statev.segment(12,3)+dmcurvature.eigen();
		//std::cout<<"mstrain: "<<mstrain<<" mcurvature: "<<mcurvature<<std::endl;
		//std::cout<<"curvature: "<<mcurvature<<std::endl;		
				
		epsQ = pow(mstrain[0] * mstrain[0] + alpha * (mstrain[1] * mstrain[1] + mstrain[2] * mstrain[2])+
					multiplier*(alpha*mcurvature[0]*mcurvature[0]*(w2+h2)+ mcurvature[1]*mcurvature[1]*w2+
					mcurvature[2]*mcurvature[2]*h2), 0.5);
		epsT = pow(mstrain[1] * mstrain[1] + mstrain[2] * mstrain[2]+multiplier*mcurvature[0]*mcurvature[0]*(w2+h2), 0.5);
	
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
			double strsQ = FractureBC(mstrain, len, epsQ, epsT, statev);
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
			
			 double strsQ = CompressBC(mstrain, len, epsQ, epsT, statev);;
			 
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
			 
			 /*
			 //CompressBC(mstrain, epsV, statev);
			std::vector<double> sigmaT = ShearBC(mstrain, mcurvature, (w2+h2)/3, statev);
			double sigmaM = sigmaT[0];
			double sigmaL = sigmaT[1];
			double coupleN = sigmaT[2];
						
			mstress[0]=statev(3)+(mstrain[0]-statev(0))*E0;
			mstress[1]=sigmaM;
			mstress[2]=sigmaL;
			if(this->GetCoupleMultiplier()){
				double multiplier=this->GetCoupleMultiplier()/3.;
				mcouple[0]=coupleN;
				//mcouple[1]=multiplier*sigmaN*mcurvature[1]*w2/epsQN;
				//mcouple[2]=multiplier*sigmaN*mcurvature[2]*h2/epsQN;	
				////
				//mcouple[0]=multiplier*alpha*mcurvature[0]*(w2+h2)*E0;
				//mcouple[1]=multiplier*mcurvature[1]*w2*E0;
				//mcouple[2]=multiplier*mcurvature[2]*h2*E0;
				mcouple[1]=statev(16)+multiplier*(mcurvature[1] - statev(13))*w2*E0;
				mcouple[2]=statev(17)+multiplier*(mcurvature[2] - statev(14))*h2*E0;
			}
			*/
		}
		double Wint = len * area * ((mstress[0]+ statev(3))/2 * dmstrain[0] + (mstress[1]+statev(4)) / 2 * dmstrain[1] +( mstress[2] + statev(5))/2 * dmstrain[2]);
		double w_N = len * (mstrain[0] - mstress[0] / E0);
		double w_M = len * (mstrain[1] - mstress[1] / (E0*alpha));
		double w_L = len * (mstrain[2] - mstress[2] / (E0 * alpha));

		double w = pow(w_N * w_N + w_M * w_M + w_L * w_L, 0.5);

		statev(0) = mstrain[0];
		statev(1) = mstrain[1];
		statev(2) = mstrain[2];
		statev(3) = mstress[0];
		statev(4) = mstress[1];
		statev(5) = mstress[2];
		statev(8) = pow(statev(0) * statev(0) + alpha * (statev(1) * statev(1) + statev(2) * statev(2)), 0.5); // TODO JBC: Why doesn't statev(8) take bending into account?
		statev(9) = pow(statev(3) * statev(3) + (statev(4) * statev(4) + statev(5) * statev(5)) / alpha, 0.5); // TODO JBC: Why doesn't statev(9) take bending into account?
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
		
		
		double Wint = len * area * ((mstress[0]+ statev(3))/2 * dmstrain[0] + (mstress[1]+statev(4)) / 2 * dmstrain[1] +( mstress[2] + statev(5))/2 * dmstrain[2]);
		double w_N = len * (mstrain[0] - mstress[0] / E0);
		double w_M = len * (mstrain[1] - mstress[1] / (E0*alpha));
		double w_L = len * (mstrain[2] - mstress[2] / (E0 * alpha));

		double w = pow(w_N * w_N + w_M * w_M + w_L * w_L, 0.5);
		
		statev(0) = mstrain[0];
		statev(1) = mstrain[1];
		statev(2) = mstrain[2];
		statev(3) = mstress[0];
		statev(4) = mstress[1];
		statev(5) = mstress[2];
		statev(8) = pow(statev(0) * statev(0) + alpha * (statev(1) * statev(1) + statev(2) * statev(2)), 0.5); 
		statev(9) = pow(statev(3) * statev(3) + (statev(4) * statev(4) + statev(5) * statev(5)) / alpha, 0.5);
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

void ChWoodMaterialVECT::ComputeStress_NEW(ChVector3d& strain_incr, ChVector3d& curvature_incr, double &length, StateVarVector& statev, double& width, double& height, ChVector3d& stress, ChVector3d& surfacic_couple) {  
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
            sigQ = FractureBC(strain, length, epsQ, epsT, statev);
        } else {  
            sigQ = CompressBC(strain, length, epsQ, epsT, statev);
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

void ChWoodMaterialVECT::ComputeStress(ChVector3d& dmstrain, ChVector3d& dmcurvature, ChVectorDynamic<>& eigenstrain ,double &len, double &epsV, StateVarVector& statev,  double& area, double& width, double& height, ChVector3d& mstress, ChVector3d& mcouple) {  
    	ChVector3d mstrain;
    	ChVector3d mcurvature;	
	//
	double E0=this->Get_E0();
	double alpha=this->Get_alpha();	
	//std::cout<<"E0: "<<E0<<" alpha: "<<alpha<<std::endl;	
	// 	
	mstrain=statev.segment(0,3)+dmstrain.eigen()-eigenstrain;	
	//	
	//
	double epsQ;
	double epsT;
	double h2=height*height;		
	double w2=width*width;
	
	if (!this->GetCoupleMultiplier()){		
		epsQ = pow(mstrain[0] * mstrain[0] + alpha * (mstrain[1] * mstrain[1] + mstrain[2] * mstrain[2]), 0.5);
		epsT = pow(mstrain[1] * mstrain[1] + mstrain[2] * mstrain[2], 0.5);
	}else{	
		double multiplier=this->GetCoupleMultiplier()/3.;					
		mcurvature=statev.segment(12,3)+dmcurvature.eigen();
		//std::cout<<"mstrain: "<<mstrain<<" mcurvature: "<<mcurvature<<std::endl;
		//std::cout<<"curvature: "<<mcurvature<<std::endl;		
				
		epsQ = pow(mstrain[0] * mstrain[0] + alpha * (mstrain[1] * mstrain[1] + mstrain[2] * mstrain[2])+
					multiplier*(alpha*mcurvature[0]*mcurvature[0]*(w2+h2)+ mcurvature[1]*mcurvature[1]*w2+
					mcurvature[2]*mcurvature[2]*h2), 0.5);
		epsT = pow(mstrain[1] * mstrain[1] + mstrain[2] * mstrain[2]+multiplier*mcurvature[0]*mcurvature[0]*(w2+h2), 0.5);
	
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
			double strsQ = FractureBC(mstrain, len, epsQ, epsT, statev);
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
			
			 double strsQ = CompressBC(mstrain, len, epsQ, epsT, statev);;
			 
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
			 
			 /*
			 //CompressBC(mstrain, epsV, statev);
			std::vector<double> sigmaT = ShearBC(mstrain, mcurvature, (w2+h2)/3, statev);
			double sigmaM = sigmaT[0];
			double sigmaL = sigmaT[1];
			double coupleN = sigmaT[2];
						
			//mstress[0]=statev(3)+(mstrain[0]-statev(0))*E0;
			mstress[0]=statev(3)+dmstrain[0]*E0;
			mstress[1]=sigmaM;
			mstress[2]=sigmaL;
			if(this->GetCoupleMultiplier()){
				double multiplier=this->GetCoupleMultiplier()/3.;
				mcouple[0]=coupleN;
				//mcouple[1]=multiplier*sigmaN*mcurvature[1]*w2/epsQN;
				//mcouple[2]=multiplier*sigmaN*mcurvature[2]*h2/epsQN;	
				////
				//mcouple[0]=multiplier*alpha*mcurvature[0]*(w2+h2)*E0;
				//mcouple[1]=multiplier*mcurvature[1]*w2*E0;
				//mcouple[2]=multiplier*mcurvature[2]*h2*E0;
				//mcouple[1]=statev(16)+multiplier*(mcurvature[1] - statev(13))*w2*E0;
				//mcouple[2]=statev(17)+multiplier*(mcurvature[2] - statev(14))*h2*E0;
				mcouple[1]=statev(16)+multiplier*dmcurvature[1] * w2*E0;
				mcouple[2]=statev(17)+multiplier*dmcurvature[2] * h2*E0;
			}
			*/
		}
		double Wint = len * area * ((mstress[0]+ statev(3))/2 * dmstrain[0] + (mstress[1]+statev(4)) / 2 * dmstrain[1] +( mstress[2] + statev(5))/2 * dmstrain[2]);
		double w_N = len * (mstrain[0] - mstress[0] / E0);
		double w_M = len * (mstrain[1] - mstress[1] / (E0*alpha));
		double w_L = len * (mstrain[2] - mstress[2] / (E0 * alpha));

		double w = pow(w_N * w_N + w_M * w_M + w_L * w_L, 0.5);

		
		statev(8) = pow(mstrain[0] * mstrain[0] + alpha * (mstrain[1] * mstrain[1] + mstrain[2] * mstrain[2]), 0.5); 
		statev(9) = pow(mstress[0] * mstress[0] + (mstress[1] * mstress[1] + mstress[2] * mstress[2]) / alpha, 0.5);		
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
		
		
		double Wint = len * area * ((mstress[0]+ statev(3))/2 * dmstrain[0] + (mstress[1]+statev(4)) / 2 * dmstrain[1] +( mstress[2] + statev(5))/2 * dmstrain[2]);
		double w_N = len * (mstrain[0] - mstress[0] / E0);
		double w_M = len * (mstrain[1] - mstress[1] / (E0*alpha));
		double w_L = len * (mstrain[2] - mstress[2] / (E0 * alpha));

		double w = pow(w_N * w_N + w_M * w_M + w_L * w_L, 0.5);
		
		
		statev(8) = pow(mstrain[0] * mstrain[0] + alpha * (mstrain[1] * mstrain[1] + mstrain[2] * mstrain[2]), 0.5); 
		statev(9) = pow(mstress[0] * mstress[0] + (mstress[1] * mstress[1] + mstress[2] * mstress[2]) / alpha, 0.5);
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






double ChWoodMaterialVECT::FractureBC(ChVector3d& mstrain, double& len, double& epsQ, double& epsT, StateVarVector& statev) {
	//
	double E0 = this->Get_E0();
	double alpha = this->Get_alpha();
	double sigmat = this->Get_sigmat();
	double sigmas = this->Get_sigmas();
	double nt = this->Get_nt();   
	double lt = this->Get_lt();
	double kt = this->Get_kt();
	//double Gt = this->Get_Gt();
	//
	//double epsQ = pow(mstrain[0] * mstrain[0] + alpha * (mstrain[1] * mstrain[1] + mstrain[2] * mstrain[2]), 0.5);
	//double epsT = pow(mstrain[1] * mstrain[1] + mstrain[2] * mstrain[2], 0.5);

	// calculate stress boudary sigma_bt for tension
	double omega, sinw, cosw, cosw2, sigma0;
	double r_st = sigmas / sigmat;
	
	if (this->GetCoupleContrib2EquStrainFlag()){
		omega = atan(epsQ / (pow(alpha, 0.5) * epsT));
	}else{
		omega = atan(mstrain[0] / pow(alpha*(mstrain[1]*mstrain[1]+mstrain[2]*mstrain[2]), 0.5));
	}
	
	sinw = sin(omega);
	cosw = cos(omega);
	cosw2 = cos(omega) * cos(omega);

	if (cosw2 < 10e-16) {   // epsT == 0
		//omega = CH_PI * 0.5;
		sigma0 = sigmat;
	}
	else {
		//omega = atan(mstrain[0] / (pow(alpha, 0.5) * epsT));
		sigma0 = sigmat * (-sin(omega) + pow((sin(omega) * sin(omega) + 4 * alpha * cosw2 / r_st / r_st), 0.5)) / (2 * alpha * cosw2 / r_st / r_st);

	}

	double eps0 = sigma0 / E0;
	//double lt = 2 * E0 * Gt / sigmat / sigmat;
	double Ht = 2 * E0 / (lt / len - 1);
	double H0 = Ht * pow(2 * omega / CH_PI, nt);

	double eps_max = pow(statev(6) * statev(6) + alpha * statev(7) * statev(7), 0.5);
	double sigma_bt = sigma0 * exp(-H0 * std::max((eps_max - eps0), 0.0) / sigma0);

	
	double eps_tr = kt * (eps_max - sigma_bt / E0);
	if (eps_tr > epsQ) {
		sigma_bt = 0;
	}
	
	

	double strs_ela = E0 * (epsQ - statev(8)) + statev(9);
	double sigma_fr = std::min(std::max(strs_ela, 0.0), sigma_bt);
	
	return sigma_fr;
}


double ChWoodMaterialVECT::CompressBC(ChVector3d& mstrain, double& len, double& epsQ, double& epsT, StateVarVector& statev) {
	//
	double E0 = this->Get_E0();
	double alpha = this->Get_alpha();
	double sigmat = this->Get_sigmat();
	double sigmas = this->Get_sigmas();
	double sigmac0 = this->Get_sigmac0();
	double beta = this->Get_beta();
	double nt = this->Get_nt();   
	double lt = this->Get_lt();
	double kt = this->Get_kt();
	
	// compression BC
	/*double Ed = this->Get_Ed();
	double Hc0 = this->Get_Hc0();
	double Hc1 = this->Get_Hc1();
	double kc0 = this->Get_kc0();
	double kc1 = this->Get_kc1();
	double kc2 = this->Get_kc2();*/
	
	//Shear BC
	/*double mu0 = this->Get_mu0();
	double muinf = this->Get_muinf();
	double sigmaN0 = this->Get_sigmaN0();
	double multiplier=this->GetCoupleMultiplier();*/

	//double Gt = this->Get_Gt();
	//
	//double epsQ = pow(mstrain[0] * mstrain[0] + alpha * (mstrain[1] * mstrain[1] + mstrain[2] * mstrain[2]), 0.5);
	//double epsT = pow(mstrain[1] * mstrain[1] + mstrain[2] * mstrain[2], 0.5);

	// calculate stress boudary sigma_bt for tension
	double omega, sinw, cosw, cosw2, beta2, sigma0;
	
	if (this->GetCoupleContrib2EquStrainFlag()){
		omega = atan(epsQ / (pow(alpha, 0.5) * epsT));
	}else{
		omega = atan(mstrain[0] / pow(alpha*(mstrain[1]*mstrain[1]+mstrain[2]*mstrain[2]), 0.5));
	}
	
	beta2 = (sigmas * sigmas) / (sigmac0 * sigmac0);

	sinw = sin(omega);
	cosw = cos(omega);
	//sinw_2 = sin(omega) * sin(omega);
	cosw2 = cosw * cosw;

	//if (omega <= 0) {  
		//omega = CH_PI * 0.5;
		//sigma0 = sigmac0;
	//}
	//else {
	sigma0 = sigmac0 / pow((sinw * sinw + (alpha * cosw2) / beta2), 0.5);
	//}

	double eps0 = sigma0 / E0;
	//double lt = 2 * E0 * Gt / sigmat / sigmat;
	double H0 = 0;

	double eps_max = epsQ;
	double sigma_bt = sigma0;
	//double sigma_bt = sigma0 * exp(-H0 * std::max((eps_max - eps0), 0.0) / sigma0);
	
	

	double strs_ela = E0 * (epsQ - statev(8)) + statev(9); 
	double sigma_fr = std::min(std::max(strs_ela, 0.0), sigma_bt);
	//std::cout<<"epsQ: "<<epsQ<<" epsQ_0: "<<statev(8)<<" sigma_0: "<<statev(9)<<" strs_ela: "<<strs_ela<<"\tsigma_bt: "<<sigma_bt<<"\tsigma_fr: "<<sigma_fr<<std::endl;
	//exit(9);
	return sigma_fr;
}


/*
double ChWoodMaterialVECT::CompressBC(ChVectorDynamic<>& mstrain, double& epsV, ChVectorDynamic<>& statev) {
	double E0 = this->Get_E0();
	double Ed = this->Get_Ed();
	double sigmac0 = this->Get_sigmac0();
	double beta = this->Get_beta();
	double Hc0 = this->Get_Hc0();
	double Hc1 = this->Get_Hc1();
	double kc0 = this->Get_kc0();
	double kc1 = this->Get_kc1();
	double kc2 = this->Get_kc2();
	double kc3 = this->Get_kc3();

	double epsD = mstrain[0] - epsV;
	double epsDV = epsV + beta * epsD;
	double epsc0 = sigmac0 / E0;
	double epsc1 = epsc0 * kc0;
	double epsv0 = epsc0 * kc3;

	//std::cout << "epsV " << epsV << std::endl;

	double r_DV;
	if (epsV <= 0) {
		r_DV = -abs(epsD) / (epsV - epsv0);
	}
	else {
		r_DV = abs(epsD) / epsv0;
	}

	double Hc = (Hc0 - Hc1) / (1 + kc2 * std::max(r_DV - kc1, 0.0)) + Hc1;
	double sigmac1 = sigmac0 + (epsc1 - epsc0) * Hc;

	double sigma_bc;
	if (-epsDV <= 0) {
		sigma_bc = sigmac0;
	}
	else if (-epsDV > 0 && -epsDV <= epsc1) {
		sigma_bc = sigmac0 + std::max((-epsDV - epsc0), 0.0) * Hc;
		//std::cout << "case2 "  << std::endl;
	}
	else {
		sigma_bc = sigmac1 * exp((-epsDV - epsc1) * Hc / sigmac1);
		//std::cout << "case3 " << std::endl;
	}

	//std::cout << "epsDV " << epsDV << std::endl;
	//std::cout << "Hc " << Hc << std::endl;
	//std::cout << "epsc1 " << epsc1 << std::endl;

	double ENc;
	if (-statev(3) < sigmac0) {
		ENc = E0;
	}
	else {
		ENc = Ed;
	}

	double sigma_ela = statev(3) + ENc * (mstrain[0] - statev(0));
	double sigma_com = std::min(std::max(sigma_ela, -sigma_bc), 0.0);
	return sigma_com;
}
*/



std::pair<double, double> ChWoodMaterialVECT::ShearBC(ChVector3d& mstrain, StateVarVector& statev) {
	double E0 = this->Get_E0();
	double alpha = this->Get_alpha();
	double mu0 = this->Get_mu0();
	double muinf = this->Get_muinf();
	double sigmas = this->Get_sigmas();
	double sigmaN0 = this->Get_sigmaN0();

	double sigmabs = sigmas + (mu0 - muinf) * sigmaN0 - muinf * statev(3) - (mu0 - muinf) * sigmaN0 * exp(statev(3) / sigmaN0);
	//double sigmabs = sigmas - muinf * statev(3);
	
	double ET = alpha * E0;
	double sigmaM_ela = statev(4) + (mstrain[1] - statev(1)) * ET;
	double sigmaL_ela = statev(5) + (mstrain[2] - statev(2)) * ET;
	double sigmaT_ela = pow(sigmaM_ela * sigmaM_ela + sigmaL_ela * sigmaL_ela, 0.5);

	double sigmaT = std::min(std::max(sigmaT_ela, 0.0), sigmabs);

	double sigmaM, sigmaL;

	if (sigmaT_ela != 0) {
		sigmaM = sigmaT * sigmaM_ela / sigmaT_ela;
		sigmaL = sigmaT * sigmaL_ela / sigmaT_ela;
	}
	else {
		sigmaM = 0;
		sigmaL = 0;
	}
	
	return std::make_pair(sigmaM, sigmaL);
}



std::vector<double> ChWoodMaterialVECT::ShearBC(ChVector3d& mstrain, ChVector3d& mcurvature, double rN2, StateVarVector& statev) {
	double E0 = this->Get_E0();
	double alpha = this->Get_alpha();
	double mu0 = this->Get_mu0();
	double muinf = this->Get_muinf();
	double sigmas = this->Get_sigmas();
	double sigmaN0 = this->Get_sigmaN0();
	double multiplier=this->GetCoupleMultiplier();

	//double sigmabs = sigmas + (mu0 - muinf) * sigmaN0 - muinf * statev(3) - (mu0 - muinf) * sigmaN0 * exp(statev(3) / sigmaN0);
	double sigmabs = sigmas - mu0 * statev(3);
	
	double ET = alpha * E0;
	double sigmaM_ela = statev(4) + (mstrain[1] - statev(1)) * ET;
	double sigmaL_ela = statev(5) + (mstrain[2] - statev(2)) * ET;
	double coupleN_ela = statev(15) + multiplier*(mcurvature[0] - statev(12)) * rN2* ET;	
	double sigmaT_ela = pow(sigmaM_ela * sigmaM_ela + sigmaL_ela * sigmaL_ela + coupleN_ela* coupleN_ela/rN2/multiplier, 0.5);

	double sigmaT = std::min(std::max(sigmaT_ela, 0.0), sigmabs);
	//std::cout<<"sigmaM_ela: "<<sigmaM_ela<<"  sigmaL_ela: "<<sigmaL_ela<<" coupleN_ela: "<<coupleN_ela<< std::endl;
	//std::cout<<"sigmaT: "<<sigmaT<<"  sigmaT_ela: "<<sigmaT_ela<<std::endl;
	double sigmaM, sigmaL, coupleN;

	if (sigmaT_ela != 0) {
		sigmaM = sigmaT * sigmaM_ela / sigmaT_ela;
		sigmaL = sigmaT * sigmaL_ela / sigmaT_ela;
		coupleN = sigmaT * coupleN_ela  / sigmaT_ela;
	}
	else {
		sigmaM = 0;
		sigmaL = 0;
		coupleN = 0;
	}
	
	return {sigmaM, sigmaL, coupleN}; //std::make_pair(sigmaM, sigmaL);
}


}  // end of namespace wood
}  // end of namespace chrono
