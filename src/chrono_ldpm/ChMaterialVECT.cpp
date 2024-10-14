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
// Authors: Erol Lale
//			Ke Yu
// =============================================================================
// Material class for LDPM and CSL elements 
//
// A description of the material parameter can be found in: https://doi.org/10.1016/j.cemconcomp.2011.02.011
// =============================================================================

#include "chrono_ldpm/ChMaterialVECT.h"

namespace chrono {
namespace ldpm {

// Construct an isotropic material.

ChMaterialVECT::ChMaterialVECT(double rho,  // material density
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


ChMaterialVECT::ChMaterialVECT(){
    /*double m_rho=2.4E-9;
    double m_E0=30000;
    double m_alpha=0.2;*/
};

/*
// Copy constructor
ChMaterialVECT::ChMaterialVECT(const ChMaterialVECT& my_material)
	: m_rho(my_material.m_rho), m_E0(my_material.m_E0),
		m_alpha(my_material.m_alpha)
{};
*/

// Destructor
ChMaterialVECT::~ChMaterialVECT()	{};

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

void ChMaterialVECT::ComputeStress(ChVectorDynamic<>& dmstrain, double &len, double &epsV, ChVectorDynamic<>& statev, ChVectorDynamic<>& mstress, double& area) {
    	mstress.resize(3);    	
	//
	double E0=this->Get_E0();
	double alpha=this->Get_alpha();	
	// 
	ChVectorDynamic<> mstrain(3);
	mstrain=statev.segment(0,3)+dmstrain;	
	//
	double epsQ = pow(mstrain(0) * mstrain(0) + alpha * (mstrain(1) * mstrain(1) + mstrain(2) * mstrain(2)), 0.5);
	double epsT = pow(mstrain(1) * mstrain(1) + mstrain(2) * mstrain(2), 0.5);
	
	if (statev(6) < mstrain(0)) {
		statev(6) = mstrain(0);
	}
	if (statev(7) < epsT) {
		statev(7) = epsT;
	}
	
	if (epsQ != 0) {
		if (mstrain(0) > 10e-16) {     // fracture behaivor
			double strsQ = FractureBC(mstrain, len, statev);
			mstress(0) = strsQ * mstrain(0) / epsQ;
			mstress(1) = alpha * strsQ * mstrain(1) / epsQ;
			mstress(2) = alpha * strsQ * mstrain(2) / epsQ;
		}
		else {
			double sigmaN = CompressBC(mstrain, epsV, statev);
			std::pair<double, double> sigmaT = ShearBC(mstrain, statev);
			double sigmaM = sigmaT.first;
			double sigmaL = sigmaT.second;
			//double sigmaTpre = pow(statev(4) * statev(4) + statev(5) * statev(5), 0.5);
			//double sigmaM = sigmaT * statev(4) / sigmaTpre;
			//double sigmaL = sigmaT * statev(5) / sigmaTpre;

			mstress(0) = sigmaN;
			mstress(1) = sigmaM;
			mstress(2) = sigmaL;
		}
		double Wint = len * area * ((mstress(0)+ statev(3))/2 * dmstrain(0) + (mstress(1)+statev(4)) / 2 * dmstrain(1) +( mstress(2) + statev(5))/2 * dmstrain(2));
		double w_N = len * (mstrain(0) - mstress(0) / E0);
		double w_M = len * (mstrain(1) - mstress(1) / (E0*alpha));
		double w_L = len * (mstrain(2) - mstress(2) / (E0 * alpha));

		double w = pow(w_N * w_N + w_M * w_M + w_L * w_L, 0.5);

		statev(0) = mstrain(0);
		statev(1) = mstrain(1);
		statev(2) = mstrain(2);
		statev(3) = mstress(0);
		statev(4) = mstress(1);
		statev(5) = mstress(2);
		statev(8) = pow(statev(0) * statev(0) + alpha * (statev(1) * statev(1) + statev(2) * statev(2)), 0.5);
		statev(9) = pow(statev(3) * statev(3) + (statev(4) * statev(4) + statev(5) * statev(5)) / alpha, 0.5);
		statev(10) = Wint + statev(10);
		statev(11) = w;
	}
	else {
		mstress << 0.0, 0.0, 0.0;
	}
	//std::cout << "stress: " << mstress << std::endl;
	//std::cout << statev(3) << ' ' << statev(4) << ' ' << statev(5) << ' ' << statev(0) << ' ' << statev(1) << ' ' << statev(2) << std::endl;
	

	/*
	if (epsQ!=0) {
	        double strsQ=E0*epsQ;
		mstress(0)=strsQ*mstrain(0)/epsQ;
		mstress(1)=alpha*strsQ*mstrain(1)/epsQ;
		mstress(2)=alpha*strsQ*mstrain(2)/epsQ;
		
		
		double Wint = len * area * ((mstress(0)+ statev(3))/2 * dmstrain(0) + (mstress(1)+statev(4)) / 2 * dmstrain(1) +( mstress(2) + statev(5))/2 * dmstrain(2));
		double w_N = len * (mstrain(0) - mstress(0) / E0);
		double w_M = len * (mstrain(1) - mstress(1) / (E0*alpha));
		double w_L = len * (mstrain(2) - mstress(2) / (E0 * alpha));


		double w = pow(w_N * w_N + w_M * w_M + w_L * w_L, 0.5);
		
		
		statev(0) = mstrain(0);
		statev(1) = mstrain(1);
		statev(2) = mstrain(2);
		statev(3) = mstress(0);
		statev(4) = mstress(1);
		statev(5) = mstress(2);
		statev(8) = pow(statev(0) * statev(0) + alpha * (statev(1) * statev(1) + statev(2) * statev(2)), 0.5);
		statev(9) = pow(statev(3) * statev(3) + (statev(4) * statev(4) + statev(5) * statev(5)) / alpha, 0.5);
		statev(10) = Wint + statev(10);
		statev(11) = w;
		
	}else{
		mstress<< 0.0, 0.0, 0.0;
		//statev.setZero();		
		//mstress.setZero();
	}
	*/
	//printf("Statev: %f, %f, %f, %f, %f, %f\n",statev(0), statev(1), statev(2), statev(3), statev(4), statev(5));
}


void ChMaterialVECT::ComputeStress(ChVectorDynamic<>& dmstrain, ChVectorDynamic<>& eigenstrain, double &len, double &epsV, ChVectorDynamic<>& statev, ChVectorDynamic<>& mstress, double& area) {
    	mstress.resize(3);    	
	//
	double E0=this->Get_E0();
	double alpha=this->Get_alpha();	
	// 
	ChVectorDynamic<> mstrain(3);
	mstrain=statev.segment(0,3)+dmstrain;
	//	
	//
	ChVectorDynamic<> netstrain=mstrain-eigenstrain;
	ChVectorDynamic<> netdmstrain=netstrain-statev.segment(12,3);	
	//std::cout<<"eigenstrain: "<<eigenstrain.x()<<"\t"<<eigenstrain.y()<<"\t"<<eigenstrain.z()<<"\t"	;
	//std::cout<<"mstrain: "<<mstrain.x()<<"\t"<<mstrain.y()<<"\t"<<mstrain.z()<<"\t"	;
	///std::cout<<"dmstrain: "<<dmstrain.x()<<"\t"<<dmstrain.y()<<"\t"<<dmstrain.z()<<"\t"	;
	//std::cout<<"netstrain: "<<netstrain.x()<<"\t"<<netstrain.y()<<"\t"<<netstrain.z()<<"\n";
	//
	double epsQ = pow(netstrain(0) * netstrain(0) + alpha * (netstrain(1) * netstrain(1) + netstrain(2) * netstrain(2)), 0.5);
	double epsT = pow(netstrain(1) * netstrain(1) + netstrain(2) * netstrain(2), 0.5);
	//std::cout<<"epsQ: "<<epsQ<<"\n";
	
	if (statev(6) < netstrain(0)) {
		statev(6) = netstrain(0);
	}
	if (statev(7) < epsT) {
		statev(7) = epsT;
	}
	
	if (epsQ != 0) {
		if (netstrain(0) > 10e-16) {     // fracture behaivor
			double strsQ = FractureBC(netstrain, len, statev);
			mstress(0) = strsQ * netstrain(0) / epsQ;
			mstress(1) = alpha * strsQ * netstrain(1) / epsQ;
			mstress(2) = alpha * strsQ * netstrain(2) / epsQ;
		}
		else {
			double sigmaN = CompressBC(netstrain, netdmstrain, epsV, statev);
			std::pair<double, double> sigmaT = ShearBC(netstrain, netdmstrain, statev);
			double sigmaM = sigmaT.first;
			double sigmaL = sigmaT.second;
			//double sigmaTpre = pow(statev(4) * statev(4) + statev(5) * statev(5), 0.5);
			//double sigmaM = sigmaT * statev(4) / sigmaTpre;
			//double sigmaL = sigmaT * statev(5) / sigmaTpre;
			
			mstress(0) = sigmaN;
			mstress(1) = sigmaM;
			mstress(2) = sigmaL;
			
		}
		double Wint = len * area * ((mstress(0)+ statev(3))/2 * netdmstrain(0) + (mstress(1)+statev(4)) / 2 * netdmstrain(1) +( mstress(2) + statev(5))/2 * netdmstrain(2));
		double w_N = len * (netstrain(0) - mstress(0) / E0);
		double w_M = len * (netstrain(1) - mstress(1) / (E0*alpha));
		double w_L = len * (netstrain(2) - mstress(2) / (E0 * alpha));


		double w = pow(w_N * w_N + w_M * w_M + w_L * w_L, 0.5);

		statev(0) = mstrain(0);
		statev(1) = mstrain(1);
		statev(2) = mstrain(2);
		statev(3) = mstress(0);
		statev(4) = mstress(1);
		statev(5) = mstress(2);
		statev(8) = pow(netstrain(0) * netstrain(0) + alpha * (netstrain(1) * netstrain(1) + netstrain(2) * netstrain(2)), 0.5);
		statev(9) = pow(mstress(0) * mstress(0) + (mstress(1) * mstress(1) + mstress(2) * mstress(2)) / alpha, 0.5);
		statev(10) = Wint + statev(10);
		statev(11) = w;
		statev(12) = netstrain(0);
		statev(13) = netstrain(1);
		statev(14) = netstrain(2);
	}
	else {
		mstress << 0.0, 0.0, 0.0;
	}
	//std::cout << "stress: " << mstress << std::endl;
	//std::cout << statev(3) << ' ' << statev(4) << ' ' << statev(5) << ' ' << statev(0) << ' ' << statev(1) << ' ' << statev(2) << std::endl;
	

	/*
	if (epsQ!=0) {
	        double strsQ=E0*epsQ;	        
	        
		mstress(0)=strsQ*netstrain(0)/epsQ;
		mstress(1)=alpha*strsQ*netstrain(1)/epsQ;
		mstress(2)=alpha*strsQ*netstrain(2)/epsQ;
		
		
		double Wint = len * area * ((mstress(0)+ statev(3))/2 * dmstrain(0) + (mstress(1)+statev(4)) / 2 * dmstrain(1) +( mstress(2) + statev(5))/2 * dmstrain(2));
		double w_N = len * (mstrain(0) - mstress(0) / E0);
		double w_M = len * (mstrain(1) - mstress(1) / (E0*alpha));
		double w_L = len * (mstrain(2) - mstress(2) / (E0 * alpha));

		double w = pow(w_N * w_N + w_M * w_M + w_L * w_L, 0.5);
		
		
		statev(0) = mstrain(0);
		statev(1) = mstrain(1);
		statev(2) = mstrain(2);
		statev(3) = mstress(0);
		statev(4) = mstress(1);
		statev(5) = mstress(2);
		statev(8) = pow(statev(0) * statev(0) + alpha * (statev(1) * statev(1) + statev(2) * statev(2)), 0.5);
		statev(9) = pow(statev(3) * statev(3) + (statev(4) * statev(4) + statev(5) * statev(5)) / alpha, 0.5);
		statev(10) = Wint + statev(10);
		statev(11) = w;
		
	}else{
		mstress<< 0.0, 0.0, 0.0;

		//statev.setZero();		
		//mstress.setZero();
	}
	*/
	//printf("Statev: %f, %f, %f, %f, %f, %f\n",statev(0), statev(1), statev(2), statev(3), statev(4), statev(5));
}

double ChMaterialVECT::FractureBC(ChVectorDynamic<>& mstrain, double& len, ChVectorDynamic<>& statev) {
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
	double epsQ = pow(mstrain(0) * mstrain(0) + alpha * (mstrain(1) * mstrain(1) + mstrain(2) * mstrain(2)), 0.5);
	double epsT = pow(mstrain(1) * mstrain(1) + mstrain(2) * mstrain(2), 0.5);

	// calculate stress boudary sigma_bt for tension
	double omega, sinw, cosw, cosw2, sigma0;
	double r_st = sigmas / sigmat;

	omega = atan(mstrain(0) / (pow(alpha, 0.5) * epsT));
	sinw = sin(omega);
	cosw = cos(omega);
	cosw2 = cos(omega) * cos(omega);

	if (cosw2 < 10e-16) {   // epsT == 0
		//omega = M_PI * 0.5;
		sigma0 = sigmat;
	}
	else {
		//omega = atan(mstrain(0) / (pow(alpha, 0.5) * epsT));
		sigma0 = sigmat * (-sin(omega) + pow((sin(omega) * sin(omega) + 4 * alpha * cosw2 / r_st / r_st), 0.5)) / (2 * alpha * cosw2 / r_st / r_st);

	}

	double eps0 = sigma0 / E0;
	//double lt = 2 * E0 * Gt / sigmat / sigmat;
	double Ht = 2 * E0 / (lt / len - 1);
	double H0 = Ht * pow(2 * omega / M_PI, nt);

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

double ChMaterialVECT::CompressBC(ChVectorDynamic<>& mstrain, double& epsV, ChVectorDynamic<>& statev) {
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

	double epsD = mstrain(0) - epsV;
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

	double sigma_ela = statev(3) + ENc * (mstrain(0) - statev(0));
	double sigma_com = std::min(std::max(sigma_ela, -sigma_bc), 0.0);
	return sigma_com;
}

std::pair<double, double> ChMaterialVECT::ShearBC(ChVectorDynamic<>& mstrain, ChVectorDynamic<>& statev) {
	double E0 = this->Get_E0();
	double alpha = this->Get_alpha();
	double mu0 = this->Get_mu0();
	double muinf = this->Get_muinf();
	double sigmas = this->Get_sigmas();
	double sigmaN0 = this->Get_sigmaN0();

	double sigmabs = sigmas + (mu0 - muinf) * sigmaN0 - muinf * statev(3) - (mu0 - muinf) * sigmaN0 * exp(statev(3) / sigmaN0);

	double ET = alpha * E0;
	double sigmaM_ela = statev(4) + (mstrain(1) - statev(1)) * ET;
	double sigmaL_ela = statev(5) + (mstrain(2) - statev(2)) * ET;
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



double ChMaterialVECT::CompressBC(ChVectorDynamic<>& mstrain, ChVectorDynamic<>& dmstrain, double& epsV, ChVectorDynamic<>& statev) {
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

	double epsD = mstrain(0) - epsV;
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

	double sigma_ela = statev(3) + ENc * (dmstrain(0));
	double sigma_com = std::min(std::max(sigma_ela, -sigma_bc), 0.0);
	return sigma_com;
}

std::pair<double, double> ChMaterialVECT::ShearBC(ChVectorDynamic<>& mstrain, ChVectorDynamic<>& dmstrain, ChVectorDynamic<>& statev) {
	double E0 = this->Get_E0();
	double alpha = this->Get_alpha();
	double mu0 = this->Get_mu0();
	double muinf = this->Get_muinf();
	double sigmas = this->Get_sigmas();
	double sigmaN0 = this->Get_sigmaN0();

	double sigmabs = sigmas + (mu0 - muinf) * sigmaN0 - muinf * statev(3) - (mu0 - muinf) * sigmaN0 * exp(statev(3) / sigmaN0);

	double ET = alpha * E0;
	double sigmaM_ela = statev(4) + (dmstrain(1) ) * ET;
	double sigmaL_ela = statev(5) + (dmstrain(2) ) * ET;
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


}  // end of namespace ldpm
}  // end of namespace chrono
