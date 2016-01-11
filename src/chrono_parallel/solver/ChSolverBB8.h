// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Hammad Mazhar
// =============================================================================
//
// This file contains an implementation of a 8 part BB based Staggered Projection solver
// =============================================================================

#pragma once

#include "chrono_parallel/ChConfigParallel.h"
#include "chrono_parallel/solver/ChSolverParallel.h"

namespace chrono {

class BBSolver {
public:
	BBSolver() {
		alpha = 0.0001;
	}
	// Set all constants and copy thing where they need to be
	void Initialize(const int size, DynamicVector<real>& gamma) {
		temp.resize(size);
		ml.resize(size);
		mg.resize(size);
		mg_p.resize(size);
		ml_candidate.resize(size);
		ms.resize(size);
		my.resize(size);
		mdir.resize(size);
		ml_p.resize(size);

		gdiff = 1.0 / Pow(size, 2.0);

		mf_p = 0;
		mf = 1e29;
		ml_candidate = ml = gamma;
		ShurProduct(ml, temp);
		mg = temp - r;
		// mg_p = mg;
		f_hist.resize(0);
		f_hist.reserve(max_iter * max_armijo_backtrace);
	}
	// Actual Solver Loop
	void Solve();

	// BB specific vectors
	DynamicVector<real> temp, ml, mg, mg_p, ml_candidate, ms, my, mdir, ml_p;
	DynamicVector<real> mD, invmD;
	std::vector<real> f_hist;
	real alpha;
	real gdiff;
	real mf_p;
	real mf;
};

class CH_PARALLEL_API ChSolverBB8: public ChSolverParallel {
public:
	ChSolverBB8();
	~ChSolverBB8() {
	}

	void Solve(const DynamicVector<real>& r,
            DynamicVector<real>& gamma) {
		//Setup Subvectors:

		SubVectorType _GAMMA_N_ = subvector(gamma, 0, _num_r_c_)
		SubVectorType _GAMMA_T_ =  subvector(gamma, _num_r_c_, 2 * _num_r_c_);

		SubVectorType _GAMMA_S_ = subvector(gamma, 3 * _num_r_c_, 3 * _num_r_c_);
		SubVectorType _GAMMA_B_ = subvector(gamma, _num_uni_,  _num_bil_);

		SubVectorType _GAMMA_RFN_ = subvector(gamma, _num_uni_ + _num_bil_, _num_rf_c_);
		SubVectorType _GAMMA_RFT_ = subvector(gamma, _num_uni_ + _num_bil_ + _num_rf_c_, 2 * _num_rf_c_);
		SubVectorType _GAMMA_FFD_ = subvector(gamma,  _num_uni_ + _num_bil_ + 3 * _num_rf_c_, _num_fluid_)
		SubVectorType _GAMMA_FFV_ = subvector(gamma,  _num_uni_ + _num_bil_ + 3 * _num_rf_c_ + _num_fluid_,  3 * _num_fluid_);

		SubVectorType _R_N_ = subvector(R, 0, _num_r_c_)
		SubVectorType _R_T_ = subvector(R, _num_r_c_, 2 * _num_r_c_);

		SubVectorType _R_S_ = subvector(R, 3 * _num_r_c_, 3 * _num_r_c_);
		SubVectorType _R_B_ = subvector(R, _num_uni_,  _num_bil_);

		SubVectorType _R_RFN_ = subvector(R, _num_uni_ + _num_bil_, _num_rf_c_);
		SubVectorType _R_RFT_ = subvector(R, _num_uni_ + _num_bil_ + _num_rf_c_, 2 * _num_rf_c_);
		SubVectorType _R_FFD_ = subvector(R,  _num_uni_ + _num_bil_ + 3 * _num_rf_c_, _num_fluid_)
		SubVectorType _R_FFV_ = subvector(R,  _num_uni_ + _num_bil_ + 3 * _num_rf_c_ + _num_fluid_,  3 * _num_fluid_);

		// Do a single solve and get everything warmed up
		Normal.Initialize(_GAMMA_N_.size(), _GAMMA_N_);
		Boundary.Initialize(_GAMMA_T_.size(), _GAMMA_T_);
		Sliding.Initialize(_GAMMA_S_.size(), _GAMMA_S_);
		Spinning.Initialize(_GAMMA_B_.size(), _GAMMA_B_);
		BoundarySliding.Initialize(_GAMMA_RFN_.size(),_GAMMA_RFN_);
		Density.Initialize(_GAMMA_RFT_.size(),_GAMMA_RFT_);
		Viscosity.Initialize(_GAMMA_FFD_.size(),_GAMMA_FFD_);
		Bilateral.Initialize(_GAMMA_FFV_.size(),_GAMMA_FFV_);

		for (int outer_iter = 0; outer_iter < 10; outer_iter++) {
			Normal.Solve();
			Boundary.Solve();
			Sliding.Solve();
			Spinning.Solve();
			BoundarySliding.Solve();
			Density.Solve();
			Viscosity.Solve();
			Bilateral.Solve();
		}
	}
	//Shur Product for Rigid normal contact
	void ShurRN(ConstSubVectorType& x, ConstSubVectorType& AX) {
		AX = _DNT_ * (_MINVDN_ * x) + _EN_ * x;
	}
	//Rigid Tangential
	void ShurRT(ConstSubVectorType& x, ConstSubVectorType& AX) {
		AX = _DTT_ * (_MINVDT_ * x) + _ET_ * x;
	}
	//Rigid Spinning
	void ShurRS(ConstSubVectorType& x, ConstSubVectorType& AX) {
		AX = _DST_ * (_MINVDS_ * x) + _ES_ * x;
	}
	//Rigid Bilateral
	void ShurBilateral(ConstSubVectorType& x, ConstSubVectorType& AX) {
		AX = _DBT_ * (_MINVDB_ * x) + _EB_ * x;
	}
	//Shur Product for Rigid fluid boundary normal contact
	void ShurRFN(ConstSubVectorType& x, ConstSubVectorType& AX) {
		AX = _DRFNT_ * (_MINVDRFN_ * x) + _ERFN_ * x;
	}
	//Rigid Fluid Sliding
	void ShurRFT(ConstSubVectorType& x, ConstSubVectorType& AX) {
		AX = _DRFTT_ * (_MINVDRFT_ * x) + _ERFT_ * x;
	}
	void ShurDensity(const DynamicVector<real>& x, DynamicVector<real>& AX) {
		AX = _DFFDT_ * (_MINVDFFDT_ * x) + _EFFD_ * x;
	}
	void ShurViscosity(const DynamicVector<real>& x, DynamicVector<real>& AX) {
		AX = _DFFVT_ * (_MINVDFFVT_ * x) + _EFFV_ * x;
	}
	// Solves normal contact and normal boundary contact with 3dof bodies
	BBSolver Normal, Boundary;
	// Solves Sliding and Spliing contact and friction boundary constraints with 3dof bodies
	BBSolver Sliding, Spinning, BoundarySliding;
	// Density constraints for fluid
	BBSolver Density;
	// Viscosity Constraints
	BBSolver Viscosity;
	// Bilateral Constraints
	BBSolver Bilateral;
};
}
