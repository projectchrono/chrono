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


#include "chrono_modal/ChModalDamping.h"
#include "chrono_modal/ChModalAssembly.h"
#include "chrono_modal/ChEigenvalueSolver.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/fea/ChNodeFEAxyz.h"
#include "chrono/fea/ChNodeFEAxyzrot.h"

namespace chrono {

using namespace fea;
using namespace collision;
using namespace geometry;

namespace modal {



ChModalDampingReductionR::ChModalDampingReductionR(ChModalAssembly& massembly) { 
        massembly.GetSubassemblyDampingMatrix(&full_R); 
}


void ChModalDampingFactorRmm::ComputeR(ChModalAssembly& assembly, 
        const ChMatrixDynamic<>& modal_M, 
        const ChMatrixDynamic<>& modal_K, 
        const ChMatrixDynamic<>& Psi, 
        ChMatrixDynamic<>& modal_R) const {

    int n_mod_coords = assembly.Get_n_modes_coords_w();
    int n_bou_coords = assembly.GetN_boundary_coords_w();
     
    ChVectorDynamic<> omegas = CH_C_2PI * assembly.Get_modes_frequencies();
    ChVectorDynamic<> zetas;
    zetas.setZero(n_mod_coords);

    if (this->damping_factors.size() == n_mod_coords)
        zetas = this->damping_factors;
    if (this->damping_factors.size() > n_mod_coords)
        zetas = this->damping_factors.segment(0,n_mod_coords);
    if (this->damping_factors.size() < n_mod_coords) {
        zetas.segment(0, this->damping_factors.size()) = this->damping_factors;
        for (int i = this->damping_factors.size(); i < n_mod_coords; ++i) {
            zetas(i) = zetas(this->damping_factors.size() - 1); // repeat last of user provided factors
        }
    }

    modal_R.setZero(modal_M.rows(), modal_M.cols());

    modal_R.block(n_bou_coords, n_bou_coords, n_mod_coords, n_mod_coords) = (2.0 * omegas.cwiseProduct(zetas)).asDiagonal();
}

void ChModalDampingFactorRayleigh::ComputeR(ChModalAssembly& assembly, 
        const ChMatrixDynamic<>& modal_M, 
        const ChMatrixDynamic<>& modal_K, 
        const ChMatrixDynamic<>& Psi, 
        ChMatrixDynamic<>& modal_R) const {
    
    // For the Rmm block: inherit parent function
    ChModalDampingFactorRmm::ComputeR(assembly, modal_M, modal_K, Psi, modal_R);

    // For the Rbb block: 
    int n_bou_coords = assembly.GetN_boundary_coords_w();

    modal_R.block(0, 0, n_bou_coords, n_bou_coords) = 
        alpha * modal_M.block(0, 0, n_bou_coords, n_bou_coords) +
        beta  * modal_K.block(0, 0, n_bou_coords, n_bou_coords);
}


void ChModalDampingFactorAssembly::ComputeR(ChModalAssembly& assembly, 
        const ChMatrixDynamic<>& modal_M, 
        const ChMatrixDynamic<>& modal_K, 
        const ChMatrixDynamic<>& Psi, 
        ChMatrixDynamic<>& modal_R) const {
    throw(ChException("ChModalDampingFactorAssembly not ready for production, cause: a) ok in debug bust wrong in release mode, b) not all modes are used, only first 6."));

    int n_mod_coords = assembly.Get_n_modes_coords_w();
    int n_bou_coords = assembly.GetN_boundary_coords_w();
    int n_bou_mod_coords = n_bou_coords + n_mod_coords;

    ChMatrixDynamic<std::complex<double>> modes_V_reduced(n_bou_mod_coords, n_bou_mod_coords);
    ChVectorDynamic<std::complex<double>> eig_reduced(n_bou_mod_coords);
    ChVectorDynamic<> freq_reduced(n_bou_mod_coords);
    ChSparseMatrix Cq_reduced;
    assembly.GetSubassemblyConstraintJacobianMatrix(&Cq_reduced);

    ChGeneralizedEigenvalueSolverLanczos     eigsolver;   // OK
    //ChGeneralizedEigenvalueSolverKrylovSchur eigsolver;   // OK 
    eigsolver.Solve(modal_M.sparseView(), modal_K.sparseView(), Cq_reduced, modes_V_reduced, eig_reduced, freq_reduced, 6); //***TODO*** must be all modes n_bou_mod_coords, not only first 6!;

    ChVectorDynamic<> omegas = CH_C_2PI * freq_reduced;
    ChVectorDynamic<> zetas;
    zetas.setZero(n_bou_mod_coords);

    if (this->damping_factors.size() == n_bou_mod_coords)
        zetas = this->damping_factors;
    if (this->damping_factors.size() > n_bou_mod_coords)
        zetas = this->damping_factors.segment(0,n_bou_mod_coords);
    if (this->damping_factors.size() < n_bou_mod_coords) {
        zetas.segment(0, this->damping_factors.size()) = this->damping_factors;
        for (int i = this->damping_factors.size(); i < n_bou_mod_coords; ++i) {
            zetas(i) = zetas(this->damping_factors.size() - 1); // repeat last of user provided factors
        }
    }

    modal_R.setZero(modal_M.rows(), modal_M.cols());
    
    ChMatrixDynamic<> V = modes_V_reduced.real();
    ChVectorDynamic<> Mmodal = (V.transpose() * modal_M * V).diagonal();

    //ChMatrixDynamic<> V_inv = (modes_V_reduced.real()).inverse(); // performance warning: use only for small n. of coords. Better do as transpose if V orthogonal?
    //modal_R = V_inv.transpose() * (2.0 * omegas.cwiseProduct(zetas)).asDiagonal() * V_inv;
    // Since modes_V_reduced is real, given symmetry of M and K, then V^-1 = V transpose,   rather do:
    ChMatrixDynamic<> modal_R_nonzero = V * (2.0 * Mmodal.cwiseProduct(omegas.cwiseProduct(zetas.segment(0,omegas.size())))).asDiagonal() * V.transpose();
    // Note that for test here we computed only first N modes, not all possible n_bou_mod_coords modes, so we restricted the operation to a sub block, leaving the rest as zero.
    modal_R.block(0, 0, modal_R_nonzero.rows(), modal_R_nonzero.cols()) = modal_R_nonzero;

    if (false) {
        ChStreamOutAsciiFile fileV("dump_modal_V.dat");
        fileV.SetNumFormat("%.12g");
        StreamOUTdenseMatlabFormat(V, fileV);
        ChStreamOutAsciiFile fileF("dump_modal_f.dat");
        fileF.SetNumFormat("%.12g");
        StreamOUTdenseMatlabFormat(freq_reduced, fileF);
        ChStreamOutAsciiFile fileRnz("dump_modal_Rnz.dat");
        fileRnz.SetNumFormat("%.12g");
        StreamOUTdenseMatlabFormat(modal_R_nonzero, fileRnz);
        ChStreamOutAsciiFile fileMm("dump_modal_Mm.dat");
        fileMm.SetNumFormat("%.12g");
        StreamOUTdenseMatlabFormat(Mmodal, fileMm);
    }
}








}  // end namespace modal

}  // end namespace chrono
