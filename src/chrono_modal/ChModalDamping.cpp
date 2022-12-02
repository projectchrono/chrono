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
        zetas = this->damping_factors.segment(0, n_mod_coords);
    if (this->damping_factors.size() < n_mod_coords) {
        zetas.segment(0, this->damping_factors.size()) = this->damping_factors;
        for (int i = this->damping_factors.size(); i < n_mod_coords; ++i) {
            zetas(i) = zetas(this->damping_factors.size() - 1);  // repeat last of user provided factors
        }
    }

    modal_R.setZero(modal_M.rows(), modal_M.cols());

    modal_R.block(n_bou_coords, n_bou_coords, n_mod_coords, n_mod_coords) =
        (2.0 * omegas.cwiseProduct(zetas)).asDiagonal();
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

    modal_R.block(0, 0, n_bou_coords, n_bou_coords) = alpha * modal_M.block(0, 0, n_bou_coords, n_bou_coords) +
                                                      beta * modal_K.block(0, 0, n_bou_coords, n_bou_coords);
}

void ChModalDampingFactorAssembly::ComputeR(ChModalAssembly& assembly,
                                            const ChMatrixDynamic<>& modal_M,
                                            const ChMatrixDynamic<>& modal_K,
                                            const ChMatrixDynamic<>& Psi,
                                            ChMatrixDynamic<>& modal_R) const {
    assert(false);  // this damping model is not ready and must be validated

    int n_mod_coords = assembly.Get_n_modes_coords_w();
    int n_bou_coords = assembly.GetN_boundary_coords_w();
    int n_bou_mod_coords = n_bou_coords + n_mod_coords;

    ChMatrixDynamic<std::complex<double>> modes_V_reduced(n_bou_mod_coords, n_bou_mod_coords);
    ChVectorDynamic<std::complex<double>> eig_reduced(n_bou_mod_coords);
    ChVectorDynamic<> freq_reduced(n_bou_mod_coords);
    ChSparseMatrix Cq_reduced;
    assembly.GetSubassemblyConstraintJacobianMatrix(&Cq_reduced);
    ChSparseMatrix M_reduced;
    assembly.GetSubassemblyMassMatrix(&M_reduced);
    ChSparseMatrix K_reduced;
    assembly.GetSubassemblyStiffnessMatrix(&K_reduced);

    /* old
    ChGeneralizedEigenvalueSolverLanczos     eigsolver;
    //ChGeneralizedEigenvalueSolverKrylovSchur eigsolver;
    eigsolver.sigma = 0.01;  //// TODO lower value of sigma working in Debug but not in Release !?!?!
    eigsolver.Solve(M_reduced, K_reduced, Cq_reduced, modes_V_reduced, eig_reduced, freq_reduced, 6); //***TODO*** must
    be all modes n_bou_mod_coords-Cq_reduced.rows(), not only first ones, but Krylov and Lanczos do not allow it..;
    */

    ChModalSolveUndamped eigsolver(
        6,      // n of lower modes (***TODO*** make parametric)
        0.01,   // lower freq, for shift&invert. (***TODO*** lower value of sigma working in Debug but not in Release!?)
        500,    // upper limit for the number of iterations, if iterative solver
        1e-10,  // tolerance for the iterative solver.
        false,  // turn to true to see some diagnostic.
        ChGeneralizedEigenvalueSolverLanczos()  // solver to use (default Lanczos)
    );
    eigsolver.Solve(M_reduced, K_reduced, Cq_reduced, modes_V_reduced, eig_reduced, freq_reduced);

    /*
    // The iterative solver above does not work well. Since the size of the M_reduced K_reduced is already small (as
    // this is a modal assembly that already went through modal reduction) we can just use a direct solver for finding
    eigs: ChQuadraticEigenvalueSolverNullspaceDirect eigsolver; ChSparseMatrix R_null; R_null.resize(M_reduced.rows(),
    M_reduced.rows()); R_null.setZero(); ChVectorDynamic<> damp_factors(M_reduced.rows());
    // Note that we might enforce symmetry of M_reduced and K_reduced via 0.5*(M+M.transpose()) bacause even small
    unsymmetry causes modes_V_reduced to have some imaginary part. eigsolver.Solve(M_reduced, R_null, K_reduced,
    Cq_reduced, modes_V_reduced, eig_reduced, freq_reduced, damp_factors, n_bou_mod_coords-Cq_reduced.rows());
    */
    ChVectorDynamic<> omegas = CH_C_2PI * freq_reduced;
    ChVectorDynamic<> zetas;
    zetas.setZero(n_bou_mod_coords);

    if (this->damping_factors.size() == n_bou_mod_coords)
        zetas = this->damping_factors;
    if (this->damping_factors.size() > n_bou_mod_coords)
        zetas = this->damping_factors.segment(0, n_bou_mod_coords);
    if (this->damping_factors.size() < n_bou_mod_coords) {
        zetas.segment(0, this->damping_factors.size()) = this->damping_factors;
        for (int i = this->damping_factors.size(); i < n_bou_mod_coords; ++i) {
            zetas(i) = zetas(this->damping_factors.size() - 1);  // repeat last of user provided factors
        }
    }

    modal_R.setZero(modal_M.rows(), modal_M.cols());

    //// NOTE: when using ChQuadraticEigenvalueSolverNullspaceDirect the V eigenvectors are normalized in the complex
    ///sense,
    // but real part of eigenvectors may be not (If using ChGeneralizedEigenvalueSolverLanczos no issue). So do this
    // hack:
    ChMatrixDynamic<> V = modes_V_reduced.real();
    for (int i = 0; i < V.cols(); ++i)
        V.col(i).normalize();
    ChMatrixDynamic<> Mmodal_matr = (V.transpose() * modal_M * V).real();
    ChVectorDynamic<> Mmodal = Mmodal_matr.diagonal();
    /*
    //// NOTE: maybe this?
    ChMatrixDynamic<> V = modes_V_reduced
    Mmodal_matr = (modes_V_reduced.conjugate().transpose() * modal_M * modes_V_reduced).real();
    Mmodal = Mmodal_matr.diagonal();
    */
    // ChMatrixDynamic<> V_inv = (modes_V_reduced.real()).inverse(); // performance warning: use only for small n. of
    // coords. Better do as transpose if V orthogonal? modal_R = V_inv.transpose() * (2.0 *
    // omegas.cwiseProduct(zetas)).asDiagonal() * V_inv;
    // Since modes_V_reduced is real, given symmetry of M and K, then V^-1 = V transpose,   rather do:
    ChMatrixDynamic<> modal_R_nonzero =
        V * (2.0 * Mmodal.cwiseProduct(omegas.cwiseProduct(zetas.segment(0, omegas.size())))).asDiagonal() *
        V.transpose();
    // Note that for test here we computed only first N modes, not all possible n_bou_mod_coords modes, so we restricted
    // the operation to a sub block, leaving the rest as zero.
    modal_R.block(0, 0, modal_R_nonzero.rows(), modal_R_nonzero.cols()) = modal_R_nonzero;

    if (true) {
        ChStreamOutAsciiFile fileM("dump_modald_M.dat");
        fileM.SetNumFormat("%.12g");
        StreamOUTdenseMatlabFormat(M_reduced.toDense(), fileM);
        ChStreamOutAsciiFile fileK("dump_modald_K.dat");
        fileK.SetNumFormat("%.12g");
        StreamOUTdenseMatlabFormat(K_reduced.toDense(), fileK);
        ChStreamOutAsciiFile fileCq("dump_modald_Cq.dat");
        fileCq.SetNumFormat("%.12g");
        StreamOUTdenseMatlabFormat(Cq_reduced.toDense(), fileCq);
        ChStreamOutAsciiFile fileV("dump_modald_V.dat");
        fileV.SetNumFormat("%.12g");
        StreamOUTdenseMatlabFormat(V, fileV);
        ChStreamOutAsciiFile fileF("dump_modald_f.dat");
        fileF.SetNumFormat("%.12g");
        StreamOUTdenseMatlabFormat(freq_reduced, fileF);
        ChStreamOutAsciiFile fileRnz("dump_modald_Rnz.dat");
        fileRnz.SetNumFormat("%.12g");
        StreamOUTdenseMatlabFormat(modal_R_nonzero, fileRnz);
        ChStreamOutAsciiFile fileMm("dump_modald_Mm.dat");
        fileMm.SetNumFormat("%.12g");
        StreamOUTdenseMatlabFormat(Mmodal, fileMm);
        ChStreamOutAsciiFile fileMm_matr("dump_modald_Mm_matr.dat");
        fileMm_matr.SetNumFormat("%.12g");
        StreamOUTdenseMatlabFormat(Mmodal_matr, fileMm_matr);
    }
}

}  // end namespace modal

}  // end namespace chrono
