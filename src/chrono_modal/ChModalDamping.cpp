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

#include <iomanip>

#include "chrono_modal/ChModalDamping.h"
#include "chrono_modal/ChModalAssembly.h"
#include "chrono_modal/ChGeneralizedEigenvalueSolver.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/fea/ChNodeFEAxyz.h"
#include "chrono/fea/ChNodeFEAxyzrot.h"

namespace chrono {

using namespace fea;

namespace modal {

ChModalDampingReductionR::ChModalDampingReductionR(ChModalAssembly& massembly) {
    massembly.GetSubassemblyMatrices(nullptr, &full_R, nullptr, nullptr);
}

void ChModalDampingFactorRmm::ComputeR(ChModalAssembly& assembly,
                                       const ChMatrixDynamic<>& modal_M,
                                       const ChMatrixDynamic<>& modal_K,
                                       const ChMatrixDynamic<>& Psi,
                                       ChMatrixDynamic<>& modal_R) const {
    unsigned int n_mod_coords = assembly.GetNumCoordinatesModal();
    unsigned int n_bou_coords = assembly.GetNumCoordinatesVelBoundary();

    ChVectorDynamic<> omegas = CH_2PI * assembly.GetUndampedFrequencies();
    ChVectorDynamic<> zetas;
    zetas.setZero(n_mod_coords);

    if (this->damping_factors.size() == n_mod_coords)
        zetas = this->damping_factors;
    if (this->damping_factors.size() > n_mod_coords)
        zetas = this->damping_factors.segment(0, n_mod_coords);
    if (this->damping_factors.size() < n_mod_coords) {
        zetas.segment(0, this->damping_factors.size()) = this->damping_factors;
        for (unsigned int i = this->damping_factors.size(); i < n_mod_coords; ++i) {
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
    int n_bou_coords = assembly.GetNumCoordinatesVelBoundary();

    modal_R.block(0, 0, n_bou_coords, n_bou_coords) = alpha * modal_M.block(0, 0, n_bou_coords, n_bou_coords) +
                                                      beta * modal_K.block(0, 0, n_bou_coords, n_bou_coords);
}

void ChModalDampingFactorAssembly::ComputeR(ChModalAssembly& assembly,
                                            const ChMatrixDynamic<>& modal_M,
                                            const ChMatrixDynamic<>& modal_K,
                                            const ChMatrixDynamic<>& Psi,
                                            ChMatrixDynamic<>& modal_R) const {
    // assert(false);  // this damping model is not ready and must be validated

    int n_mod_coords = assembly.GetNumCoordinatesModal();
    int n_bou_coords = assembly.GetNumCoordinatesVelBoundary();
    int n_bou_mod_coords = n_bou_coords + n_mod_coords;
    int n_constrs = assembly.GetNumConstraints();
    int n_eff_modes = n_bou_mod_coords - n_constrs;

    ChMatrixDynamic<std::complex<double>> modes_V_reduced(n_bou_mod_coords, n_eff_modes);
    ChVectorDynamic<std::complex<double>> eig_reduced(n_eff_modes);
    ChVectorDynamic<> freq_reduced(n_eff_modes);
    ChSparseMatrix K_reduced, M_reduced, Cq_reduced;

    assembly.GetSubassemblyMatrices(&K_reduced, nullptr, &M_reduced, &Cq_reduced);

    if (false) {
        std::ofstream fileM("dump_modalreduced_M.dat");
        fileM << std::setprecision(12) << std::scientific;
        StreamOut(M_reduced, fileM);
        std::ofstream fileK("dump_modalreduced_K.dat");
        fileK << std::setprecision(12) << std::scientific;
        StreamOut(K_reduced, fileK);
        std::ofstream fileCq("dump_modalreduced_Cq.dat");
        fileCq << std::setprecision(12) << std::scientific;
        StreamOut(Cq_reduced, fileCq);
    }

    /* old
    ChGeneralizedEigenvalueSolverLanczos     eigsolver;
    //ChGeneralizedEigenvalueSolverKrylovSchur eigsolver;
    eigsolver.sigma = 0.01;  //// TODO lower value of sigma working in Debug but not in Release !?!?!
    eigsolver.Solve(M_reduced, K_reduced, Cq_reduced, modes_V_reduced, eig_reduced, freq_reduced, 6); //// TODO  must
    be all modes n_bou_mod_coords-Cq_reduced.rows(), not only first ones, but Krylov and Lanczos do not allow it..;
    */

    // ChModalSolverUndamped eigsolver(
    //     6,      // n of lower modes (***TODO*** make parametric)
    //     0.01,   // lower freq, for shift&invert. (***TODO*** lower value of sigma working in Debug but not in
    //     Release!?) 500,    // upper limit for the number of iterations, if iterative solver 1e-10,  // tolerance for
    //     the iterative solver. false,  // turn to true to see some diagnostic. ChGeneralizedEigenvalueSolverLanczos()
    //     // solver to use (default Lanczos)
    //);

    /*
    // The iterative solver above does not work well. Since the size of the M_reduced K_reduced is already small (as
    // this is a modal assembly that already went through modal reduction) we can just use a direct solver for finding
    eigs: ChUnsymGenEigenvalueSolverNullspaceDirect eigsolver; ChSparseMatrix R_null; R_null.resize(M_reduced.rows(),
    M_reduced.rows()); R_null.setZero(); ChVectorDynamic<> damp_factors(M_reduced.rows());
    // Note that we might enforce symmetry of M_reduced and K_reduced via 0.5*(M+M.transpose()) bacause even small
    unsymmetry causes modes_V_reduced to have some imaginary part. eigsolver.Solve(M_reduced, R_null, K_reduced,
    Cq_reduced, modes_V_reduced, eig_reduced, freq_reduced, damp_factors, n_bou_mod_coords-Cq_reduced.rows());
*/

    // TODO: implement an eigensolver using Eigen::Dense matrix directly, without Nullspace transformation,
    // because the null space transforamtion is not safe in general.

    ChUnsymGenEigenvalueSolverKrylovSchur eigsolver;
    ChSparseMatrix R_null;
    R_null.resize(M_reduced.rows(), M_reduced.rows());
    R_null.setZero();
    ChVectorDynamic<> damp_factors(M_reduced.rows());
    ChSparseMatrix A, B;
    eigsolver.BuildDampedSystem(M_reduced, R_null, K_reduced, Cq_reduced, A, B, true);
    eigsolver.Solve(A, B, modes_V_reduced, eig_reduced, 6, 1e-6);

    ChUnsymGenEigenvalueSolver::GetNaturalFrequencies(eig_reduced, freq_reduced);
    ChUnsymGenEigenvalueSolver::GetDampingRatios(eig_reduced, damp_factors);

    ChVectorDynamic<> zetas;
    ChVectorDynamic<> omegas = CH_2PI * freq_reduced;
    zetas.setZero(n_eff_modes);

    if (this->damping_factors.size() == n_eff_modes)
        zetas = this->damping_factors;
    if (this->damping_factors.size() > n_eff_modes)
        zetas = this->damping_factors.segment(0, n_eff_modes);
    if (this->damping_factors.size() < n_eff_modes) {
        zetas.segment(0, this->damping_factors.size()) = this->damping_factors;
        for (int i = this->damping_factors.size(); i < n_eff_modes; ++i) {
            zetas(i) = zetas(this->damping_factors.size() - 1);  // repeat last of user provided factors
        }
    }

    modal_R.setZero(modal_M.rows(), modal_M.cols());

    //// NOTE: when using ChUnsymGenEigenvalueSolverNullspaceDirect the V eigenvectors are normalized in the complex
    /// sense,
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

    if (false) {
        std::ofstream fileV("dump_modalxxx_V.dat");
        fileV << std::setprecision(12) << std::scientific;
        StreamOut(V, fileV);
        std::ofstream fileF("dump_modalxxx_f.dat");
        fileF << std::setprecision(12) << std::scientific;
        StreamOut(freq_reduced, fileF);
        std::ofstream fileRnz("dump_modalxxx_Rnz.dat");
        fileRnz << std::setprecision(12) << std::scientific;
        StreamOut(modal_R_nonzero, fileRnz);
        std::ofstream fileMm("dump_modalxxx_Mm.dat");
        fileMm << std::setprecision(12) << std::scientific;
        StreamOut(Mmodal, fileMm);
        std::ofstream fileMm_matr("dump_modalxxx_Mm_matr.dat");
        fileMm_matr << std::setprecision(12) << std::scientific;
        StreamOut(Mmodal_matr, fileMm_matr);
    }
}

}  // end namespace modal

}  // end namespace chrono