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
// Author: Dario Mangoni
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/fea/ChElementBeamEuler.h"
#include "chrono/fea/ChBuilderBeam.h"
#include "chrono/timestepper/ChAssemblyAnalysis.h"

#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/utils/ChUtilsValidation.h"

#include "chrono/fea/ChMesh.h"

#include "chrono_modal/ChUnsymGenEigenvalueSolver.h"
#include "chrono_modal/ChSymGenEigenvalueSolver.h"
#include "chrono_modal/ChGeneralizedEigenvalueSolver.h"
#include "chrono_modal/ChModalSolverUndamped.h"
#include "chrono_modal/ChModalSolverDamped.h"

#include "chrono/solver/ChDirectSolverLScomplex.h"

#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_thirdparty/fast_matrix_market/app/Eigen.hpp"

#include <iomanip>

#include "gtest/gtest.h"

using namespace chrono;
using namespace chrono::modal;
using namespace chrono::fea;

static const std::string val_dir = "../RESULTS/";
static const std::string out_dir = val_dir + "modal/";
static const std::string ref_dir = "testing/modal/eigensolver/";

static const double tolerance = 1e-3;

double GetEigenvaluesMaxDiff(const ChVectorDynamic<double>& eig1, const ChVectorDynamic<double>& eig2) {
    return (eig1 - eig2).lpNorm<Eigen::Infinity>();
}

double GetEigenvaluesMaxDiff(const ChVectorDynamic<std::complex<double>>& eig1,
                             const ChVectorDynamic<std::complex<double>>& eig2) {
    return std::max((eig1.real() - eig2.real()).lpNorm<Eigen::Infinity>(),
                    (eig1.imag().cwiseAbs() - eig2.imag().cwiseAbs()).lpNorm<Eigen::Infinity>());
}

void prepare_folders(std::string testname) {
    // Create output directory (if it does not already exist)
    if (!filesystem::create_directory(filesystem::path(val_dir))) {
        std::cerr << "Error creating directory " << val_dir << std::endl;
        throw std::invalid_argument("Error creating directory " + val_dir);
    }
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cerr << "Error creating directory " << out_dir << std::endl;
        throw std::invalid_argument("Error creating directory " + out_dir);
    }
    if (!filesystem::create_directory(filesystem::path(out_dir + testname))) {
        std::cerr << "Error creating directory " << out_dir + testname << std::endl;
        throw std::invalid_argument("Error creating directory " + out_dir + testname);
    }
}

std::shared_ptr<ChAssembly> BuildBeamFixBody(ChSystem& sys) {
    /*
     * Beam with end body
     *
     *   (fixed node)----()----()----()----()<--link-->[body]
     *
     */

    sys.Clear();

    auto assembly = chrono_types::make_shared<ChAssembly>();

    sys.Add(assembly);

    double beam_Young = 100.e6;
    double beam_density = 1000;
    double beam_wz = 0.3;
    double beam_wy = 0.05;
    double beam_L = 6;

    double body_end_xwidth = 0.5;

    // beam
    auto mesh = chrono_types::make_shared<ChMesh>();
    assembly->Add(mesh);

    mesh->SetAutomaticGravity(false);

    auto section = chrono_types::make_shared<ChBeamSectionEulerAdvanced>();

    section->SetDensity(beam_density);
    section->SetYoungModulus(beam_Young);
    section->SetShearModulusFromPoisson(0.31);
    section->SetRayleighDampingBeta(0.00001);
    section->SetRayleighDampingAlpha(0.001);
    section->SetAsRectangularSection(beam_wy, beam_wz);

    ChBuilderBeamEuler builder;

    builder.BuildBeam(mesh,                      // the mesh where to put the created nodes and elements
                      section,                   // the ChBeamSectionEuler to use for the ChElementBeamEuler elements
                      4,                         // the number of ChElementBeamEuler to create
                      ChVector3d(0, 0, 0),       // the 'A' point in space (beginning of beam)
                      ChVector3d(beam_L, 0, 0),  // the 'B' point in space (end of beam)
                      ChVector3d(0, 1, 0)        // the 'Y' up direction of the section for the beam
    );

    builder.GetLastBeamNodes().front()->SetFixed(true);

    auto body_end = chrono_types::make_shared<ChBodyEasyBox>(body_end_xwidth, 1, 1, 200);
    body_end->SetPos(ChVector3d(beam_L + body_end_xwidth / 2.0, 0, 0));
    assembly->Add(body_end);

    auto link_beamend_body = chrono_types::make_shared<ChLinkMateFix>();
    link_beamend_body->Initialize(builder.GetLastBeamNodes().back(), body_end,
                                  ChFrame<>(ChVector3d(beam_L, 0, 0), QUNIT));
    assembly->Add(link_beamend_body);

    sys.Setup();
    sys.Update(false);

    return assembly;
}

void generateKRMCqFromAssembly(std::shared_ptr<ChAssembly> assembly,
                               ChSparseMatrix& K,
                               ChSparseMatrix& R,
                               ChSparseMatrix& M,
                               ChSparseMatrix& Cq) {
    ChSystemNSC sys;
    // auto assembly = BuildBeamFixBody(sys);

    assembly->Setup();
    assembly->Update(0.0, false);

    ChSystemDescriptor temp_descriptor;

    temp_descriptor.BeginInsertion();
    assembly->InjectVariables(temp_descriptor);
    assembly->InjectKRMMatrices(temp_descriptor);
    assembly->InjectConstraints(temp_descriptor);
    temp_descriptor.EndInsertion();

    // Generate the A and B in state space
    int n_vars = temp_descriptor.CountActiveVariables();
    int n_constr = temp_descriptor.CountActiveConstraints();
    M.resize(n_vars, n_vars);
    K.resize(n_vars, n_vars);
    R.resize(n_vars, n_vars);
    Cq.resize(n_constr, n_vars);

    // Stiffness matrix
    assembly->LoadKRMMatrices(1.0, 0.0, 0.0);
    temp_descriptor.SetMassFactor(0.0);
    temp_descriptor.PasteMassKRMMatrixInto(K, 0, 0);

    // Damping matrix
    assembly->LoadKRMMatrices(0.0, 1.0, 0.0);
    temp_descriptor.SetMassFactor(0.0);
    temp_descriptor.PasteMassKRMMatrixInto(R, 0, 0);

    // Mass matrix
    assembly->LoadKRMMatrices(0.0, 0.0, 1.0);
    temp_descriptor.SetMassFactor(1.0);
    temp_descriptor.PasteMassKRMMatrixInto(M, 0, 0);

    // Constraint Jacobian
    assembly->LoadConstraintJacobians();
    temp_descriptor.PasteConstraintsJacobianMatrixInto(Cq, 0, 0);
}

void dumpKRMMatricesFromAssembly(std::shared_ptr<ChAssembly> assembly, std::string refname) {
    ChSparseMatrix K, R, M, Cq;

    generateKRMCqFromAssembly(assembly, K, R, M, Cq);

    std::ofstream stream_K(utils::GetValidationDataFile(ref_dir + refname + "/" + refname + "_K.txt"));
    std::ofstream stream_R(utils::GetValidationDataFile(ref_dir + refname + "/" + refname + "_R.txt"));
    std::ofstream stream_M(utils::GetValidationDataFile(ref_dir + refname + "/" + refname + "_M.txt"));
    std::ofstream stream_Cq(utils::GetValidationDataFile(ref_dir + refname + "/" + refname + "_Cq.txt"));

    if (stream_M.fail() || stream_K.fail() || stream_R.fail() || stream_Cq.fail()) {
        std::cerr << "Error opening file for writing in " << ref_dir + refname << " folder" << std::endl;
        return;
    }

    fast_matrix_market::write_matrix_market_eigen(stream_M, M);
    fast_matrix_market::write_matrix_market_eigen(stream_K, K);
    fast_matrix_market::write_matrix_market_eigen(stream_R, R);
    fast_matrix_market::write_matrix_market_eigen(stream_Cq, Cq);
}

// int main() {
//     std::string testname = "SymKMCqChrono";
//
//     // Create a system
//     ChSystemNSC sys;
//     auto assembly = BuildBeamFixBody(sys);
//     generateKRMCqFromAssembly(assembly, testname);
//
//     return 0;
// }

template <typename EigenSolverType, typename ScalarType>
void ExecuteEigenSolverCallAB(EigenSolverType eigen_solver, std::string refname) {
    ChSparseMatrix A;
    ChSparseMatrix B;
    ChMatrixDynamic<ScalarType> sigma_mat;
    ChMatrixDynamic<int> reqeigs_mat;
    ChMatrixDynamic<ScalarType> eigvects_MATLAB;
    ChVectorDynamic<ScalarType> eigvals_MATLAB;

    ChMatrixDynamic<ScalarType> eigvects_CHRONO;
    ChVectorDynamic<ScalarType> eigvals_CHRONO;

    std::ifstream stream_A(utils::GetValidationDataFile(ref_dir + refname + "/" + refname + "_A.txt"));
    std::ifstream stream_B(utils::GetValidationDataFile(ref_dir + refname + "/" + refname + "_B.txt"));
    std::ifstream stream_sigma(utils::GetValidationDataFile(ref_dir + refname + "/" + refname + "_sigma.txt"));
    std::ifstream stream_reqeigs(utils::GetValidationDataFile(ref_dir + refname + "/" + refname + "_reqeigs.txt"));
    std::ifstream stream_eigvals_MATLAB(
        utils::GetValidationDataFile(ref_dir + refname + "/" + refname + "_eigvals_MATLAB.txt"));
    std::ifstream stream_eigvects_MATLAB(
        utils::GetValidationDataFile(ref_dir + refname + "/" + refname + "_eigvects_MATLAB.txt"));

    fast_matrix_market::read_matrix_market_eigen(stream_A, A);
    fast_matrix_market::read_matrix_market_eigen(stream_B, B);
    fast_matrix_market::read_matrix_market_eigen_dense(stream_sigma, sigma_mat);
    ScalarType sigma = sigma_mat(0, 0);
    fast_matrix_market::read_matrix_market_eigen_dense(stream_reqeigs, reqeigs_mat);
    int reqeigs = reqeigs_mat(0, 0);
    fast_matrix_market::read_matrix_market_eigen_dense(stream_eigvals_MATLAB, eigvals_MATLAB);
    fast_matrix_market::read_matrix_market_eigen_dense(stream_eigvects_MATLAB, eigvects_MATLAB);

    eigen_solver.Solve(A, B, eigvects_CHRONO, eigvals_CHRONO, reqeigs, sigma);

    eigen_solver.SortRitzPairs(eigvals_MATLAB, eigvects_MATLAB);

    // std::ofstream stream_eigvals_CHRONO(
    //     utils::GetValidationDataFile(ref_dir + refname + "/" + refname + "_eigvals_CHRONO.txt"));
    // std::ofstream stream_eigvects_CHRONO(
    //     utils::GetValidationDataFile(ref_dir + refname + "/" + refname + "_eigvects_CHRONO.txt"));
    // fast_matrix_market::write_matrix_market_eigen_dense(stream_eigvals_CHRONO, eigvals_CHRONO);
    // fast_matrix_market::write_matrix_market_eigen_dense(stream_eigvects_CHRONO, eigvects_CHRONO);

    double max_delta_eigvals = GetEigenvaluesMaxDiff(eigvals_CHRONO, eigvals_MATLAB);
    ASSERT_NEAR(max_delta_eigvals, 0, tolerance) << "Eigenvalues not matching.\n"
                                                 << "MATLAB:\n"
                                                 << eigvals_MATLAB << "\nCHRONO:\n"
                                                 << eigvals_CHRONO << std::endl;

    double max_residual_CHRONO = eigen_solver.GetMaxResidual(A, B, eigvects_CHRONO, eigvals_CHRONO);

    ASSERT_NEAR(max_residual_CHRONO, 0, tolerance) << "Residuals exceeding threshold" << std::endl;
}

void ExecuteEigenSolverCallKMCq(ChSymGenEigenvalueSolver& eigen_solver, std::string refname) {
    ChSparseMatrix M;
    ChSparseMatrix K;
    ChSparseMatrix Cq;
    ChMatrixDynamic<double> sigma_mat;
    ChMatrixDynamic<int> reqeigs_mat;
    ChMatrixDynamic<double> eigvects_MATLAB;
    ChVectorDynamic<double> eigvals_MATLAB;

    ChMatrixDynamic<double> eigvects_CHRONO;
    ChVectorDynamic<double> eigvals_CHRONO;

    std::ifstream stream_M(utils::GetValidationDataFile(ref_dir + refname + "/" + refname + "_M.txt"));
    std::ifstream stream_K(utils::GetValidationDataFile(ref_dir + refname + "/" + refname + "_K.txt"));
    std::ifstream stream_Cq(utils::GetValidationDataFile(ref_dir + refname + "/" + refname + "_Cq.txt"));
    std::ifstream stream_sigma(utils::GetValidationDataFile(ref_dir + refname + "/" + refname + "_sigma.txt"));
    std::ifstream stream_reqeigs(utils::GetValidationDataFile(ref_dir + refname + "/" + refname + "_reqeigs.txt"));
    std::ifstream stream_eigvals_MATLAB(
        utils::GetValidationDataFile(ref_dir + refname + "/" + refname + "_eigvals_MATLAB.txt"));
    std::ifstream stream_eigvects_MATLAB(
        utils::GetValidationDataFile(ref_dir + refname + "/" + refname + "_eigvects_MATLAB.txt"));

    fast_matrix_market::read_matrix_market_eigen(stream_M, M);
    fast_matrix_market::read_matrix_market_eigen(stream_K, K);
    fast_matrix_market::read_matrix_market_eigen(stream_Cq, Cq);
    fast_matrix_market::read_matrix_market_eigen_dense(stream_sigma, sigma_mat);
    double sigma = sigma_mat(0, 0);
    fast_matrix_market::read_matrix_market_eigen_dense(stream_reqeigs, reqeigs_mat);
    int reqeigs = reqeigs_mat(0, 0);
    fast_matrix_market::read_matrix_market_eigen_dense(stream_eigvals_MATLAB, eigvals_MATLAB);
    fast_matrix_market::read_matrix_market_eigen_dense(stream_eigvects_MATLAB, eigvects_MATLAB);

    eigen_solver.SortRitzPairs(eigvals_MATLAB, eigvects_MATLAB);

    // ChSymGenEigenvalueSolverKrylovSchur eigen_solver;
    const bool scaleCq = true;

    ChSparseMatrix A, B;
    double scaling = ChGeneralizedEigenvalueSolver<double>::BuildUndampedSystem(M, K, Cq, A, B, scaleCq);

    // eigen_solver.sort_ritz_pairs = false;
    eigen_solver.Solve(A, B, eigvects_CHRONO, eigvals_CHRONO, reqeigs, sigma);

    double max_residual_CHRONO_AB = eigen_solver.GetMaxResidual(A, B, eigvects_CHRONO, eigvals_CHRONO);

    eigvects_CHRONO.bottomRows(Cq.rows()) *= scaling;

    // instead of doing a simple difference, consider the imaginary part to be the same if positive or negative
    double max_delta_eigvals = GetEigenvaluesMaxDiff(eigvals_CHRONO, eigvals_MATLAB);

    ASSERT_NEAR(max_delta_eigvals, 0, tolerance) << "Eigenvalues not matching.\n"
                                                 << "MATLAB:\n"
                                                 << eigvals_MATLAB << "\nCHRONO:\n"
                                                 << eigvals_CHRONO << std::endl;

    double max_residual_CHRONO = eigen_solver.GetMaxResidual(K, M, Cq, eigvects_CHRONO, eigvals_CHRONO);

    ASSERT_NEAR(max_residual_CHRONO, max_residual_CHRONO_AB, tolerance)
        << "Test developer error: residuals calculated on A,B are different from those calculated on K,M,Cq"
        << std::endl;

    ASSERT_NEAR(max_residual_CHRONO, 0, tolerance) << "Residuals exceeding threshold" << std::endl;
}

void ExecuteEigenSolverCallUnsymKMCq(std::string refname) {
    ChSparseMatrix M;
    ChSparseMatrix K;
    ChSparseMatrix Cq;
    ChMatrixDynamic<std::complex<double>> sigma_mat;
    ChMatrixDynamic<int> reqeigs_mat;
    ChMatrixDynamic<std::complex<double>> eigvects_MATLAB;
    ChVectorDynamic<std::complex<double>> eigvals_MATLAB;

    ChMatrixDynamic<std::complex<double>> eigvects_CHRONO;
    ChVectorDynamic<std::complex<double>> eigvals_CHRONO;

    std::ifstream stream_M(utils::GetValidationDataFile(ref_dir + refname + "/" + refname + "_M.txt"));
    std::ifstream stream_K(utils::GetValidationDataFile(ref_dir + refname + "/" + refname + "_K.txt"));
    std::ifstream stream_Cq(utils::GetValidationDataFile(ref_dir + refname + "/" + refname + "_Cq.txt"));
    std::ifstream stream_sigma(utils::GetValidationDataFile(ref_dir + refname + "/" + refname + "_sigma.txt"));
    std::ifstream stream_reqeigs(utils::GetValidationDataFile(ref_dir + refname + "/" + refname + "_reqeigs.txt"));
    std::ifstream stream_eigvals_MATLAB(
        utils::GetValidationDataFile(ref_dir + refname + "/" + refname + "_eigvals_MATLAB.txt"));
    std::ifstream stream_eigvects_MATLAB(
        utils::GetValidationDataFile(ref_dir + refname + "/" + refname + "_eigvects_MATLAB.txt"));

    fast_matrix_market::read_matrix_market_eigen(stream_M, M);
    fast_matrix_market::read_matrix_market_eigen(stream_K, K);
    fast_matrix_market::read_matrix_market_eigen(stream_Cq, Cq);
    fast_matrix_market::read_matrix_market_eigen_dense(stream_sigma, sigma_mat);
    std::complex<double> sigma = sigma_mat(0, 0);
    fast_matrix_market::read_matrix_market_eigen_dense(stream_reqeigs, reqeigs_mat);
    int reqeigs = reqeigs_mat(0, 0);
    fast_matrix_market::read_matrix_market_eigen_dense(stream_eigvals_MATLAB, eigvals_MATLAB);
    fast_matrix_market::read_matrix_market_eigen_dense(stream_eigvects_MATLAB, eigvects_MATLAB);

    ChUnsymGenEigenvalueSolverKrylovSchur eigen_solver(chrono_types::make_shared<ChSolverSparseComplexLU>());

    eigen_solver.SortRitzPairs(eigvals_MATLAB, eigvects_MATLAB);

    // ChSymGenEigenvalueSolverKrylovSchur eigen_solver;
    const bool scaleCq = true;

    ChSparseMatrix A, B;
    double scaling = ChGeneralizedEigenvalueSolver<std::complex<double>>::BuildUndampedSystem(M, K, Cq, A, B, scaleCq);

    // eigen_solver.sort_ritz_pairs = false;
    eigen_solver.Solve(A, B, eigvects_CHRONO, eigvals_CHRONO, reqeigs, sigma);
    double max_residual_CHRONO_AB = eigen_solver.GetMaxResidual(A, B, eigvects_CHRONO, eigvals_CHRONO);

    eigvects_CHRONO.bottomRows(Cq.rows()) *= scaling;

    // instead of doing a simple difference, consider the imaginary part to be the same if positive or negative
    double max_delta_eigvals = GetEigenvaluesMaxDiff(eigvals_CHRONO, eigvals_MATLAB);

    ASSERT_NEAR(max_delta_eigvals, 0, tolerance) << "Eigenvalues not matching.\n"
                                                 << "MATLAB:\n"
                                                 << eigvals_MATLAB << "\nCHRONO:\n"
                                                 << eigvals_CHRONO << std::endl;

    double max_residual_CHRONO = eigen_solver.GetMaxResidual(K, M, Cq, eigvects_CHRONO, eigvals_CHRONO);

    ASSERT_NEAR(max_residual_CHRONO, max_residual_CHRONO_AB, tolerance)
        << "Test developer error: residuals calculated on A,B are different from those calculated on K,M,Cq"
        << std::endl;

    ASSERT_NEAR(max_residual_CHRONO, 0, tolerance) << "Residuals exceeding threshold" << std::endl;
}

void ExecuteEigenSolverCallKRMCq(std::string refname) {
    ChSparseMatrix M;
    ChSparseMatrix K;
    ChSparseMatrix R;
    ChSparseMatrix Cq;
    ChMatrixDynamic<std::complex<double>> sigma_mat;
    ChMatrixDynamic<int> reqeigs_mat;
    ChMatrixDynamic<std::complex<double>> eigvects_MATLAB;
    ChVectorDynamic<std::complex<double>> eigvals_MATLAB;

    ChMatrixDynamic<std::complex<double>> eigvects_CHRONO;
    ChVectorDynamic<std::complex<double>> eigvals_CHRONO;

    std::ifstream stream_M(utils::GetValidationDataFile(ref_dir + refname + "/" + refname + "_M.txt"));
    std::ifstream stream_R(utils::GetValidationDataFile(ref_dir + refname + "/" + refname + "_R.txt"));
    std::ifstream stream_K(utils::GetValidationDataFile(ref_dir + refname + "/" + refname + "_K.txt"));
    std::ifstream stream_Cq(utils::GetValidationDataFile(ref_dir + refname + "/" + refname + "_Cq.txt"));
    std::ifstream stream_sigma(utils::GetValidationDataFile(ref_dir + refname + "/" + refname + "_sigma.txt"));
    std::ifstream stream_reqeigs(utils::GetValidationDataFile(ref_dir + refname + "/" + refname + "_reqeigs.txt"));
    std::ifstream stream_eigvals_MATLAB(
        utils::GetValidationDataFile(ref_dir + refname + "/" + refname + "_eigvals_MATLAB.txt"));
    std::ifstream stream_eigvects_MATLAB(
        utils::GetValidationDataFile(ref_dir + refname + "/" + refname + "_eigvects_MATLAB.txt"));

    fast_matrix_market::read_matrix_market_eigen(stream_M, M);
    fast_matrix_market::read_matrix_market_eigen(stream_K, K);
    fast_matrix_market::read_matrix_market_eigen(stream_R, R);
    fast_matrix_market::read_matrix_market_eigen(stream_Cq, Cq);
    fast_matrix_market::read_matrix_market_eigen_dense(stream_sigma, sigma_mat);
    std::complex<double> sigma = sigma_mat(0, 0);
    fast_matrix_market::read_matrix_market_eigen_dense(stream_reqeigs, reqeigs_mat);
    int reqeigs = reqeigs_mat(0, 0);
    fast_matrix_market::read_matrix_market_eigen_dense(stream_eigvals_MATLAB, eigvals_MATLAB);
    fast_matrix_market::read_matrix_market_eigen_dense(stream_eigvects_MATLAB, eigvects_MATLAB);

    ChUnsymGenEigenvalueSolverKrylovSchur eigen_solver(chrono_types::make_shared<ChSolverSparseComplexLU>());
    eigen_solver.SortRitzPairs(eigvals_MATLAB, eigvects_MATLAB);

    const bool scaleCq = true;
    ChSparseMatrix A, B;
    double scaling = eigen_solver.BuildDampedSystem(M, R, K, Cq, A, B, scaleCq);

    eigen_solver.Solve(A, B, eigvects_CHRONO, eigvals_CHRONO, reqeigs, sigma);

    double max_residual_CHRONO_AB = eigen_solver.GetMaxResidual(A, B, eigvects_CHRONO, eigvals_CHRONO);

    eigvects_CHRONO.bottomRows(Cq.rows()) *= scaling;

    // instead of doing a simple difference, consider the imaginary part to be the same if positive or negative
    double max_delta_eigvals = GetEigenvaluesMaxDiff(eigvals_CHRONO, eigvals_MATLAB);

    ASSERT_NEAR(max_delta_eigvals, 0, tolerance) << "Eigenvalues not matching.\n"
                                                 << "MATLAB:\n"
                                                 << eigvals_MATLAB << "\nCHRONO:\n"
                                                 << eigvals_CHRONO << std::endl;
    double max_residual_CHRONO = eigen_solver.GetMaxResidual(K, R, M, Cq, eigvects_CHRONO, eigvals_CHRONO);

    ASSERT_NEAR(max_residual_CHRONO, max_residual_CHRONO_AB, tolerance)
        << "Test developer error: residuals calculated on A,B are different from those calculated on K,R,M,Cq"
        << std::endl;

    ASSERT_NEAR(max_residual_CHRONO, 0, tolerance) << "Residuals exceeding threshold" << std::endl;
}

TEST(CountNonZerosForEachRow, Count) {
    std::string refname = "CountNonZeros";

    ChSparseMatrix Q;
    Eigen::VectorXi Q_nnz_rows_MATLAB;
    Eigen::VectorXi Q_nnz_cols_MATLAB;

    std::ifstream stream_Q(utils::GetValidationDataFile(ref_dir + refname + "/" + refname + "_Q.txt"));
    std::ifstream stream_Q_nnz_rows(
        utils::GetValidationDataFile(ref_dir + refname + "/" + refname + "_Q_nnz_rows.txt"));
    std::ifstream stream_Q_nnz_cols(
        utils::GetValidationDataFile(ref_dir + refname + "/" + refname + "_Q_nnz_cols.txt"));
    fast_matrix_market::read_matrix_market_eigen(stream_Q, Q);
    fast_matrix_market::read_matrix_market_eigen_dense(stream_Q_nnz_rows, Q_nnz_rows_MATLAB);
    fast_matrix_market::read_matrix_market_eigen_dense(stream_Q_nnz_cols, Q_nnz_cols_MATLAB);

    Eigen::VectorXi Q_nnz_rows_CHRONO(Q.rows());
    Q_nnz_rows_CHRONO.setZero();
    Eigen::VectorXi Q_nnz_cols_CHRONO(Q.cols());
    Q_nnz_cols_CHRONO.setZero();

    CountNonZerosForEachRow(Q, Q_nnz_rows_CHRONO, 0);
    CountNonZerosForEachRowTransposed(Q, Q_nnz_cols_CHRONO, 0);

    ASSERT_EQ(Q_nnz_rows_CHRONO, Q_nnz_rows_MATLAB);
    ASSERT_EQ(Q_nnz_cols_CHRONO, Q_nnz_cols_MATLAB);
}

TEST(ChSymGenEigenvalueSolverKrylovSchur, SymAB) {
    ExecuteEigenSolverCallAB<ChSymGenEigenvalueSolverKrylovSchur, double>(ChSymGenEigenvalueSolverKrylovSchur(),
                                                                          "SymAB");
}

TEST(ChSymGenEigenvalueSolverLanczos, SymAB) {
    ExecuteEigenSolverCallAB<ChSymGenEigenvalueSolverLanczos, double>(ChSymGenEigenvalueSolverLanczos(), "SymAB");
}

TEST(ChSymGenEigenvalueSolverKrylovSchur, SymKMCq) {
    ChSymGenEigenvalueSolverKrylovSchur eigen_solver;
    ExecuteEigenSolverCallKMCq(eigen_solver, "SymKMCq");
}

// TEST(ChSymGenEigenvalueSolverLanczos, SymKMCq) {
//     // WARNING: this test is not passing; eigvalues are matching, but eigenvectors are not
//     ChSymGenEigenvalueSolverLanczos eigen_solver;
//     ExecuteEigenSolverCallKMCq(eigen_solver, "SymKMCq");
// }

TEST(ChSymGenEigenvalueSolverKrylovSchur, SymKMCqChrono_AB) {
    ExecuteEigenSolverCallAB<ChSymGenEigenvalueSolverKrylovSchur, double>(ChSymGenEigenvalueSolverKrylovSchur(),
                                                                          "SymKMCqChrono");
}

TEST(ChSymGenEigenvalueSolverLanczos, SymKMCqChrono_AB) {
    ExecuteEigenSolverCallAB<ChSymGenEigenvalueSolverLanczos, double>(ChSymGenEigenvalueSolverLanczos(),
                                                                      "SymKMCqChrono");
}

TEST(ChSymGenEigenvalueSolverKrylovSchur, SymKMCqChrono) {
    ChSymGenEigenvalueSolverKrylovSchur eigen_solver;
    ExecuteEigenSolverCallKMCq(eigen_solver, "SymKMCq");
}

TEST(ChSymGenEigenvalueSolverLanczos, SymKMCqChrono) {
    ChSymGenEigenvalueSolverLanczos eigen_solver;
    ExecuteEigenSolverCallKMCq(eigen_solver, "SymKMCqChrono");
}

TEST(ChUnsymGenEigenvalueSolverKrylovSchur, UnsymAB) {
    ExecuteEigenSolverCallAB<ChUnsymGenEigenvalueSolverKrylovSchur, std::complex<double>>(
        ChUnsymGenEigenvalueSolverKrylovSchur(chrono_types::make_shared<ChSolverSparseComplexLU>()), "UnsymAB");
}

TEST(ChUnsymGenEigenvalueSolverKrylovSchur, UnsymKMCq) {
    ExecuteEigenSolverCallUnsymKMCq("UnsymKMCq");
}

TEST(ChUnsymGenEigenvalueSolverKrylovSchur, UnsymKRMCq) {
    ExecuteEigenSolverCallKRMCq("UnsymKRMCq");
}

//
// WARNING: while this test is generally working fine, with some machine/compiler/OS configuration it may fail
// because an additional eigenvalue is found by the solver.
// TEST(ChUnsymGenEigenvalueSolverKrylovSchur, UnsymKRMCq_multifreq) {
//    std::string refname = "UnsymKRMCq_multifreq";
//
//    ChSparseMatrix M;
//    ChSparseMatrix K;
//    ChSparseMatrix R;
//    ChSparseMatrix Cq;
//    ChMatrixDynamic<std::complex<double>> sigma_1_mat;
//    ChMatrixDynamic<int> reqeigs_1_mat;
//    ChMatrixDynamic<std::complex<double>> sigma_2_mat;
//    ChMatrixDynamic<int> reqeigs_2_mat;
//    ChMatrixDynamic<std::complex<double>> eigvects_MATLAB_nounique;
//    ChVectorDynamic<std::complex<double>> eigvals_MATLAB_nounique;
//
//    ChMatrixDynamic<std::complex<double>> eigvects_CHRONO;
//    ChVectorDynamic<std::complex<double>> eigvals_CHRONO;
//
//    std::ifstream stream_M(utils::GetValidationDataFile(ref_dir + refname + "/" + refname + "_M.txt"));
//    std::ifstream stream_R(utils::GetValidationDataFile(ref_dir + refname + "/" + refname + "_R.txt"));
//    std::ifstream stream_K(utils::GetValidationDataFile(ref_dir + refname + "/" + refname + "_K.txt"));
//    std::ifstream stream_Cq(utils::GetValidationDataFile(ref_dir + refname + "/" + refname + "_Cq.txt"));
//    std::ifstream stream_sigma_1(utils::GetValidationDataFile(ref_dir + refname + "/" + refname + "_sigma_1.txt"));
//    std::ifstream stream_sigma_2(utils::GetValidationDataFile(ref_dir + refname + "/" + refname + "_sigma_2.txt"));
//    std::ifstream stream_reqeigs_1(utils::GetValidationDataFile(ref_dir + refname + "/" + refname +
//    "_reqeigs_1.txt")); std::ifstream stream_reqeigs_2(utils::GetValidationDataFile(ref_dir + refname + "/" + refname
//    + "_reqeigs_2.txt")); std::ifstream stream_eigvals_MATLAB(
//        utils::GetValidationDataFile(ref_dir + refname + "/" + refname + "_eigvals_MATLAB.txt"));
//    std::ifstream stream_eigvects_MATLAB(
//        utils::GetValidationDataFile(ref_dir + refname + "/" + refname + "_eigvects_MATLAB.txt"));
//
//    fast_matrix_market::read_matrix_market_eigen(stream_M, M);
//    fast_matrix_market::read_matrix_market_eigen(stream_K, K);
//    fast_matrix_market::read_matrix_market_eigen(stream_R, R);
//    fast_matrix_market::read_matrix_market_eigen(stream_Cq, Cq);
//    fast_matrix_market::read_matrix_market_eigen_dense(stream_sigma_1, sigma_1_mat);
//    fast_matrix_market::read_matrix_market_eigen_dense(stream_sigma_2, sigma_2_mat);
//    std::complex<double> sigma_1 = sigma_1_mat(0, 0);
//    std::complex<double> sigma_2 = sigma_2_mat(0, 0);
//    fast_matrix_market::read_matrix_market_eigen_dense(stream_reqeigs_1, reqeigs_1_mat);
//    fast_matrix_market::read_matrix_market_eigen_dense(stream_reqeigs_2, reqeigs_2_mat);
//    int reqeigs_1 = reqeigs_1_mat(0, 0);
//    int reqeigs_2 = reqeigs_2_mat(0, 0);
//    fast_matrix_market::read_matrix_market_eigen_dense(stream_eigvals_MATLAB, eigvals_MATLAB_nounique);
//    fast_matrix_market::read_matrix_market_eigen_dense(stream_eigvects_MATLAB, eigvects_MATLAB_nounique);
//
//    std::list<std::pair<int, std::complex<double>>> eig_requests;
//    eig_requests.push_back(std::make_pair(reqeigs_1, sigma_1));
//    eig_requests.push_back(std::make_pair(reqeigs_2, sigma_2));
//
//    ChUnsymGenEigenvalueSolverKrylovSchur eigen_solver(chrono_types::make_shared<ChSolverSparseComplexLU>());
//    eigen_solver.SortRitzPairs(eigvals_MATLAB_nounique, eigvects_MATLAB_nounique);
//
//    ChMatrixDynamic<std::complex<double>> eigvects_MATLAB(eigvects_MATLAB_nounique.rows(),
//                                                          eigvects_MATLAB_nounique.cols());
//    ChVectorDynamic<std::complex<double>> eigvals_MATLAB(eigvals_MATLAB_nounique.size());
//
//    int found_eigs = 0;
//    eigen_solver.InsertUniqueRitzPairs(eigvals_MATLAB_nounique, eigvects_MATLAB_nounique, eigvals_MATLAB,
//                                       eigvects_MATLAB, eigen_solver.GetNaturalFrequency, found_eigs,
//                                       eigvals_MATLAB_nounique.size());
//    eigvects_MATLAB.conservativeResize(Eigen::NoChange_t::NoChange, found_eigs);
//
//    const bool scaleCq = true;
//
//    ChSparseMatrix A, B;
//    double scaling = eigen_solver.BuildDampedSystem(M, R, K, Cq, A, B, scaleCq);
//
//    modal::Solve<>(eigen_solver, A, B, eigvects_CHRONO, eigvals_CHRONO, eig_requests, true, 0);
//
//    ASSERT_EQ(eigvals_CHRONO.size(), eigvals_MATLAB.size()) << "Different number of eigenvalues found.\n"
//                                                 << "MATLAB:\n"
//                                                 << eigvals_MATLAB << "\nCHRONO:\n"
//                                                 << eigvals_CHRONO << std::endl;
//
//    double max_residual_CHRONO_AB = eigen_solver.GetMaxResidual(A, B, eigvects_CHRONO, eigvals_CHRONO);
//
//    eigvects_CHRONO.bottomRows(Cq.rows()) *= scaling;
//
//    // instead of doing a simple difference, consider the imaginary part to be the same if positive or negative
//    double max_delta_eigvals = GetEigenvaluesMaxDiff(eigvals_CHRONO, eigvals_MATLAB);
//
//    ASSERT_NEAR(max_delta_eigvals, 0, tolerance) << "Eigenvalues not matching.\n"
//                                                 << "MATLAB:\n"
//                                                 << eigvals_MATLAB << "\nCHRONO:\n"
//                                                 << eigvals_CHRONO << std::endl;
//
//    double max_residual_CHRONO = eigen_solver.GetMaxResidual(K, R, M, Cq, eigvects_CHRONO, eigvals_CHRONO);
//
//    ASSERT_NEAR(max_residual_CHRONO, max_residual_CHRONO_AB, tolerance)
//        << "Test developer error: residuals calculated on A,B are different from those calculated on K,R,M,Cq"
//        << std::endl;
//
//    ASSERT_NEAR(max_residual_CHRONO, 0, tolerance) << "Residuals exceeding threshold" << std::endl;
//}

template <typename EigsolverType>
void ExecuteModalSolverUndamped() {
    ChSystemNSC sys;
    auto assembly = BuildBeamFixBody(sys);

    ChSparseMatrix K, R, M, Cq;
    generateKRMCqFromAssembly(assembly, K, R, M, Cq);

    auto eigen_solver = chrono_types::make_shared<EigsolverType>();
    int num_modes = 10;

    ChModalSolverUndamped<EigsolverType> modal_solver(num_modes, 1e-5, true, false, eigen_solver);
    modal_solver.SetClipPositionCoords(false);
    ChMatrixDynamic<typename EigsolverType::ScalarType> eigvects_assembly;
    ChVectorDynamic<typename EigsolverType::ScalarType> eigvals_assembly;
    ChVectorDynamic<double> freq_assembly;
    modal_solver.Solve(*assembly, eigvects_assembly, eigvals_assembly, freq_assembly);

    ChMatrixDynamic<typename EigsolverType::ScalarType> eigvects_KMCq;
    ChVectorDynamic<typename EigsolverType::ScalarType> eigvals_KMCq;
    ChVectorDynamic<double> freq_KMCq;
    modal_solver.Solve(K, M, Cq, eigvects_KMCq, eigvals_KMCq, freq_KMCq);

    double eigvals_diff = GetEigenvaluesMaxDiff(eigvals_assembly, eigvals_KMCq);

    double res_CHRONO_KMCq = eigen_solver->GetMaxResidual(K, M, Cq, eigvects_KMCq, eigvals_KMCq);

    double res_CHRONO_assembly = eigen_solver->GetMaxResidual(K, M, Cq, eigvects_assembly, eigvals_assembly);

    ASSERT_NEAR(eigvals_diff, 0, tolerance)
        << "Eigvals difference: " << eigvals_diff << " above threshold: " << tolerance << std::endl;

    ASSERT_NEAR(res_CHRONO_KMCq, 0, tolerance)
        << "Residuals (KMCq mode): " << res_CHRONO_KMCq << " above threshold: " << tolerance << std::endl;

    ASSERT_NEAR(res_CHRONO_assembly, 0, tolerance)
        << "Residuals (assembly mode): " << res_CHRONO_assembly << " above threshold: " << tolerance << std::endl;
}

TEST(ChModalSolverUndamped, ChUnsymGenEigenvalueSolverKrylovSchur) {
    // WARNING: the test is not actually generating unsymmetric matrices!
    ExecuteModalSolverUndamped<ChUnsymGenEigenvalueSolverKrylovSchur>();
}

TEST(ChModalSolverUndamped, ChSymGenEigenvalueSolverKrylovSchur) {
    ExecuteModalSolverUndamped<ChSymGenEigenvalueSolverKrylovSchur>();
}

TEST(ChModalSolverUndamped, ChSymGenEigenvalueSolverLanczos) {
    ExecuteModalSolverUndamped<ChSymGenEigenvalueSolverLanczos>();
}

TEST(ChModalSolverDamped, ChUnsymGenEigenvalueSolverKrylovSchur) {
    ChSystemNSC sys;
    auto assembly = BuildBeamFixBody(sys);

    ChSparseMatrix K, R, M, Cq;
    generateKRMCqFromAssembly(assembly, K, R, M, Cq);

    auto eigen_solver = chrono_types::make_shared<ChUnsymGenEigenvalueSolverKrylovSchur>();
    int num_modes = 10;

    ChModalSolverDamped modal_solver(num_modes, 1e-5, true, false, eigen_solver);
    modal_solver.SetClipPositionCoords(false);
    ChMatrixDynamic<typename ChUnsymGenEigenvalueSolverKrylovSchur::ScalarType> eigvects_assembly;
    ChVectorDynamic<typename ChUnsymGenEigenvalueSolverKrylovSchur::ScalarType> eigvals_assembly;
    ChVectorDynamic<double> freq_assembly;
    ChVectorDynamic<double> damping_ratios_assembly;
    modal_solver.Solve(*assembly, eigvects_assembly, eigvals_assembly, freq_assembly, damping_ratios_assembly);

    ChMatrixDynamic<typename ChUnsymGenEigenvalueSolverKrylovSchur::ScalarType> eigvects_KRMCq;
    ChVectorDynamic<typename ChUnsymGenEigenvalueSolverKrylovSchur::ScalarType> eigvals_KRMCq;
    ChVectorDynamic<double> freq_KRMCq;
    ChVectorDynamic<double> damping_ratios_KRMCq;
    modal_solver.Solve(K, R, M, Cq, eigvects_KRMCq, eigvals_KRMCq, freq_KRMCq, damping_ratios_KRMCq);

    double eigvals_diff = GetEigenvaluesMaxDiff(eigvals_assembly, eigvals_KRMCq);

    double res_CHRONO_KRMCq = eigen_solver->GetMaxResidual(K, R, M, Cq, eigvects_KRMCq, eigvals_KRMCq);

    double res_CHRONO_assembly = eigen_solver->GetMaxResidual(K, R, M, Cq, eigvects_assembly, eigvals_assembly);

    ASSERT_NEAR(eigvals_diff, 0, tolerance)
        << "Eigvals difference: " << eigvals_diff << " above threshold: " << tolerance << std::endl;

    ASSERT_NEAR(res_CHRONO_KRMCq, 0, tolerance)
        << "Residuals (KRMCq mode): " << res_CHRONO_KRMCq << " above threshold: " << tolerance << std::endl;

    ASSERT_NEAR(res_CHRONO_assembly, 0, tolerance)
        << "Residuals (assembly mode): " << res_CHRONO_assembly << " above threshold: " << tolerance << std::endl;
}
