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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================
//
// Show how to use the ChModalAssembly to do a basic modal analysis (eigenvalues
// and eigenvector of the ChModalAssembly, which can also contain constraints.
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChLinkLock.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/fea/ChElementBeamEuler.h"
#include "chrono/fea/ChBuilderBeam.h"
#include "chrono/fea/ChMeshFileLoader.h"

#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/utils/ChUtilsValidation.h"

#include "chrono/fea/ChMesh.h"

#include "chrono/core/ChTimer.h"

#include "chrono_modal/ChModalAssembly.h"

#include "chrono/solver/ChDirectSolverLScomplex.h"

#include "chrono_thirdparty/filesystem/path.h"
#include <iomanip>

#include <unsupported/Eigen/SparseExtra>

#include "gtest/gtest.h"

using namespace chrono;
using namespace chrono::modal;
using namespace chrono::fea;

static const std::string val_dir = "../RESULTS/";
static const std::string out_dir = val_dir + "modal/";
static const std::string ref_dir = "testing/modal/analysis/";

static const double tolerance = 1e-3;
static const bool verbose = true;

void prepare_folders() {
    // Create output directory (if it does not already exist)
    if (!filesystem::create_directory(filesystem::path(val_dir))) {
        std::cerr << "Error creating directory " << val_dir << std::endl;
        throw std::invalid_argument("Error creating directory " + val_dir);
    }
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cerr << "Error creating directory " << out_dir << std::endl;
        throw std::invalid_argument("Error creating directory " + out_dir);
    }
}

TEST(ChGeneralizedEigenvalueSolverKrylovSchur, BeamFixBody) {
    prepare_folders();

    /*
     * Beam with end body
     *
     *   (fixed node)----()----()----()----()<--link-->[body]
     *
     */

    ChSystemNSC sys;
    auto assembly = chrono_types::make_shared<ChAssembly>();

    sys.Add(assembly);

    double beam_Young = 100.e6;
    double beam_density = 1000;
    double beam_wz = 0.3;
    double beam_wy = 0.05;
    double beam_L = 6;

    double body_end_xwidth = 0.5;

    unsigned int num_modes = 24;

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

    // This helps creating sequences of nodes and ChElementBeamEuler elements:
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
    sys.Update();

    ChGeneralizedEigenvalueSolverKrylovSchur eigen_solver;
    ChModalSolveUndamped modal_solver(num_modes, 1e-5,
                                      500,            // max iterations per each {modes,freq} pair
                                      1e-10,          // tolerance
                                      true,           // verbose
                                      eigen_solver);  //  solver type

    ChMatrixDynamic<std::complex<double>> eigvects;
    ChVectorDynamic<std::complex<double>> eigvals;
    ChVectorDynamic<double> freq;

    modal_solver.Solve(*assembly, eigvects, eigvals, freq);

    if (verbose) {
        for (unsigned int mode = 0; mode < eigvects.cols(); ++mode) {
            std::cout << "Mode: " << std::setw(2) << mode << " | " << std::setw(4) << eigvals(mode) << std::endl;
        }
    }

    std::string testname = std::string(::testing::UnitTest::GetInstance()->current_test_suite()->name()) + "_" +
                           std::string(::testing::UnitTest::GetInstance()->current_test_info()->name());

    ChMatrixDynamic<std::complex<double>> eigvects_MATLAB;
    ChVectorDynamic<std::complex<double>> eigvals_MATLAB;
    ChVectorDynamic<double> freq_MATLAB;
    Eigen::loadMarketVector(eigvals_MATLAB, utils::GetValidationDataFile(ref_dir + testname + "_eigvals_MATLAB.txt"));
    Eigen::saveMarketVector(eigvals, out_dir + testname + "_eigvals_CHRONO.txt");
    ASSERT_NEAR((eigvals_MATLAB - eigvals).cwiseQuotient(eigvals).lpNorm<Eigen::Infinity>(), 0, tolerance);
}
