// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
//
// Unit test for the CRM rheology-failure error flag.
//
// The flag aborts a run when the updated stress state of the active rheology
// model is not finite. It must fire on a genuinely poisoned state and must not
// fire on state the active model does not use.
//
// A CRM bed created with the default particle pressure starts with a non-finite
// consolidation state: FsiDataManager::AddSphParticle derives the MCC specific
// volume with log(pc / p1) and log(pc / confining), which is not finite for the
// default pc = 0. Neither of the two runs below may be aborted by that state:
//
//   1. Under MU_OF_I the consolidation state is neither read nor updated, so it
//      cannot indicate a failure of the active model.
//   2. Under MCC it is part of the update, but the specific volume is clamped
//      (fmax) on the first step, which also removes the non-finite value.
//
// Case 2 is NOT a claim that a default-pressure MCC bed is physically sound. The
// non-finite specific volume is read before it is clamped: it feeds the MCC bulk
// modulus candidate, and fmin(fmax(K_cand, 0.1 * K_bulk), K_bulk) returns the
// floor when the candidate is NaN. Such a bed therefore runs with the bulk
// modulus pinned at a tenth of nominal on the first step and with a specific
// volume of exactly 1.0 (zero void ratio) thereafter, with no diagnostic. That
// is an initialization defect, reported separately upstream. What this test
// asserts is only that the error flag must not turn that silent repair into an
// abort, since aborting would change behavior that this PR is not meant to
// change.
//
// Case 1 is a regression guard: checking consolidation state under MU_OF_I made
// the flag abort healthy runs. A genuine firing of the flag is not reproducible
// from the public API at this revision (any input that makes the stress state
// non-finite also makes positions non-finite in the same step, so the position
// scan reports first); it is covered by a forced-fire build instead.
//
// =============================================================================

#include <iostream>
#include <stdexcept>
#include <string>

#include "chrono/physics/ChSystemSMC.h"

#include "chrono_fsi/sph/ChFsiProblemSPH.h"

using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::fsi::sph;

double bxDim = 0.2;
double byDim = 0.2;
double bzDim = 0.1;
double initial_spacing = 0.02;
double dt = 1e-4;
int num_steps = 10;

int num_failures = 0;

// Step a CRM bed built with the DEFAULT particle pressure under the given rheology.
// Returns true if the run threw.
bool RunBed(RheologyCRM rheology, std::string& what, bool depth_pressure = false) {
    ChSystemSMC sysMBS;
    ChFsiProblemCartesian fsi(initial_spacing, &sysMBS);
    fsi.SetVerbose(false);

    // The error checks this test exercises are enabled by default; ask for them
    // explicitly so the test cannot be silently voided by a change of default.
    fsi.GetFluidSystemSPH()->EnableGPUErrorCheck(true);
    fsi.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
    sysMBS.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));

    ChFsiFluidSystemSPH::ElasticMaterialProperties mat_props;
    mat_props.density = 1700;
    mat_props.Young_modulus = 1e6;
    mat_props.Poisson_ratio = 0.3;
    mat_props.mu_I0 = 0.04;
    mat_props.mu_fric_s = 0.8;
    mat_props.mu_fric_2 = 0.8;
    mat_props.average_diam = 0.005;
    mat_props.cohesion_coeff = 0;
    mat_props.rheology_model = rheology;
    mat_props.mcc_M = 1.2;
    mat_props.mcc_kappa = 0.01;
    mat_props.mcc_lambda = 0.1;
    mat_props.mcc_v_lambda = 2.0;
    fsi.SetElasticSPH(mat_props);

    ChFsiFluidSystemSPH::SPHParameters sph_params;
    sph_params.integration_scheme = IntegrationScheme::RK2;
    sph_params.initial_spacing = initial_spacing;
    sph_params.d0_multiplier = 1;
    sph_params.artificial_viscosity = 0.5;
    sph_params.viscosity_method = ViscosityMethod::ARTIFICIAL_BILATERAL;
    sph_params.boundary_method = BoundaryMethod::ADAMI;
    fsi.SetSPHParameters(sph_params);

    fsi.SetStepSizeCFD(dt);
    fsi.SetStepsizeMBD(dt);

    // Bed with a floor and side walls; particle pressure is left at its default.
    if (depth_pressure)
        fsi.RegisterParticlePropertiesCallback(chrono_types::make_shared<DepthPressurePropertiesCallback>(bzDim));

    fsi.Construct(ChVector3d(bxDim, byDim, bzDim), ChVector3d(0, 0, 0), BoxSide::ALL & ~BoxSide::Z_POS);
    fsi.Initialize();

    try {
        for (int i = 0; i < num_steps; i++)
            fsi.DoStepDynamics(dt);
    } catch (const std::exception& e) {
        what = e.what();
        return true;
    }
    return false;
}

int main(int argc, char* argv[]) {
    // ------------------------------------------------------------------------
    // 1. mu(I): the non-finite consolidation state is not part of this model's
    //    update, so the run must not be aborted by it.
    std::string what;
    std::cout << "MU_OF_I bed with default particle pressure:" << std::endl;
    if (RunBed(RheologyCRM::MU_OF_I, what)) {
        std::cout << "FAIL: the run aborted: " << what << std::endl;
        num_failures++;
    } else {
        std::cout << "  ok: ran " << num_steps << " steps without aborting" << std::endl;
    }

    // ------------------------------------------------------------------------
    // 2. MCC, same default-pressure initial condition: must also run. By the time
    //    the predicate runs, the model has clamped the state it poisoned itself
    //    with (see the note at the top of this file: the run is not healthy, it is
    //    silently repaired). The flag must not convert that into an abort.
    what.clear();
    std::cout << "MCC bed with default particle pressure (non-finite initial consolidation state):" << std::endl;
    if (RunBed(RheologyCRM::MCC, what)) {
        std::cout << "FAIL: the run aborted: " << what << std::endl;
        num_failures++;
    } else {
        std::cout << "  ok: ran " << num_steps << " steps without aborting" << std::endl;
    }

    // ------------------------------------------------------------------------
    // 3. MCC with a physical depth-based initial pressure (the documented way to
    //    initialize a Cam-Clay bed): must run.
    what.clear();
    std::cout << "MCC bed with depth-based initial pressure:" << std::endl;
    if (RunBed(RheologyCRM::MCC, what, true)) {
        std::cout << "FAIL: the run aborted: " << what << std::endl;
        num_failures++;
    } else {
        std::cout << "  ok: ran " << num_steps << " steps without aborting" << std::endl;
    }

    if (num_failures > 0) {
        std::cout << "\n" << num_failures << " failure(s)" << std::endl;
        return 1;
    }
    std::cout << "\nAll rheology-error-flag checks passed" << std::endl;
    return 0;
}
