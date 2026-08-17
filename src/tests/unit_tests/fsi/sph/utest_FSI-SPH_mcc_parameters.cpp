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
// Unit test for the Modified Cam-Clay parameter preconditions.
//
// The MCC equations require 0 < kappa < lambda: the elastic bulk modulus is
// K = v p / kappa, and the hardening update divides by (lambda - kappa). Before
// these checks existed, a configuration violating that ran to completion with no
// diagnostic at all and produced essentially zero bearing capacity, which is how
// the transposed kappa/lambda documentation went unnoticed.
//
// Checks:
// 1. A valid MCC parameter set is accepted.
// 2. Every violating set is rejected: non-finite, kappa <= 0, lambda <= kappa
//    (including exact equality), M <= 0, v_lambda <= 0.
// 3. An ordering that holds by a vanishingly small margin is warned about and
//    accepted, not rejected: it is a legal if extreme model. Both arms of the
//    floor are exercised, and a gap just above the floor must stay silent.
// 4. A default-constructed material, which has all four MCC values at zero, is
//    accepted for mu(I) and rejected only when MCC is actually selected.
// 5. A CFD problem is never rejected by these checks, even when a stale MCC
//    rheology selection is left behind. SetCfdSPH clears elastic_SPH without
//    resetting rheology_model_crm, so the pair can come apart and the checks are
//    gated on both flags.
// 6. The rejection happens on the real path: a full Initialize() with the
//    transposed values throws before any time stepping.
//
// =============================================================================

#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>

#include "chrono/physics/ChSystemSMC.h"

#include "chrono_fsi/sph/ChFsiProblemSPH.h"

using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::fsi::sph;

double initial_spacing = 0.02;
double dt = 1e-4;

int num_failures = 0;

void ExpectThrow(const std::string& name, std::function<void()> f) {
    try {
        f();
        std::cout << "FAIL (no throw): " << name << std::endl;
        num_failures++;
    } catch (const std::runtime_error& e) {
        std::cout << "  ok (rejected): " << name << std::endl;
        std::cout << "                 " << e.what() << std::endl;
    }
}

void ExpectNoThrow(const std::string& name, std::function<void()> f) {
    try {
        f();
        std::cout << "  ok (accepted): " << name << std::endl;
    } catch (const std::exception& e) {
        std::cout << "FAIL (threw):   " << name << " - " << e.what() << std::endl;
        num_failures++;
    }
}

// The near-equality condition is reported on cerr and accepted, not thrown, so these two helpers
// capture cerr and key on a substring unique to that one message. CheckSPHParameters emits other,
// unrelated warnings, so keying on the word WARNING alone would give a test that can pass for the
// wrong reason.
const char* gap_warning_marker = "differ by only";

std::string CaptureCerr(std::function<void()> f) {
    std::ostringstream capture;
    std::streambuf* saved = std::cerr.rdbuf(capture.rdbuf());
    try {
        f();
    } catch (...) {
        std::cerr.rdbuf(saved);
        throw;
    }
    std::cerr.rdbuf(saved);
    return capture.str();
}

void ExpectGapWarning(const std::string& name, std::function<void()> f) {
    try {
        std::string out = CaptureCerr(f);
        if (out.find(gap_warning_marker) == std::string::npos) {
            std::cout << "FAIL (accepted, but did not warn): " << name << std::endl;
            num_failures++;
            return;
        }
        std::cout << "  ok (warned, accepted): " << name << std::endl;
        std::cout << "                 " << out.substr(0, out.find('\n')) << std::endl;
    } catch (const std::exception& e) {
        std::cout << "FAIL (threw):   " << name << " - " << e.what() << std::endl;
        num_failures++;
    }
}

void ExpectNoGapWarning(const std::string& name, std::function<void()> f) {
    try {
        std::string out = CaptureCerr(f);
        if (out.find(gap_warning_marker) != std::string::npos) {
            std::cout << "FAIL (warned):  " << name << std::endl;
            num_failures++;
            return;
        }
        std::cout << "  ok (no warning): " << name << std::endl;
    } catch (const std::exception& e) {
        std::cout << "FAIL (threw):   " << name << " - " << e.what() << std::endl;
        num_failures++;
    }
}

// A parameter set that satisfies every precondition. Individual tests spoil one value at a time,
// so that each failure is attributable to the value it changed.
ChFsiFluidSystemSPH::SoilProperties ValidMCC() {
    ChFsiFluidSystemSPH::SoilProperties mat_props;
    mat_props.density = 1700;
    mat_props.Young_modulus = 1e6;
    mat_props.Poisson_ratio = 0.3;
    mat_props.rheology_model = RheologyCRM::MCC;
    mat_props.mcc_M = 1.2;
    mat_props.mcc_kappa = 0.00625;
    mat_props.mcc_lambda = 0.025;
    mat_props.mcc_v_lambda = 2.0;
    return mat_props;
}

ChFsiFluidSystemSPH::SPHParameters CrmSPHParameters() {
    ChFsiFluidSystemSPH::SPHParameters sph_params;
    sph_params.integration_scheme = IntegrationScheme::RK2;
    // Set explicitly, even though shifting is irrelevant to parameter validation and even though
    // this is the documented default: SPHParameters::SPHParameters() does not initialize
    // shifting_method, so reading it without assigning it first is undefined behavior. Leaving it
    // alone made this test take different branches in the single- and double-precision builds of
    // identical source.
    sph_params.shifting_method = ShiftingMethod::XSPH;
    sph_params.initial_spacing = initial_spacing;
    sph_params.d0_multiplier = 1;
    sph_params.num_bce_layers = 3;
    sph_params.artificial_viscosity = 0.5;
    sph_params.viscosity_method = ViscosityMethod::ARTIFICIAL_BILATERAL;
    sph_params.boundary_method = BoundaryMethod::ADAMI;
    return sph_params;
}

// Validate a material without building a bed or touching the GPU. CheckSPHParameters() is public
// and is exactly what Initialize() calls, so this exercises the shipped code path; case 5 below
// then proves the same call really is reached from Initialize().
void ValidateMaterial(const ChFsiFluidSystemSPH::SoilProperties& mat_props) {
    ChFsiFluidSystemSPH sysSPH;
    sysSPH.SetVerbose(false);
    sysSPH.SetCrmSPH(mat_props);
    sysSPH.SetSPHParameters(CrmSPHParameters());
    sysSPH.CheckSPHParameters();
}

int main(int argc, char* argv[]) {
    std::cout << "1. A valid MCC parameter set:" << std::endl;
    ExpectNoThrow("kappa 0.00625 < lambda 0.025, M 1.2, v_lambda 2.0", [&] { ValidateMaterial(ValidMCC()); });

    std::cout << "\n2. Violating parameter sets, one spoiled value each:" << std::endl;

    ExpectThrow("lambda < kappa (the transposition this test exists for)", [&] {
        auto m = ValidMCC();
        m.mcc_kappa = 0.025;
        m.mcc_lambda = 0.00625;
        ValidateMaterial(m);
    });
    ExpectThrow("lambda == kappa (zero hardening denominator)", [&] {
        auto m = ValidMCC();
        m.mcc_lambda = m.mcc_kappa;
        ValidateMaterial(m);
    });
    ExpectThrow("kappa == 0 (division by zero in K = v p / kappa)", [&] {
        auto m = ValidMCC();
        m.mcc_kappa = 0;
        ValidateMaterial(m);
    });
    ExpectThrow("kappa < 0", [&] {
        auto m = ValidMCC();
        m.mcc_kappa = -0.01;
        ValidateMaterial(m);
    });
    ExpectThrow("M == 0", [&] {
        auto m = ValidMCC();
        m.mcc_M = 0;
        ValidateMaterial(m);
    });
    ExpectThrow("M < 0", [&] {
        auto m = ValidMCC();
        m.mcc_M = -1.2;
        ValidateMaterial(m);
    });
    ExpectThrow("v_lambda == 0", [&] {
        auto m = ValidMCC();
        m.mcc_v_lambda = 0;
        ValidateMaterial(m);
    });
    ExpectThrow("v_lambda < 0", [&] {
        auto m = ValidMCC();
        m.mcc_v_lambda = -1.0;
        ValidateMaterial(m);
    });
    ExpectThrow("kappa is NaN", [&] {
        auto m = ValidMCC();
        m.mcc_kappa = std::numeric_limits<double>::quiet_NaN();
        ValidateMaterial(m);
    });
    ExpectThrow("lambda is infinite", [&] {
        auto m = ValidMCC();
        m.mcc_lambda = std::numeric_limits<double>::infinity();
        ValidateMaterial(m);
    });

    // ------------------------------------------------------------------------
    // 3. lambda > kappa can hold by a margin so small that the hardening law, which divides by
    //    (lambda - kappa), is effectively dividing by nothing. The floor is
    //    max(1e-6, 1e-3 * lambda), so with lambda near 0.025 the relative arm governs at
    //    2.5e-5 and the absolute arm only matters for far smaller slopes. The two warning
    //    cases below are chosen to fire through different arms, and the last case sits just
    //    above the floor to show the warning is bounded and does not fire on ordinary values.
    std::cout << "\n3. An ordering that holds only barely (warn, do not reject):" << std::endl;

    ExpectNoGapWarning("the valid set, gap 0.01875 against a floor of 2.5e-5: silent", [&] {  //
        ValidateMaterial(ValidMCC());
    });
    ExpectGapWarning("gap 1e-7, below both arms of the floor", [&] {
        auto m = ValidMCC();
        m.mcc_kappa = 0.025;
        m.mcc_lambda = 0.0250001;
        ValidateMaterial(m);
    });
    ExpectGapWarning("gap 1e-5, above the absolute arm at 1e-6 but below the relative arm at 2.5e-5", [&] {
        auto m = ValidMCC();
        m.mcc_kappa = 0.025;
        m.mcc_lambda = 0.02501;
        ValidateMaterial(m);
    });
    ExpectNoGapWarning("gap 1e-4, just above the floor: silent", [&] {
        auto m = ValidMCC();
        m.mcc_kappa = 0.025;
        m.mcc_lambda = 0.0251;
        ValidateMaterial(m);
    });

    std::cout << "\n4. The all-zero MCC defaults:" << std::endl;
    ExpectNoThrow("default material, mu(I) selected: the zero MCC defaults are irrelevant", [&] {
        ChFsiFluidSystemSPH::SoilProperties m;  // mcc_* all default to 0
        m.rheology_model = RheologyCRM::MU_OF_I;
        ValidateMaterial(m);
    });
    ExpectThrow("default material, MCC selected: unset parameters must not run silently", [&] {
        ChFsiFluidSystemSPH::SoilProperties m;
        m.rheology_model = RheologyCRM::MCC;
        ValidateMaterial(m);
    });

    std::cout << "\n5. A CFD problem is out of scope for these checks:" << std::endl;
    ExpectNoThrow("SetCrmSPH(MCC) then SetCfdSPH: stale MCC selection must not reject CFD", [&] {
        ChFsiFluidSystemSPH sysSPH;
        sysSPH.SetVerbose(false);
        sysSPH.SetCrmSPH(ValidMCC());  // leaves rheology_model_crm == MCC ...
        ChFsiFluidSystemSPH::FluidProperties fluid_props;
        fluid_props.density = 1000;
        fluid_props.viscosity = 1;
        sysSPH.SetCfdSPH(fluid_props);  // ... and clears elastic_SPH without resetting it
        auto sph_params = CrmSPHParameters();
        sph_params.viscosity_method = ViscosityMethod::LAMINAR;  // CFD rejects ARTIFICIAL_BILATERAL
        sysSPH.SetSPHParameters(sph_params);
        sysSPH.CheckSPHParameters();
    });

    // ------------------------------------------------------------------------
    // 6. The check must fire on the path a user actually takes, not only when
    //    CheckSPHParameters() is called directly. These are the values measured
    //    to produce a silent zero-bearing-capacity run before the fix.
    std::cout << "\n6. Full Initialize() on a CRM bed:" << std::endl;

    auto build_and_initialize = [](double kappa, double lambda) {
        ChSystemSMC sysMBS;
        ChFsiProblemCartesian fsi(initial_spacing, &sysMBS);
        fsi.SetVerbose(false);
        fsi.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
        sysMBS.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));

        auto mat_props = ValidMCC();
        mat_props.mcc_kappa = kappa;
        mat_props.mcc_lambda = lambda;
        fsi.SetCrmSPH(mat_props);
        fsi.SetSPHParameters(CrmSPHParameters());
        fsi.SetStepSizeCFD(dt);
        fsi.SetStepsizeMBD(dt);

        fsi.Construct(ChVector3d(0.2, 0.2, 0.1), ChVector3d(0, 0, 0), BoxSide::ALL & ~BoxSide::Z_POS);
        fsi.Initialize();
        fsi.DoStepDynamics(dt);
    };

    ExpectThrow("Initialize() with kappa 0.025, lambda 0.00625 (the measured silent-failure case)",
                [&] { build_and_initialize(0.025, 0.00625); });
    ExpectNoThrow("Initialize() and one step with kappa 0.00625, lambda 0.025",
                  [&] { build_and_initialize(0.00625, 0.025); });

    if (num_failures > 0) {
        std::cout << "\n" << num_failures << " failure(s)" << std::endl;
        return 1;
    }
    std::cout << "\nAll MCC parameter checks passed" << std::endl;
    return 0;
}
