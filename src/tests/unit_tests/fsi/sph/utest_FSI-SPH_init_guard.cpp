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
// Unit test for the initialization guard on ChFsiFluidSystem::DoStepDynamics.
//
// A standalone fluid system (one used without the ChFsiSystem wrapper) could be
// stepped before Initialize() was called. That bypasses CheckSPHParameters(),
// so every configuration check is skipped, and then dereferences the null
// m_fluid_dynamics that Initialize() would have constructed. The wrapper
// ChFsiSystem::DoStepDynamics has always guarded against this; the base class
// did not, so the two paths disagreed about the same mistake.
//
// Checks:
// 1. Stepping before Initialize() throws std::runtime_error rather than
//    crashing or silently proceeding.
// 2. The documented pattern (configure, Initialize, then step) still works.
// 3. The system is usable after the rejected call, i.e. the failed attempt
//    left no partial state behind.
//
// =============================================================================

#include <iostream>
#include <stdexcept>

#include "chrono/physics/ChSystemSMC.h"
#include "chrono_fsi/sph/ChFsiFluidSystemSPH.h"

using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::fsi::sph;

using std::cout;
using std::endl;

double initial_spacing = 0.01;
double dt = 2e-4;

int num_failures = 0;

void check(bool condition, const std::string& what) {
    if (condition) {
        cout << "  PASS  " << what << endl;
    } else {
        cout << "  FAIL  " << what << endl;
        num_failures++;
    }
}

int main(int argc, char* argv[]) {
    cout << "Initialization guard on ChFsiFluidSystem::DoStepDynamics" << endl;

    // ------------------------------------------------------------------
    // 1. Stepping before Initialize() must throw, not crash.
    // ------------------------------------------------------------------
    {
        ChFsiFluidSystemSPH sysSPH;
        sysSPH.SetVerbose(false);
        sysSPH.SetInitialSpacing(initial_spacing);
        sysSPH.SetStepSize(dt);

        bool threw = false;
        std::string what;
        try {
            sysSPH.DoStepDynamics(dt);
        } catch (const std::runtime_error& e) {
            threw = true;
            what = e.what();
        } catch (...) {
            // Any other exception type still beats a segmentation fault, but the
            // guard is specified to raise std::runtime_error like its wrapper.
        }
        check(threw, "DoStepDynamics before Initialize() throws std::runtime_error");
        check(what.find("not initialized") != std::string::npos, "the exception names the problem: " + what);
        // The rejected call must leave the object untouched: the guard runs before the timer,
        // frame and time counters are touched, so all three still read their constructor values.
        check(sysSPH.GetSimTime() == 0.0, "simulation time is still 0 after the rejected call");
        check(sysSPH.GetRtf() == 0.0, "real-time factor is still 0 after the rejected call");
    }

    // ------------------------------------------------------------------
    // 2. The documented order still works, and 3. the system survives a
    //    rejected call. Both are checked on one system so that the second
    //    genuinely tests recovery rather than a fresh object.
    // ------------------------------------------------------------------
    {
        ChSystemSMC sysMBS;
        ChFsiFluidSystemSPH sysSPH;
        sysSPH.SetVerbose(false);
        sysSPH.SetInitialSpacing(initial_spacing);
        sysSPH.SetStepSize(dt);

        // A rejected call first, so any partial state mutation would show up below.
        bool threw = false;
        try {
            sysSPH.DoStepDynamics(dt);
        } catch (const std::runtime_error&) {
            threw = true;
        }
        check(threw, "the rejected call throws as expected");

        // A minimal box of fluid, enough that Initialize() has something to do.
        ChFsiFluidSystemSPH::SoilProperties mat_props;
        mat_props.density = 1700;
        mat_props.Young_modulus = 1e6;
        mat_props.Poisson_ratio = 0.3;
        mat_props.mu_fric_s = 0.7;
        mat_props.mu_fric_2 = 0.7;
        mat_props.average_diam = 0.005;
        sysSPH.SetCrmSPH(mat_props);

        sysSPH.AddBoxSPH(ChVector3d(0, 0, 0), ChVector3d(0.05, 0.05, 0.05));
        sysSPH.SetComputationalDomain(ChAABB(ChVector3d(-0.2, -0.2, -0.2), ChVector3d(0.2, 0.2, 0.2)));

        bool initialized = true;
        try {
            // Qualified deliberately. ChFsiFluidSystemSPH's Initialize overloads HIDE the
            // base no-arg Initialize(), so an unqualified call does not compile for a
            // standalone user. That is a separate defect this audit tracks; writing this
            // test reproduced it independently, which is why the qualification is here and
            // is commented rather than silently applied.
            sysSPH.ChFsiFluidSystem::Initialize();
        } catch (const std::exception& e) {
            initialized = false;
            cout << "  (Initialize failed: " << e.what() << ")" << endl;
        }
        check(initialized, "Initialize() succeeds after a rejected step");

        if (initialized) {
            bool stepped = true;
            try {
                sysSPH.DoStepDynamics(dt);
            } catch (const std::exception& e) {
                stepped = false;
                cout << "  (step failed: " << e.what() << ")" << endl;
            }
            check(stepped, "DoStepDynamics succeeds after Initialize()");
        }
    }

    // ------------------------------------------------------------------
    // 4. A fresh object that is configured and initialized in the documented
    //    order, with no rejected call first, steps without throwing.
    // ------------------------------------------------------------------
    {
        ChFsiFluidSystemSPH sysSPH;
        sysSPH.SetVerbose(false);
        sysSPH.SetInitialSpacing(initial_spacing);
        sysSPH.SetStepSize(dt);
        ChFsiFluidSystemSPH::SoilProperties mat_props;
        mat_props.density = 1700;
        mat_props.Young_modulus = 1e6;
        mat_props.Poisson_ratio = 0.3;
        mat_props.mu_fric_s = 0.7;
        mat_props.mu_fric_2 = 0.7;
        mat_props.average_diam = 0.005;
        sysSPH.SetCrmSPH(mat_props);
        sysSPH.AddBoxSPH(ChVector3d(0, 0, 0), ChVector3d(0.05, 0.05, 0.05));
        sysSPH.SetComputationalDomain(ChAABB(ChVector3d(-0.2, -0.2, -0.2), ChVector3d(0.2, 0.2, 0.2)));
        bool ok = true;
        try {
            sysSPH.ChFsiFluidSystem::Initialize();
            sysSPH.DoStepDynamics(dt);
        } catch (const std::exception& e) {
            ok = false;
            cout << "  (fresh object failed: " << e.what() << ")" << endl;
        }
        check(ok, "a fresh, properly initialized system steps without throwing");
        check(sysSPH.GetSimTime() > 0.0, "and its simulation time advanced");
    }

    cout << (num_failures == 0 ? "Test succeeded" : "Test FAILED") << endl;
    return num_failures == 0 ? 0 : 1;
}
