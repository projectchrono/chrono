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
// Unit test for the initialization guard on ChFsiFluidSystemSPH configuration
// setters. These setters write only the host-side parameter structure; device
// parameters are uploaded once during Initialize(). A call after Initialize()
// therefore cannot take effect and must fail loudly instead of silently.
//
// Checks:
// 1. The documented pattern (configure, then Initialize) works unchanged.
// 2. Every guarded setter throws std::runtime_error after Initialize().
// 3. SetComputationalDomain and SetOutputLevel remain callable after
//    Initialize() (the CRMTerrain moving patch updates the computational
//    domain mid-simulation; the output level is a host-only setting).
// 4. The system still advances after the rejected calls (no partial state
//    mutation).
//
// =============================================================================

#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>

#include "chrono/physics/ChSystemSMC.h"

#include "chrono_fsi/sph/ChFsiProblemSPH.h"

using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::fsi::sph;

// Dimensions of the fluid box and initial spacing
double bxDim = 0.1;
double byDim = 0.1;
double bzDim = 0.1;
double initial_spacing = 0.01;
double dt = 2e-4;

int num_failures = 0;

void ExpectThrow(const std::string& name, std::function<void()> f) {
    try {
        f();
        std::cout << "FAIL (no throw): " << name << std::endl;
        num_failures++;
    } catch (const std::runtime_error&) {
        std::cout << "  ok (guarded):  " << name << std::endl;
    }
}

void ExpectNoThrow(const std::string& name, std::function<void()> f) {
    try {
        f();
        std::cout << "  ok (allowed):  " << name << std::endl;
    } catch (const std::exception& e) {
        std::cout << "FAIL (threw):   " << name << " - " << e.what() << std::endl;
        num_failures++;
    }
}

int main(int argc, char* argv[]) {
    // Create the FSI problem (same minimal setup as utest_FSI-SPH_Poiseuille_flow)
    ChSystemSMC sysMBS;
    ChFsiProblemCartesian fsi(initial_spacing, &sysMBS);
    fsi.SetVerbose(false);
    auto sysSPH = fsi.GetFluidSystemSPH();

    fsi.SetGravitationalAcceleration(ChVector3d(0, 0, 0));

    ChFsiFluidSystemSPH::FluidProperties fluid_props;
    fluid_props.density = 1000;
    fluid_props.viscosity = 1;
    fsi.SetCfdSPH(fluid_props);

    ChFsiFluidSystemSPH::SPHParameters sph_params;
    sph_params.integration_scheme = IntegrationScheme::RK2;
    sph_params.num_bce_layers = 3;
    sph_params.initial_spacing = initial_spacing;
    sph_params.d0_multiplier = 1;
    sph_params.max_velocity = 0.1;
    sph_params.shifting_method = ShiftingMethod::NONE;
    sph_params.density_reinit_steps = 10000;
    sph_params.viscosity_method = ViscosityMethod::LAMINAR;
    sph_params.use_delta_sph = false;
    sph_params.eos_type = EosType::ISOTHERMAL;
    sph_params.use_consistent_gradient_discretization = true;
    sph_params.use_consistent_laplacian_discretization = true;
    fsi.SetSPHParameters(sph_params);

    fsi.SetStepSizeCFD(dt);
    fsi.SetStepsizeMBD(dt);

    // Pre-initialization setter calls must be accepted (documented pattern)
    std::cout << "Before Initialize():" << std::endl;
    ExpectNoThrow("SetArtificialViscosityCoefficient (pre-init)", [&] { sysSPH->SetArtificialViscosityCoefficient(0.02); });
    ExpectNoThrow("SetBodyForce (pre-init)", [&] { sysSPH->SetBodyForce(ChVector3d(0, 0, 0)); });

    ChVector3d fsize(bxDim, byDim, bzDim - 2 * initial_spacing);
    fsi.Construct(fsize, ChVector3d(0, 0, initial_spacing), BoxSide::Z_NEG | BoxSide::Z_POS);

    ChVector3d c_min(-bxDim / 2 - initial_spacing / 2, -byDim / 2 - initial_spacing / 2, -10 * initial_spacing);
    ChVector3d c_max(+bxDim / 2 + initial_spacing / 2, +byDim / 2 + initial_spacing / 2, bzDim + 10 * initial_spacing);
    ChAABB domain(c_min, c_max);
    fsi.SetComputationalDomain(domain, BC_ALL_PERIODIC);

    fsi.Initialize();

    // Prove the system advances before any post-init setter calls
    for (int step = 0; step < 5; step++)
        fsi.DoStepDynamics(dt);

    // Every device-parameter setter must be rejected after Initialize()
    std::cout << "After Initialize():" << std::endl;

    ChFsiFluidSystemSPH::ElasticMaterialProperties mat_props;
    ChFsiFluidSystemSPH::LinSolverParameters linsolv_params;

    ExpectThrow("SetBoundaryType", [&] { sysSPH->SetBoundaryType(BoundaryMethod::ADAMI); });
    ExpectThrow("SetViscosityType", [&] { sysSPH->SetViscosityType(ViscosityMethod::LAMINAR); });
    ExpectThrow("SetArtificialViscosityCoefficient", [&] { sysSPH->SetArtificialViscosityCoefficient(0.5); });
    ExpectThrow("SetKernelType", [&] { sysSPH->SetKernelType(KernelType::CUBIC_SPLINE); });
    ExpectThrow("SetShiftingMethod", [&] { sysSPH->SetShiftingMethod(ShiftingMethod::NONE); });
    ExpectThrow("SetSPHLinearSolver", [&] { sysSPH->SetSPHLinearSolver(SolverType::JACOBI); });
    ExpectThrow("SetIntegrationScheme", [&] { sysSPH->SetIntegrationScheme(IntegrationScheme::RK2); });
    ExpectThrow("SetContainerDim", [&] { sysSPH->SetContainerDim(ChVector3d(1, 1, 1)); });
    ExpectThrow("SetActiveDomain", [&] { sysSPH->SetActiveDomain(ChVector3d(1, 1, 1)); });
    ExpectThrow("SetActiveDomainDelay", [&] { sysSPH->SetActiveDomainDelay(1.0); });
    ExpectThrow("SetNumBCELayers", [&] { sysSPH->SetNumBCELayers(3); });
    ExpectThrow("SetInitPressure", [&] { sysSPH->SetInitPressure(0.1); });
    ExpectThrow("SetGravitationalAcceleration", [&] { sysSPH->SetGravitationalAcceleration(ChVector3d(0, 0, -9.8)); });
    ExpectThrow("SetBodyForce", [&] { sysSPH->SetBodyForce(ChVector3d(0.1, 0, 0)); });
    ExpectThrow("SetInitialSpacing", [&] { sysSPH->SetInitialSpacing(0.02); });
    ExpectThrow("SetKernelMultiplier", [&] { sysSPH->SetKernelMultiplier(1.2); });
    ExpectThrow("SetDensity", [&] { sysSPH->SetDensity(1200); });
    ExpectThrow("SetShiftingPPSTParameters", [&] { sysSPH->SetShiftingPPSTParameters(3.0, 0.0); });
    ExpectThrow("SetShiftingXSPHParameters", [&] { sysSPH->SetShiftingXSPHParameters(0.5); });
    ExpectThrow("SetShiftingDiffusionParameters", [&] { sysSPH->SetShiftingDiffusionParameters(1.0, 1.0, 1.0); });
    ExpectThrow("SetConsistentDerivativeDiscretization", [&] { sysSPH->SetConsistentDerivativeDiscretization(false, false); });
    ExpectThrow("SetCohesionForce", [&] { sysSPH->SetCohesionForce(0.0); });
    ExpectThrow("SetNumProximitySearchSteps", [&] { sysSPH->SetNumProximitySearchSteps(4); });
    ExpectThrow("SetUseVariableTimeStep", [&] { sysSPH->SetUseVariableTimeStep(false); });
    ExpectThrow("SetCfdSPH", [&] { sysSPH->SetCfdSPH(fluid_props); });
    ExpectThrow("SetElasticSPH", [&] { sysSPH->SetElasticSPH(mat_props); });
    ExpectThrow("SetSPHParameters", [&] { sysSPH->SetSPHParameters(sph_params); });
    ExpectThrow("SetLinSolverParameters", [&] { sysSPH->SetLinSolverParameters(linsolv_params); });

    // These must remain callable after Initialize()
    ExpectNoThrow("SetComputationalDomain (post-init)", [&] { sysSPH->SetComputationalDomain(domain); });
    ExpectNoThrow("SetOutputLevel (post-init)", [&] { sysSPH->SetOutputLevel(OutputLevel::STATE); });

    // The rejected calls must not have altered any state: the system still advances
    ExpectNoThrow("DoStepDynamics after rejected calls", [&] {
        for (int step = 0; step < 5; step++)
            fsi.DoStepDynamics(dt);
    });

    // ------------------------------------------------------------------------
    // Standalone fluid system (no ChFsiSystem wrapper): the guards must arm here
    // too, i.e. Initialize() itself must set the initialization flag.
    std::cout << "Standalone ChFsiFluidSystemSPH (no FSI wrapper):" << std::endl;
    ChFsiFluidSystemSPH sph2;
    sph2.SetVerbose(false);
    ExpectNoThrow("standalone pre-init configuration", [&] {
        sph2.SetInitialSpacing(initial_spacing);
        sph2.SetCfdSPH(fluid_props);
        sph2.SetSPHParameters(sph_params);
        sph2.SetStepSize(dt);
        sph2.SetGravitationalAcceleration(ChVector3d(0, 0, 0));
        sph2.SetComputationalDomain(domain, BC_ALL_PERIODIC);
    });
    sph2.AddBoxSPH(ChVector3d(0, 0, bzDim / 2), ChVector3d(bxDim / 2, byDim / 2, bzDim / 4));
    ExpectNoThrow("standalone Initialize()", [&] { sph2.ChFsiFluidSystem::Initialize(); });  // base entry point (derived overloads hide it)
    ExpectThrow("standalone SetDensity (post-init)", [&] { sph2.SetDensity(1200); });
    ExpectThrow("standalone SetArtificialViscosityCoefficient (post-init)", [&] { sph2.SetArtificialViscosityCoefficient(0.3); });

    if (num_failures > 0) {
        std::cout << "\n" << num_failures << " failure(s)" << std::endl;
        return 1;
    }
    std::cout << "\nAll setter-guard checks passed" << std::endl;
    return 0;
}
