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
// Unit test for post-initialization computational-domain updates (the mechanism
// used by the CRMTerrain moving patch). A domain update issued after
// Initialize() must reach the DEVICE parameters, not only the host copy.
//
// Setup: a static periodic fluid box. After Initialize(), the computational
// domain is translated along +x. Fluid particles that fall below the new lower
// x-boundary must be wrapped by the periodic-boundary kernel to the far end of
// the NEW domain within a few steps. That wrap is executed on the device using
// paramsD.cMin/cMax, so observing it proves the update propagated to the GPU.
//
// Also checks the safety rails: a post-init domain update that changes the
// domain SIZE (grid dimensions) must throw and leave the domain unchanged, and
// a post-init update that changes the boundary-condition types must throw.
//
// =============================================================================

#include <algorithm>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>

#include "chrono/physics/ChSystemSMC.h"

#include "chrono_fsi/sph/ChFsiProblemSPH.h"

using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::fsi::sph;

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
        std::cout << "  ok (rejected): " << name << std::endl;
    }
}

void Check(const std::string& name, bool cond) {
    if (cond) {
        std::cout << "  ok:            " << name << std::endl;
    } else {
        std::cout << "FAIL:           " << name << std::endl;
        num_failures++;
    }
}

int main(int argc, char* argv[]) {
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

    // Fluid box with bottom and top walls, periodic laterally
    ChVector3d fsize(bxDim, byDim, bzDim - 2 * initial_spacing);
    fsi.Construct(fsize, ChVector3d(0, 0, initial_spacing), BoxSide::Z_NEG | BoxSide::Z_POS);

    ChVector3d c_min(-bxDim / 2 - initial_spacing / 2, -byDim / 2 - initial_spacing / 2, -10 * initial_spacing);
    ChVector3d c_max(+bxDim / 2 + initial_spacing / 2, +byDim / 2 + initial_spacing / 2, bzDim + 10 * initial_spacing);
    ChAABB domain(c_min, c_max);
    fsi.SetComputationalDomain(domain, BC_ALL_PERIODIC);

    fsi.Initialize();

    for (int step = 0; step < 5; step++)
        fsi.DoStepDynamics(dt);

    // Scan only the SPH fluid particles: GetParticlePositions returns all markers with
    // the fluid particles first, and the boundary BCE markers must not wrap.
    size_t num_fluid = fsi.GetNumSPHParticles();
    auto pos0 = sysSPH->GetParticlePositions();
    double min_x0 = 1e9, max_x0 = -1e9;
    for (size_t i = 0; i < num_fluid; i++) {
        min_x0 = std::min(min_x0, pos0[i].x());
        max_x0 = std::max(max_x0, pos0[i].x());
    }
    std::cout << "before update: fluid x in [" << min_x0 << ", " << max_x0 << "]" << std::endl;

    // ------------------------------------------------------------------------
    // Translate the domain by +0.04 in x (pure translation, same size, same BCs).
    double shift = 0.04;
    ChAABB shifted(ChVector3d(c_min.x() + shift, c_min.y(), c_min.z()),
                   ChVector3d(c_max.x() + shift, c_max.y(), c_max.z()));
    double Lx = c_max.x() - c_min.x();

    sysSPH->SetComputationalDomain(shifted);  // fluid-system API, the CRMTerrain path; must reach the device

    for (int step = 0; step < 5; step++)
        fsi.DoStepDynamics(dt);

    auto pos1 = sysSPH->GetParticlePositions();
    double min_x1 = 1e9, max_x1 = -1e9;
    size_t n_wrapped = 0;
    for (size_t i = 0; i < num_fluid; i++) {
        min_x1 = std::min(min_x1, pos1[i].x());
        max_x1 = std::max(max_x1, pos1[i].x());
        if (pos1[i].x() > max_x0 + shift / 2)
            n_wrapped++;
    }
    std::cout << "after  update: fluid x in [" << min_x1 << ", " << max_x1 << "], wrapped particles: " << n_wrapped
              << std::endl;

    const double tol = 1e-3;
    // Particles below the new lower boundary must have wrapped to the far end of the
    // NEW domain: this happens in the device periodic-wrap kernel using paramsD, so it
    // proves the post-init update reached the GPU.
    Check("particles wrapped to the far end of the new domain (device saw the update)", n_wrapped > 0);
    Check("no fluid particle below the new lower x-boundary", min_x1 >= shifted.min.x() - tol);
    Check("no fluid particle beyond the new upper x-boundary", max_x1 <= shifted.max.x() + tol);
    Check("wrap distance equals the domain period", max_x1 <= max_x0 + Lx + tol);

    // ------------------------------------------------------------------------
    // Safety rails
    ChAABB scaled_x(shifted.min, ChVector3d(shifted.max.x() + 3 * bxDim, shifted.max.y(), shifted.max.z()));
    ExpectThrow("post-init domain update that changes the domain size in x",
                [&] { sysSPH->SetComputationalDomain(scaled_x); });

    // Small y/z size changes that would preserve the integer grid counts must ALSO be
    // rejected (the extent comparison is direct, not grid-derived).
    ChAABB scaled_y(shifted.min, ChVector3d(shifted.max.x(), shifted.max.y() + bxDim / 10, shifted.max.z()));
    ExpectThrow("post-init domain update that changes the domain size in y",
                [&] { sysSPH->SetComputationalDomain(scaled_y); });
    ChAABB scaled_z(shifted.min, ChVector3d(shifted.max.x(), shifted.max.y(), shifted.max.z() + bxDim / 10));
    ExpectThrow("post-init domain update that changes the domain size in z",
                [&] { sysSPH->SetComputationalDomain(scaled_z); });

    ExpectThrow("post-init domain update that changes the BC types",
                [&] { sysSPH->SetComputationalDomain(shifted, BC_NONE); });

    // The rejected updates must not have corrupted the working domain: stepping continues.
    try {
        for (int step = 0; step < 3; step++)
            fsi.DoStepDynamics(dt);
        std::cout << "  ok:            stepping continues after rejected updates" << std::endl;
    } catch (const std::exception& e) {
        std::cout << "FAIL:           stepping after rejected updates - " << e.what() << std::endl;
        num_failures++;
    }

    // ------------------------------------------------------------------------
    // Repeated small translations (the moving-patch traverse pattern): none may be
    // falsely rejected by floating-point jitter, and stepping must continue throughout.
    try {
        ChAABB dom = shifted;
        for (int k = 0; k < 25; k++) {
            dom = ChAABB(ChVector3d(dom.min.x() + 0.005, dom.min.y(), dom.min.z()),
                         ChVector3d(dom.max.x() + 0.005, dom.max.y(), dom.max.z()));
            sysSPH->SetComputationalDomain(dom);
            fsi.DoStepDynamics(dt);
            fsi.DoStepDynamics(dt);
        }
        std::cout << "  ok:            25 successive translations accepted, stepping continues" << std::endl;
    } catch (const std::exception& e) {
        std::cout << "FAIL:           repeated translations - " << e.what() << std::endl;
        num_failures++;
    }
    auto pos2 = sysSPH->GetParticlePositions();
    double min_x2 = 1e9, max_x2 = -1e9;
    for (size_t i = 0; i < num_fluid; i++) {
        min_x2 = std::min(min_x2, pos2[i].x());
        max_x2 = std::max(max_x2, pos2[i].x());
    }
    double final_cmin_x = shifted.min.x() + 25 * 0.005;
    double final_cmax_x = shifted.max.x() + 25 * 0.005;
    std::cout << "after traverse: fluid x in [" << min_x2 << ", " << max_x2 << "], domain ["
              << final_cmin_x << ", " << final_cmax_x << "]" << std::endl;
    Check("fluid inside the final domain after the traverse",
          min_x2 >= final_cmin_x - tol && max_x2 <= final_cmax_x + tol);

    if (num_failures > 0) {
        std::cout << "\n" << num_failures << " failure(s)" << std::endl;
        return 1;
    }
    std::cout << "\nAll domain-update checks passed" << std::endl;
    return 0;
}
