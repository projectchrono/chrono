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
// Unit test for the range of the interaction between SPH particles: two particles
// interact if and only if they are closer than the kernel support radius, for every
// kernel type.
//
// The support radius is h_multiplier * h: 2h for the quadratic, cubic spline and
// Wendland kernels, and 3h for the quintic spline. The force loops see only the pairs
// in the neighbor list, so the neighbor search must find every pair within that
// radius. A search limited to 2h for every kernel leaves the quintic spline without
// its pairs between 2h and 3h, where its kernel and gradient are still nonzero.
//
// For each kernel, a pair of particles at rest, denser than the rest density so that
// they repel, is placed 0.85 support radii apart, where it must repel, and 1.05
// support radii apart, where it must not interact at all.
//
// =============================================================================

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "chrono_fsi/sph/ChFsiFluidSystemSPH.h"

using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::fsi::sph;

double initial_spacing = 0.01;
double dt = 2e-4;

int num_failures = 0;

void Check(const std::string& name, bool cond) {
    if (cond)
        std::cout << "  ok:   " << name << std::endl;
    else {
        std::cout << "FAIL:   " << name << std::endl;
        num_failures++;
    }
}

// Run a pair of fluid particles, initially at rest and denser than the rest density so that they
// repel, for a few steps, and return the speed at which they separate. With d0_multiplier = 1 the
// kernel length h equals the spacing. The box is periodic on every axis and 21 spacings long, at least
// 7 support radii for every kernel, so the pair does not reach a periodic image of itself.
//
// The particles are set up identically, so any force they do not exert on each other is the same on
// both, and their separation speed is exactly zero unless they interact.
double RunRepellingPair(KernelType kernel_type, double distance, int num_steps) {
    ChFsiFluidSystemSPH sysSPH;
    sysSPH.SetVerbose(false);
    sysSPH.SetGravitationalAcceleration(ChVector3d(0, 0, 0));

    ChFsiFluidSystemSPH::FluidProperties fluid_props;
    fluid_props.density = 1000;
    fluid_props.viscosity = 1;
    sysSPH.SetCfdSPH(fluid_props);

    ChFsiFluidSystemSPH::SPHParameters sph_params;
    sph_params.integration_scheme = IntegrationScheme::RK2;
    sph_params.num_bce_layers = 3;
    sph_params.initial_spacing = initial_spacing;
    sph_params.d0_multiplier = 1;
    sph_params.kernel_type = kernel_type;
    sph_params.max_velocity = 1.0;
    sph_params.shifting_method = ShiftingMethod::NONE;
    sph_params.density_reinit_steps = 10000;
    sph_params.viscosity_method = ViscosityMethod::LAMINAR;
    sph_params.use_delta_sph = false;
    sph_params.eos_type = EosType::ISOTHERMAL;
    sysSPH.SetSPHParameters(sph_params);
    sysSPH.SetStepSize(dt);

    ChVector3d size = 21 * initial_spacing * ChVector3d(1, 1, 1);
    ChVector3d center = size / 2;
    double density = 1.5 * fluid_props.density;
    sysSPH.AddSPHParticle(center - ChVector3d(distance / 2, 0, 0), density, 0.0, fluid_props.viscosity);
    sysSPH.AddSPHParticle(center + ChVector3d(distance / 2, 0, 0), density, 0.0, fluid_props.viscosity);

    sysSPH.SetComputationalDomain(ChAABB(ChVector3d(0, 0, 0), size), BC_ALL_PERIODIC);
    sysSPH.ChFsiFluidSystem::Initialize();  // the base no-arg overload is hidden
    for (int step = 0; step < num_steps; step++)
        sysSPH.DoStepDynamics(dt);

    // The particle that starts at larger x minus the other
    auto pos = sysSPH.GetParticlePositions();
    auto vel = sysSPH.GetParticleVelocities();
    double speed = vel[1].x() - vel[0].x();
    return (pos[1].x() > pos[0].x()) ? speed : -speed;
}

int main(int argc, char* argv[]) {
    const int steps = 40;

    struct Kernel {
        KernelType type;
        const char* name;
        double support;  // support radius, in multiples of h
    };
    const Kernel kernels[] = {
        {KernelType::QUADRATIC, "quadratic", 2},
        {KernelType::CUBIC_SPLINE, "cubic spline", 2},
        {KernelType::QUINTIC_SPLINE, "quintic spline", 3},
        {KernelType::WENDLAND, "Wendland", 2},
    };

    double h = initial_spacing;  // d0_multiplier = 1
    for (const auto& k : kernels) {
        double speed_within = RunRepellingPair(k.type, 0.85 * k.support * h, steps);
        double speed_beyond = RunRepellingPair(k.type, 1.05 * k.support * h, steps);
        std::cout << k.name << " kernel, support radius " << k.support << "h: separation speed " << speed_within << " m/s at 0.85 support radii, " << speed_beyond << " m/s at 1.05"
                  << std::endl;
        Check(std::string("a pair within the support radius of the ") + k.name + " kernel repels", speed_within > 1e-5);
        Check(std::string("a pair beyond the support radius of the ") + k.name + " kernel does not interact", speed_beyond == 0);
    }

    if (num_failures == 0) {
        std::cout << "\nAll kernel support checks passed" << std::endl;
        return 0;
    }
    std::cout << "\n" << num_failures << " check(s) failed" << std::endl;
    return 1;
}
