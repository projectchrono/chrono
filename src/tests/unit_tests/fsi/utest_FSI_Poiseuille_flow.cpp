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
// Authors: Wei Hu, Radu Serban
// =============================================================================
//
// Unit test for Poiseuille flow.
// Use an analytical solution to verify the numerical simulation.
// Start with a partially developed Poiseuille flow (i.e., at some t_start > 0).
//
// =============================================================================

#include <assert.h>
#include <stdlib.h>
#include <cmath>
#include <valarray>

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/core/ChTimer.h"

#include "chrono_fsi/sph/ChFsiProblemSPH.h"

using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::fsi::sph;

//------------------------------------------------------------------

bool render = true;
bool verbose = true;

// Test tolerances
const double v_tolerance = 5e-3;
const double d_tolerance = 1e-4;

// Dimensions of the computational domain
double bxDim = 0.2;
double byDim = 0.1;
double bzDim = 0.2;

// Start time, step size, number of steps, forcing term, initial spacing
double t_start = 1.0;
double dt = 2e-3;
double num_steps = 500;
double force = 0.05;
double initial_spacing = 0.01;

typedef std::valarray<double> DataVector;

//------------------------------------------------------------------

// Analytical solution for the unsteady plane Poiseuille flow (flow between two parallel plates).
double PoiseuilleAnalytical(double Z, double H, double time, const ChFsiFluidSystemSPH& sysSPH) {
    double nu = sysSPH.GetViscosity() / sysSPH.GetDensity();
    double F = sysSPH.GetBodyForce().x();

    // Adjust plate separation and boundary locations for analytical formula. This accounts for the fact that
    // Chrono::FSI enforces the wall no-slip condition at the mid-point between the last BCE layer and SPH particles
    // closest to the wall.
    H -= initial_spacing;
    Z -= 0.5 * initial_spacing;

    // Truncate infinite series to 50 terms
    double v = 1.0 / (2.0 * nu) * F * Z * (H - Z);
    for (int n = 0; n < 50; n++) {
        v = v - 4.0 * F * std::pow(H, 2) / (nu * std::pow(CH_PI, 3) * std::pow(2 * n + 1, 3)) *
                    std::sin(CH_PI * Z * (2 * n + 1) / H) *
                    std::exp(-pow(2 * n + 1, 2) * std::pow(CH_PI, 2) * nu * time / std::pow(H, 2));
    }

    return v;
}

//------------------------------------------------------------------

// Callback for setting initial SPH particle velocity
class InitialVelocityCallback : public ChFsiProblemSPH::ParticlePropertiesCallback {
  public:
    InitialVelocityCallback(double fluid_height, double time)
        : ParticlePropertiesCallback(), height(fluid_height), time(time) {}

    virtual void set(const ChFsiFluidSystemSPH& sysSPH, const ChVector3d& pos) override {
        double v_x = PoiseuilleAnalytical(pos.z(), height, time, sysSPH);
        p0 = 0;
        rho0 = sysSPH.GetDensity();
        mu0 = sysSPH.GetViscosity();
        v0 = ChVector3d(v_x, 0, 0);
    }

    double height;
    double time;
};

//------------------------------------------------------------------

int main(int argc, char* argv[]) {
    // Create a Chrono system and the FSI problem
    ChSystemSMC sysMBS;
    ChFsiProblemCartesian fsi(initial_spacing, &sysMBS);
    fsi.SetVerbose(verbose);
    auto sysFSI = fsi.GetFsiSystemSPH();
    auto sysSPH = fsi.GetFluidSystemSPH();

    // Set gravitational acceleration
    const ChVector3d gravity(0, 0, 0);
    fsi.SetGravitationalAcceleration(gravity);

    // Set CFD fluid properties
    ChFsiFluidSystemSPH::FluidProperties fluid_props;
    fluid_props.density = 1000;
    fluid_props.viscosity = 1;
    fsi.SetCfdSPH(fluid_props);

    // Set forcing term
    sysSPH->SetBodyForce(ChVector3d(force, 0, 0));

    // Set SPH solution parameters
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
    sph_params.use_consistent_gradient_discretization = true;  // consistent discretization only for laminar viscosity
    sph_params.use_consistent_laplacian_discretization = true;
    fsi.SetSPHParameters(sph_params);

    fsi.SetStepSizeCFD(dt);
    fsi.SetStepsizeMBD(dt);

    // Create SPH material (do not create boundary BCEs)
    // Add box container (only bottom and top walls)
    ChVector3d fsize(bxDim, byDim, bzDim - 2 * initial_spacing);
    fsi.Construct(fsize,                              // length x width x depth
                  ChVector3d(0, 0, initial_spacing),  // position of bottom origin
                  BoxSide::Z_NEG | BoxSide::Z_POS     // bottom and top boundaries
    );

    // Explicitly set computational domain
    ChVector3d c_min(-bxDim / 2 - initial_spacing / 2, -byDim / 2 - initial_spacing / 2, -10 * initial_spacing);
    ChVector3d c_max(+bxDim / 2 + initial_spacing / 2, +byDim / 2 + initial_spacing / 2, bzDim + 10 * initial_spacing);
    fsi.SetComputationalDomain(ChAABB(c_min, c_max), BC_ALL_PERIODIC);

    // Set particle initial velocity
    fsi.RegisterParticlePropertiesCallback(chrono_types::make_shared<InitialVelocityCallback>(bzDim, t_start));

    // Initialize FSI problem
    fsi.Initialize();

    // Simulation loop
    auto num_particles = fsi.GetNumSPHParticles();
    DataVector v(num_particles);
    DataVector va(num_particles);
    DataVector d(num_particles);
    DataVector p(num_particles);

    bool passed = true;
    double time = t_start;

    ChTimer timer;
    timer.start();
    for (int step = 0; step < num_steps; step++) {
        fsi.DoStepDynamics(dt);

        time += dt;

        // Copy data from device to host
        auto pos = sysSPH->GetParticlePositions();        // particle positions
        auto vel = sysSPH->GetParticleVelocities();       // particle velocities
        auto dpv = sysSPH->GetParticleFluidProperties();  // particle properties (density, pressure, viscosity)

        // Extract information in arrays
        for (size_t i = 0; i < num_particles; i++) {
            v[i] = vel[i].x();                                               // velocity in flow direction
            va[i] = PoiseuilleAnalytical(pos[i].z(), bzDim, time, *sysSPH);  // analytical velocity
            d[i] = dpv[i].x();                                               // density at particle location
            p[i] = dpv[i].y();                                               // pressure at particle location
        }

        auto v_max = v.max();
        auto v_min = v.min();
        auto va_max = va.max();
        auto va_min = va.min();

        auto d_max = d.max();
        auto d_min = d.min();

        auto v_err = v - va;
        ////auto v_err_INF = std::abs(v_err).max();
        ////auto v_err_L2 = std::sqrt((v_err * v_err).sum());
        auto v_err_RMS = std::sqrt((v_err * v_err).sum() / v_err.size());

        auto v_rel_err = v_err_RMS / va_max;
        auto d_rel_err = (d_max - d_min) / sysSPH->GetDensity();

        if (verbose) {
            std::cout << "step: " << step << " time: " << time << std::endl;
            std::cout << "  v_min: " << v_min << " v_max: " << v_max << std::endl;
            std::cout << "  va_min: " << va_min << " va_max: " << va_max << std::endl;
            std::cout << "  d_min: " << d_min << " d_max: " << d_max << std::endl;
            std::cout << "    velocity rel err: " << v_rel_err << std::endl;
            std::cout << "    density rel err:  " << d_rel_err << std::endl;
        }

        if (v_rel_err > v_tolerance || d_rel_err > d_tolerance) {
            passed = false;
            break;
        }
    }
    std::cout << "\nSimulated time: " << time - t_start << "\nRun time: " << timer() << std::endl;

    if (!passed) {
        std::cout << "Test failed" << std::endl;
        return 1;
    }

    std::cout << "Test succeeded" << std::endl;
    return 0;
}
