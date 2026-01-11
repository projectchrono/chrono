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
// Author: Radu Serban
// =============================================================================
//
// Benchmark test for measuring performance of the FSI-SPH initialization for
// problems with large number of particles.
//
// =============================================================================

#include <cassert>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <fstream>

#include "chrono/physics/ChSystemNSC.h"
#include "chrono_fsi/sph/ChFsiProblemSPH.h"

using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::fsi::sph;

using std::cout;
using std::cerr;
using std::endl;

// -----------------------------------------------------------------------------

// Dimensions of fluid domain
ChVector3d fsize(2, 2, 2);

// Initial particle spacing
double initial_spacing = 0.005;

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    double step_size = 1e-4;                // MBD and SPH integration step size
    double meta_time_step = 5 * step_size;  // meta-step (communication interval)
    double t_end = 0.1;                     // simulation duration
    int ps_freq = 1;
    bool use_variable_time_step = true;

    // Create the Chrono system and associated collision system
    ChSystemNSC sysMBS;
    sysMBS.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // Create the FSI problem
    ChFsiProblemCartesian fsi(initial_spacing, &sysMBS);
    fsi.SetVerbose(false);
    auto sysFSI = fsi.GetFsiSystemSPH();

    // Set gravitational acceleration
    const ChVector3d gravity(0, 0, -9.8);
    fsi.SetGravitationalAcceleration(gravity);

    // Set integration step size
    fsi.SetStepSizeCFD(step_size);
    fsi.SetStepsizeMBD(step_size);

    // Set CFD fluid properties
    ChFsiFluidSystemSPH::FluidProperties fluid_props;
    fluid_props.density = 1000;
    fluid_props.viscosity = 1;

    fsi.SetCfdSPH(fluid_props);

    // Set SPH solution parameters
    int num_bce_layers = 4;
    ChFsiFluidSystemSPH::SPHParameters sph_params;
    sph_params.integration_scheme = IntegrationScheme::RK2;
    sph_params.num_bce_layers = num_bce_layers;
    sph_params.initial_spacing = initial_spacing;
    sph_params.d0_multiplier = 1;
    sph_params.max_velocity = 4.5;
    sph_params.shifting_method = ShiftingMethod::XSPH;
    sph_params.shifting_xsph_eps = 0.5;
    sph_params.artificial_viscosity = 0.03;
    sph_params.eos_type = EosType::TAIT;
    sph_params.use_consistent_gradient_discretization = false;
    sph_params.use_consistent_laplacian_discretization = false;
    sph_params.num_proximity_search_steps = ps_freq;
    sph_params.use_delta_sph = true;
    sph_params.delta_sph_coefficient = 0.1;
    sph_params.use_variable_time_step = use_variable_time_step;
    sph_params.boundary_method = BoundaryMethod::ADAMI;
    sph_params.viscosity_method = ViscosityMethod::ARTIFICIAL_UNILATERAL;

    fsi.SetSPHParameters(sph_params);

    // Set surface reconstruction parameters
    ChFsiFluidSystemSPH::SplashsurfParameters splashsurf_params;
    splashsurf_params.smoothing_length = 2.0;
    splashsurf_params.cube_size = 0.3;
    splashsurf_params.surface_threshold = 0.6;

    fsi.SetSplashsurfParameters(splashsurf_params);

    // Initialize problem

    ChTimer timer_total;
    ChTimer timer_construct;
    ChTimer timer_init;

    timer_total.start();

    // Create SPH material and boundaries
    timer_construct.start();
    fsi.Construct(fsize,                          // length x width x depth
                  ChVector3d(0, 0, 0),            // position of bottom origin
                  BoxSide::ALL & ~BoxSide::Z_POS  // all boundaries except top
    );
    timer_construct.stop();

    // Initialize FSI problem
    timer_init.start();
    fsi.Initialize();
    timer_init.stop();

    sysFSI->SynchronizeDevice();
    timer_total.stop();

    const auto& domain_aabb = fsi.GetComputationalDomain();

    cout << "ChFsiProblemSPH::Construct():  " << timer_construct() << endl;
    cout << "ChFsiProblemSPH::Initialize(): " << timer_init() << endl;
    cout << "Total initialization time:     " << timer_total() << endl;
    cout << endl;
    cout << "Num. SPH particles: " << fsi.GetNumSPHParticles() << endl;
    cout << "Num. BCE markers:   " << fsi.GetNumBoundaryBCEMarkers() << endl;
    cout << endl;
    cout << "Computational domain: " << endl;
    cout << "   min: " << domain_aabb.min << endl;
    cout << "   max: " << domain_aabb.max << endl;
    cout << endl;

    return 0;
}
