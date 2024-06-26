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
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/core/ChTimer.h"

#include "chrono_fsi/ChSystemFsi.h"

#include "chrono_fsi/visualization/ChFsiVisualization.h"
#ifdef CHRONO_VSG
    #include "chrono_fsi/visualization/ChFsiVisualizationVSG.h"
#endif

using namespace chrono;
using namespace chrono::fsi;

//------------------------------------------------------------------

// Run-time visualization system (OpenGL or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

bool render = false;
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
double init_spacing = 0.01;

typedef std::valarray<double> DataVector;

//------------------------------------------------------------------

// Analytical solution for the unsteady plane Poiseuille flow (flow between two parallel plates).
double PoiseuilleAnalytical(double Z, double L, double time, ChSystemFsi& sysFSI) {
    double nu = sysFSI.GetViscosity() / sysFSI.GetDensity();
    double F = sysFSI.GetBodyForce().x();

    // Adjust plate separation and boundary locations for analytical formula. This accounts for the fact that
    // Chrono::FSI enforces the wall no-slip condition at the mid-point between the last BCE layer and SPH particles
    // closest to the wall.
    double initSpace0 = sysFSI.GetInitialSpacing();
    L -= initSpace0;
    Z -= 0.5 * initSpace0;

    // Truncate infinite series to 50 terms
    double v = 1.0 / (2.0 * nu) * F * Z * (L - Z);
    for (int n = 0; n < 50; n++) {
        v = v - 4.0 * F * pow(L, 2) / (nu * pow(CH_PI, 3) * pow(2 * n + 1, 3)) * sin(CH_PI * Z * (2 * n + 1) / L) *
                    exp(-pow(2 * n + 1, 2) * pow(CH_PI, 2) * nu * time / pow(L, 2));
    }

    return v;
}

// Create the wall boundary and the BCE particles
void CreateSolidPhase(ChSystemSMC& sysMBS, ChSystemFsi& sysFSI) {
    // Create ground body
    auto body = chrono_types::make_shared<ChBody>();
    body->SetFixed(true);
    body->EnableCollision(false);
    sysMBS.AddBody(body);

    // Add BCE particles to the bottom and top wall boundaries
    sysFSI.AddBoxContainerBCE(body,                                           //
                              ChFrame<>(ChVector3d(0, 0, bzDim / 2), QUNIT),  //
                              ChVector3d(bxDim, byDim, bzDim),                //
                              ChVector3i(0, 0, 2));
}

// Create run-time visualization system
std::shared_ptr<ChFsiVisualization> CreateVisSys(ChSystemFsi& sysFSI) {
#if !defined(CHRONO_VSG)
    render = false;
#endif

    std::shared_ptr<ChFsiVisualization> visFSI;

    if (render) {
        // Estimate max particle velocity over entire simulation
        auto v_max = PoiseuilleAnalytical(bzDim / 2, bzDim, t_start + num_steps * dt, sysFSI);

#ifdef CHRONO_VSG
        visFSI = chrono_types::make_shared<ChFsiVisualizationVSG>(&sysFSI);
#endif

        visFSI->SetTitle("Chrono::FSI Poiseuille flow");
        visFSI->AddCamera(ChVector3d(0, -5 * byDim, 0.5 * bzDim), ChVector3d(0, 0, 0.5 * bzDim));
        visFSI->SetCameraMoveScale(1.0f);
        visFSI->EnableFluidMarkers(true);
        visFSI->EnableBoundaryMarkers(true);
        visFSI->SetRenderMode(ChFsiVisualization::RenderMode::SOLID);
        visFSI->SetSPHColorCallback(chrono_types::make_shared<VelocityColorCallback>(0, v_max));
        visFSI->Initialize();
    }

    return visFSI;
}

//------------------------------------------------------------------

int main(int argc, char* argv[]) {
    // Create a physics system and a corresponding FSI system
    ChSystemSMC sysMBS;
    ChSystemFsi sysFSI(&sysMBS);

    // Initialize the parameters using an input JSON file
    std::string myJson = GetChronoDataFile("fsi/input_json/demo_FSI_Poiseuille_flow_Explicit.json");
    sysFSI.ReadParametersFromFile(myJson);

    // Override parameter settings
    sysFSI.SetStepSize(dt);
    sysFSI.SetInitialSpacing(init_spacing);
    sysFSI.SetBodyForce(ChVector3d(force, 0, 0));

    // Reset the domain size to handle periodic boundary condition
    auto initSpace0 = sysFSI.GetInitialSpacing();
    ChVector3d cMin(-bxDim / 2 - initSpace0 / 2, -byDim / 2 - initSpace0 / 2, -10.0 * initSpace0);
    ChVector3d cMax(bxDim / 2 + initSpace0 / 2, byDim / 2 + initSpace0 / 2, bzDim + 10.0 * initSpace0);
    sysFSI.SetBoundaries(cMin, cMax);

    // Create SPH particles for the fluid domain
    ChVector3d boxCenter(0, 0, bzDim / 2);
    ChVector3d boxHalfDim(bxDim / 2, byDim / 2, bzDim / 2 - initSpace0);
    chrono::utils::ChGridSampler<> sampler(initSpace0);
    std::vector<ChVector3d> points = sampler.SampleBox(boxCenter, boxHalfDim);
    auto num_particles = points.size();
    for (const auto& point : points) {
        double v_x = PoiseuilleAnalytical(point.z(), bzDim, t_start, sysFSI);
        sysFSI.AddSPHParticle(point, ChVector3d(v_x, 0.0, 0.0));
    }

    // Create BCE markers for the solid domain
    CreateSolidPhase(sysMBS, sysFSI);

    // Complete construction of the FSI system
    sysFSI.Initialize();

    // Create the run-time visualization system
    auto visFSI = CreateVisSys(sysFSI);

    // Simulation loop
    DataVector v(num_particles);
    DataVector va(num_particles);
    DataVector d(num_particles);
    DataVector p(num_particles);

    bool passed = true;
    double time = t_start;

    ChTimer timer;
    timer.start();
    for (int step = 0; step < num_steps; step++) {
        sysFSI.DoStepDynamics_FSI();

        if (render && !visFSI->Render())
            break;

        time += dt;

        // Copy data from device to host
        auto pos = sysFSI.GetParticlePositions();        // particle positions
        auto vel = sysFSI.GetParticleVelocities();       // particle velocities
        auto dpv = sysFSI.GetParticleFluidProperties();  // particle properties (density, pressure, viscosity)

        // Extract information in arrays
        for (size_t i = 0; i < num_particles; i++) {
            v[i] = vel[i].x();                                              // velocity in flow direction
            va[i] = PoiseuilleAnalytical(pos[i].z(), bzDim, time, sysFSI);  // analytical velocity
            d[i] = dpv[i].x();                                              // density at particle location
            p[i] = dpv[i].y();                                              // pressure at particle location
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
        auto d_rel_err = (d_max - d_min) / sysFSI.GetDensity();

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
