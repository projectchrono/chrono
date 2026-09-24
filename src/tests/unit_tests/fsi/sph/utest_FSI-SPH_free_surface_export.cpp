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
// Unit tests for ChFsiFluidSystemSPH::GetFreeSurfaceFlags(). The flags are
// calculated and stored in sorted marker order, and the accessor scatters them
// back to original order. Two scenarios:
//
// 1. An open-top CFD box, walls on the other five sides. Checks what the test
//    means: only the exposed top layer may be flagged. In particular a particle
//    packed against a wall must NOT be flagged, because BCE markers contribute
//    to the divergence of the position field and so complete the kernel support
//    of the particles next to them. Discriminating: excluding BCE neighbors from
//    that accumulation flags the entire wall-adjacent bottom layer.
//
// 2. A CRM bed with a small active domain. Checks who may be reported: a
//    particle in the extended halo of an active domain exists only to complete
//    the neighborhoods of the particles inside the domain, so its own
//    neighborhood is truncated by construction and its free-surface test always
//    fires. Those must be filtered out, or the exported field shows a spurious
//    free surface along the whole active-domain boundary. Discriminating:
//    without the filter, 504 particles are flagged instead of 28, 456 of them
//    outside the active AABB in a band up to 4 particle spacings wide.
//
// Both scenarios pin free_surface_threshold rather than inheriting the library
// default, since what is under test is which particles get reported and not the
// choice of default.
//
// =============================================================================

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>

#include "chrono/physics/ChSystemSMC.h"

#include "chrono_fsi/sph/ChFsiProblemSPH.h"

using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::fsi::sph;

// A value between the exposed surface layer and the layer below it flags exactly the free surface.
double free_surface_threshold = 2.4;

int num_failures = 0;

void Check(const std::string& name, bool cond) {
    if (cond) {
        std::cout << "  ok:   " << name << std::endl;
    } else {
        std::cout << "  FAIL: " << name << std::endl;
        num_failures++;
    }
}

// -----------------------------------------------------------------------------
// Scenario 1: open-top CFD box
// -----------------------------------------------------------------------------

void TestOpenTopBoxCFD() {
    std::cout << "\nOpen-top CFD box" << std::endl;

    double spacing = 0.01;
    double L = 0.10;
    double H = 0.06;
    double dt = 2e-4;
    int num_steps = 20;

    ChSystemSMC sysMBS;
    ChFsiProblemCartesian fsi(spacing, &sysMBS);
    fsi.SetVerbose(false);
    auto sysSPH = fsi.GetFluidSystemSPH();

    fsi.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));

    ChFsiFluidSystemSPH::FluidProperties fluid_props;
    fluid_props.density = 1000;
    fluid_props.viscosity = 1;
    fsi.SetCfdSPH(fluid_props);

    ChFsiFluidSystemSPH::SPHParameters sph_params;
    sph_params.integration_scheme = IntegrationScheme::RK2;
    sph_params.num_bce_layers = 3;
    sph_params.initial_spacing = spacing;
    sph_params.d0_multiplier = 1.2;
    sph_params.max_velocity = 1.0;
    sph_params.shifting_method = ShiftingMethod::XSPH;
    sph_params.viscosity_method = ViscosityMethod::LAMINAR;
    sph_params.eos_type = EosType::ISOTHERMAL;
    sph_params.free_surface_threshold = free_surface_threshold;
    fsi.SetSPHParameters(sph_params);

    fsi.SetStepSizeCFD(dt);
    fsi.SetStepsizeMBD(dt);

    // Walls on all sides except the top: the omitted Z_POS side is the free surface
    fsi.Construct(ChVector3d(L, L, H), ChVector3d(0, 0, 0), BoxSide::Z_NEG | BoxSide::X_NEG | BoxSide::X_POS | BoxSide::Y_NEG | BoxSide::Y_POS);

    fsi.Initialize();

    for (int step = 0; step < num_steps; step++)
        fsi.DoStepDynamics(dt);

    // GetParticlePositions and GetFreeSurfaceFlags both return all markers in the same order, with
    // the SPH particles first.
    size_t num_sph = fsi.GetNumSPHParticles();
    auto pos = sysSPH->GetParticlePositions();
    auto flags = sysSPH->GetFreeSurfaceFlags();

    Check("flags array is sized as the marker arrays", flags.size() == pos.size());
    if (flags.size() != pos.size())
        return;

    double zmin = 1e9;
    double zmax = -1e9;
    for (size_t i = 0; i < num_sph; i++) {
        zmin = std::min(zmin, pos[i].z());
        zmax = std::max(zmax, pos[i].z());
    }

    int num_flagged = 0;
    int num_flagged_bce = 0;
    double flagged_zmin = 1e9;
    for (size_t i = 0; i < flags.size(); i++) {
        if (!flags[i])
            continue;
        if (i >= num_sph) {
            num_flagged_bce++;
            continue;
        }
        num_flagged++;
        flagged_zmin = std::min(flagged_zmin, pos[i].z());
    }

    // The bottom layer is packed against a wall, so BCE markers complete its kernel support
    int num_bottom = 0;
    int num_bottom_flagged = 0;
    for (size_t i = 0; i < num_sph; i++) {
        if (pos[i].z() >= zmin + 0.5 * spacing)
            continue;
        num_bottom++;
        if (flags[i])
            num_bottom_flagged++;
    }

    std::cout << "  SPH particles: " << num_sph << ", markers: " << flags.size() << std::endl;
    std::cout << "  fluid z range: [" << zmin << ", " << zmax << "]" << std::endl;
    std::cout << "  flagged: " << num_flagged << " SPH particles, " << num_flagged_bce << " BCE markers" << std::endl;
    std::cout << "  bottom layer: " << num_bottom << " particles, " << num_bottom_flagged << " flagged" << std::endl;

    // Guards against every check below passing on an all-zero field
    Check("the free surface is flagged", num_flagged > 0);

    Check("no BCE marker is flagged", num_flagged_bce == 0);

    // Only the exposed layer is a free surface; the bulk below it has complete support
    Check("all flagged particles lie within two layers of the top surface", flagged_zmin > zmax - 2.5 * spacing);

    // The property BCE markers are there to provide: a particle next to a solid is not a surface
    Check("no particle in the wall-adjacent bottom layer is flagged", num_bottom_flagged == 0);

    // A sanity check on the setup: the surface is one layer of a box that is several layers deep
    Check("flagged count is a small fraction of the fluid", num_flagged < 0.4 * (double)num_sph);
}

// -----------------------------------------------------------------------------
// Scenario 2: CRM bed with a small active domain
// -----------------------------------------------------------------------------

void TestActiveDomainCRM() {
    std::cout << "\nCRM bed with a small active domain" << std::endl;

    double spacing = 0.02;
    double L = 0.6;
    double H = 0.10;
    double body_radius = 0.02;
    double body_height = 0.03;
    double ad_size = 0.12;
    double dt = 1e-4;
    int num_steps = 20;

    ChSystemSMC sysMBS;
    ChFsiProblemCartesian fsi(spacing, &sysMBS);
    fsi.SetVerbose(false);
    auto sysSPH = fsi.GetFluidSystemSPH();

    fsi.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));

    ChFsiFluidSystemSPH::SoilProperties mat_props;
    mat_props.density = 1700;
    mat_props.Young_modulus = 1e6;
    mat_props.Poisson_ratio = 0.3;
    mat_props.rheology_model = RheologyCRM::MU_OF_I;
    mat_props.mu_fric_s = 0.7;
    mat_props.mu_fric_2 = 0.7;
    mat_props.average_diam = 0.005;
    mat_props.cohesion_coeff = 0;
    fsi.SetCrmSPH(mat_props);

    ChFsiFluidSystemSPH::SPHParameters sph_params;
    sph_params.integration_scheme = IntegrationScheme::RK2;
    sph_params.num_bce_layers = 3;
    sph_params.initial_spacing = spacing;
    sph_params.d0_multiplier = 1.2;
    sph_params.shifting_method = ShiftingMethod::PPST_XSPH;
    sph_params.viscosity_method = ViscosityMethod::ARTIFICIAL_BILATERAL;
    sph_params.free_surface_threshold = free_surface_threshold;
    fsi.SetSPHParameters(sph_params);

    fsi.SetStepSizeCFD(dt);
    fsi.SetStepsizeMBD(dt);

    // Rigid sphere hovering above the middle of the bed
    auto body = chrono_types::make_shared<ChBody>();
    body->SetPos(ChVector3d(0, 0, H + body_height));
    body->SetMass(1.0);
    body->SetInertiaXX(ChVector3d(0.01, 0.01, 0.01));
    sysMBS.AddBody(body);
    fsi.AddRigidBodySphere(body, ChVector3d(0, 0, 0), body_radius, false);

    // Restrict the active domain to a small box around the sphere
    fsi.SetActiveDomain(ChVector3d(ad_size, ad_size, ad_size));

    // Bed with an open top: the omitted Z_POS side is the free surface
    fsi.Construct(ChVector3d(L, L, H), ChVector3d(0, 0, 0), BoxSide::Z_NEG | BoxSide::X_NEG | BoxSide::X_POS | BoxSide::Y_NEG | BoxSide::Y_POS);

    fsi.Initialize();

    for (int step = 0; step < num_steps; step++)
        fsi.DoStepDynamics(dt);

    size_t num_sph = fsi.GetNumSPHParticles();
    auto pos = sysSPH->GetParticlePositions();
    auto flags = sysSPH->GetFreeSurfaceFlags();

    Check("flags array is sized as the marker arrays", flags.size() == pos.size());
    if (flags.size() != pos.size())
        return;

    // The sphere barely moves over num_steps, but read its actual position rather than assuming
    ChVector3d bpos = body->GetPos();
    double half = 0.5 * ad_size;

    // Generous next to the signal being guarded against, which is a band several particle spacings
    // wide, yet tight enough to stay well inside it
    double tol = 0.5 * spacing;

    int num_flagged = 0;
    int num_flagged_bce = 0;
    int num_outside = 0;
    double max_overshoot = 0;
    double flagged_zmin = 1e9;

    for (size_t i = 0; i < flags.size(); i++) {
        if (!flags[i])
            continue;
        if (i >= num_sph) {
            num_flagged_bce++;
            continue;
        }

        num_flagged++;
        flagged_zmin = std::min(flagged_zmin, pos[i].z());

        double dx = std::abs(pos[i].x() - bpos.x()) - half;
        double dy = std::abs(pos[i].y() - bpos.y()) - half;
        double dz = std::abs(pos[i].z() - bpos.z()) - half;
        double overshoot = std::max(dx, std::max(dy, dz));
        if (overshoot > tol) {
            num_outside++;
            max_overshoot = std::max(max_overshoot, overshoot);
        }
    }

    std::cout << "  SPH particles: " << num_sph << ", markers: " << flags.size() << std::endl;
    std::cout << "  active domain: half-extent " << half << " about (" << bpos.x() << ", " << bpos.y() << ", " << bpos.z() << ")" << std::endl;
    std::cout << "  flagged: " << num_flagged << " SPH particles, " << num_flagged_bce << " BCE markers" << std::endl;
    std::cout << "  flagged outside the active AABB: " << num_outside << " (max overshoot " << max_overshoot << " m = " << (max_overshoot / spacing) << " spacings)" << std::endl;

    // Guards against the checks below passing on an all-zero field
    Check("the free surface inside the active domain is flagged", num_flagged > 0);

    // The actual regression: particles in the extended halo must not be reported
    Check("no flagged particle lies outside the active AABB", num_outside == 0);

    // A flag on a BCE marker would mean the scatter or the marker-type filter is wrong
    Check("no BCE marker is flagged", num_flagged_bce == 0);

    // Only the bed surface should be flagged, not particles buried in the bulk
    Check("flagged particles lie near the bed surface", num_flagged == 0 || flagged_zmin > H - 3 * spacing);

    // A sanity check on the setup rather than on the filter: if the active domain ever stopped
    // restricting the flagged set, most of the bed surface would be reported and the checks above
    // would be measuring something other than what this test is about
    Check("flagged count is a small fraction of the bed", num_flagged < 0.2 * (double)num_sph);
}

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    TestOpenTopBoxCFD();
    TestActiveDomainCRM();

    if (num_failures > 0) {
        std::cout << "\n" << num_failures << " failure(s)" << std::endl;
        return 1;
    }
    std::cout << "\nAll free-surface export checks passed" << std::endl;
    return 0;
}
