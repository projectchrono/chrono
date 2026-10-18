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
// Unit test for the handling of particles outside the computational domain.
//
// A particle further than 40h outside the domain is reported by calcHashD as an
// error, which terminates the run. Rigid-body (BCE) markers are meant to be
// exempt from that guard so a body can be initialized outside the domain and
// travel across its boundaries; fluid particles are not exempt. Separately, the
// bin index of any particle must reduce into the cell grid, since the resulting
// hash is used to index the per-cell arrays.
//
// Case 1: a fixed rigid body far outside a non-periodic domain must not
//         terminate the run.
// Case 2: fluid particles far outside the domain must still be reported, so the
//         exemption cannot be mistaken for removing the guard. See the comment in the
//         case body for why the fluid is a wide slab rather than a few escaped
//         particles: only the slab is reported under both HIP and CUDA.
// Case 3: with periodic boundaries, a rigid body one period outside the domain is
//         imaged into it and must act on the fluid, and a body TEN periods out must act
//         on it identically, because both are the same image. Both are asserted. The
//         second assertion is the regression test for the minimum-image shift being
//         total: a single-period shift binned a marker more than one period out into its
//         image cell yet measured it a whole box away, so it silently failed to
//         interact, and rigid-body markers reach that state because
//         ApplyPeriodicBoundary*_D does not wrap them. On unfixed code the ten-period
//         deviation equals the body-versus-no-body deviation, which is the signature of
//         the far body doing nothing at all; on fixed code it falls to round-off.
//
//         Note on what this case does NOT cover: the bin-index reduction. Measured
//         against a library with that reduction left un-total, where a diagnostic
//         confirms hundreds of out-of-range hashes, an earlier form of this case still
//         passed with zero deviation, because the resulting out-of-bounds writes do not
//         perturb observable output. The reduction cannot be checked through the public
//         API; it was verified with a temporary in-kernel diagnostic instead.
// Case 4: with NO periodic boundary, a rigid body outside the domain and further from
//         every fluid marker than the kernel support radius must exert no force at all,
//         so the fluid must match a run with no body. (A body just outside an open
//         boundary but within support legitimately DOES interact; that is not this
//         case.) This is what the minimum-image shift used to break: the bin index of a
//         marker outside a non-periodic axis reduces into that axis's edge bin, making
//         it a neighbor candidate of the markers genuinely there, and a shift of one box
//         length applied without regard to periodicity then turned a separation of about
//         one box length into about zero. A body one box length out therefore pushed on
//         fluid it was nowhere near, while the same body four box lengths out, where one
//         shift cannot reach, did nothing at all.
// Case 5: MIXED periodicity, one periodic axis and two non-periodic axes in the same run.
//         Cases 3 and 4 are each uniform, every axis periodic or none of them. But the shift is
//         applied per axis, so the configuration that mixes the two paths inside a single
//         distance evaluation is covered by neither, and it is also the common production one: a
//         channel periodic along the flow direction with solid walls across it. This case is
//         case 3 with two of its three axes made non-periodic, so the only differences are the
//         boundary-condition flag and the body's z offset. It asserts both halves at once: a
//         body ten periods out along the PERIODIC x axis still acts on the fluid exactly as one
//         period out, while a body displaced a whole box length along the NON-periodic z axis
//         exerts no force at all. Unfixed code fails one or the other whichever way it errs,
//         because folding every axis unconditionally invents a force across z, and folding at
//         most one period loses the interaction across x. See the comment in the case body for
//         why the z displacement is tried at one period out as well as at ten: at ten, the two
//         defects mask each other and the phantom cannot appear.
//
// =============================================================================

#include <cmath>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/utils/ChBodyGeometry.h"

#include "chrono_fsi/sph/ChFsiProblemSPH.h"

using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::fsi::sph;

double initial_spacing = 0.01;
double dt = 2e-4;

int num_failures = 0;

void Check(const std::string& name, bool cond) {
    if (cond)
        std::cout << "  ok:            " << name << std::endl;
    else {
        std::cout << "FAIL:           " << name << std::endl;
        num_failures++;
    }
}

void ExpectNoThrow(const std::string& name, std::function<void()> f) {
    try {
        f();
        std::cout << "  ok:            " << name << std::endl;
    } catch (const std::exception& e) {
        std::cout << "FAIL (threw):   " << name << ": " << e.what() << std::endl;
        num_failures++;
    }
}

void ExpectThrow(const std::string& name, std::function<void()> f) {
    try {
        f();
        std::cout << "FAIL (no throw): " << name << std::endl;
        num_failures++;
    } catch (const std::exception&) {
        std::cout << "  ok (reported): " << name << std::endl;
    }
}

void SetCommonParameters(ChFsiProblemCartesian& fsi, const ChVector3d& gravity) {
    fsi.SetVerbose(false);
    fsi.SetGravitationalAcceleration(gravity);

    ChFsiFluidSystemSPH::FluidProperties fluid_props;
    fluid_props.density = 1000;
    fluid_props.viscosity = 1;
    fsi.SetCfdSPH(fluid_props);

    ChFsiFluidSystemSPH::SPHParameters sph_params;
    sph_params.integration_scheme = IntegrationScheme::RK2;
    sph_params.num_bce_layers = 3;
    sph_params.initial_spacing = initial_spacing;
    // h = d0_multiplier * initial_spacing, so this makes h equal the spacing and the guard margin
    // 40h a round 0.4 m, which keeps the geometry in these cases checkable by hand. It is a
    // deliberate choice for test arithmetic, not a production value: the default is 1.2
    // (ChFsiFluidSystemSPH.h), and 1 is the lowest value the library accepts without warning, since
    // CheckSPHParameters warns below 1 and, for the cubic spline kernel, above 1.5.
    sph_params.d0_multiplier = 1;
    sph_params.max_velocity = 1.0;
    sph_params.shifting_method = ShiftingMethod::NONE;
    sph_params.density_reinit_steps = 10000;
    sph_params.viscosity_method = ViscosityMethod::LAMINAR;
    sph_params.use_delta_sph = false;
    sph_params.eos_type = EosType::ISOTHERMAL;
    fsi.SetSPHParameters(sph_params);

    fsi.SetStepSizeCFD(dt);
    fsi.SetStepsizeMBD(dt);
}

// A fixed box body with BCE markers, placed at the given position.
void AddFixedBody(ChSystemSMC& sysMBS, ChFsiProblemCartesian& fsi, const ChVector3d& pos) {
    ChContactMaterialData cmat;
    auto geometry = chrono_types::make_shared<utils::ChBodyGeometry>();
    geometry->materials.push_back(cmat);
    geometry->coll_boxes.push_back(
        utils::ChBodyGeometry::BoxShape(ChVector3d(0, 0, 0), QUNIT, ChVector3d(0.04, 0.04, 0.04), 0));
    auto body = chrono_types::make_shared<ChBody>();
    body->SetPos(pos);
    body->SetRot(QUNIT);
    body->SetFixed(true);
    sysMBS.AddBody(body);
    fsi.AddRigidBody(body, geometry, false);
}

// -----------------------------------------------------------------------------

void TestRigidBodyOutsideDomain() {
    std::cout << "Case 1: rigid body far outside a non-periodic domain" << std::endl;

    double bxDim = 0.1, byDim = 0.1, bzDim = 0.1;

    ChSystemSMC sysMBS;
    ChFsiProblemCartesian fsi(initial_spacing, &sysMBS);
    SetCommonParameters(fsi, ChVector3d(0, 0, 0));

    ChVector3d fsize(bxDim, byDim, bzDim - 2 * initial_spacing);
    fsi.Construct(fsize, ChVector3d(0, 0, initial_spacing), BoxSide::Z_NEG | BoxSide::Z_POS);

    // With d0_multiplier = 1 the kernel length h equals the initial spacing, so the
    // guard margin 40h is 0.4 m; the body's markers sit about 1.5 m beyond it.
    double body_x = 2.0;
    AddFixedBody(sysMBS, fsi, ChVector3d(body_x, 0, 0));

    ChVector3d c_min(-bxDim / 2 - initial_spacing / 2, -byDim / 2 - initial_spacing / 2, -10 * initial_spacing);
    ChVector3d c_max(+bxDim / 2 + initial_spacing / 2, +byDim / 2 + initial_spacing / 2, bzDim + 10 * initial_spacing);
    fsi.SetComputationalDomain(ChAABB(c_min, c_max));

    Check("test setup places the body beyond the guard margin", body_x - 0.04 > c_max.x() + 40 * initial_spacing);

    ExpectNoThrow("rigid-body markers outside the domain do not terminate the run", [&] {
        fsi.Initialize();
        for (int step = 0; step < 3; step++)
            fsi.DoStepDynamics(dt);
    });
}

// -----------------------------------------------------------------------------
// Driven on a standalone fluid system so the report is a catchable exception: in a
// coupled step the multibody thread is still joinable while the stack unwinds and
// the process terminates instead of propagating.
void TestFluidOutsideDomain() {
    std::cout << "Case 2: fluid particles far outside the domain" << std::endl;

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
    // h = d0_multiplier * initial_spacing, so this makes h equal the spacing and the guard margin
    // 40h a round 0.4 m, which keeps the geometry in these cases checkable by hand. It is a
    // deliberate choice for test arithmetic, not a production value: the default is 1.2
    // (ChFsiFluidSystemSPH.h), and 1 is the lowest value the library accepts without warning, since
    // CheckSPHParameters warns below 1 and, for the cubic spline kernel, above 1.5.
    sph_params.d0_multiplier = 1;
    sph_params.max_velocity = 1.0;
    sph_params.shifting_method = ShiftingMethod::NONE;
    sph_params.density_reinit_steps = 10000;
    sph_params.viscosity_method = ViscosityMethod::LAMINAR;
    sph_params.use_delta_sph = false;
    sph_params.eos_type = EosType::ISOTHERMAL;
    sysSPH.SetSPHParameters(sph_params);
    sysSPH.SetStepSize(dt);

    // A slab of fluid reaching far past the guard margin on both sides. Measured note, because it
    // is not obvious: whether the guard reports out-of-domain fluid depends on the configuration
    // and on the backend. A slab reaching 0.6 m is reported under HIP but NOT under CUDA, and that
    // is true of unpatched code too, so it is a pre-existing divergence rather than anything this
    // change causes. A handful of individually escaped particles is reported under neither. The
    // slab below, reaching 6.0 m, is reported under both, which is why the test uses it.
    double half_length = 6.0;
    int nx = (int)(half_length / initial_spacing);
    for (int i = -nx; i <= nx; i++)
        for (int j = -1; j <= 1; j++)
            for (int k = 0; k <= 2; k++)
                sysSPH.AddSPHParticle(ChVector3d(i * initial_spacing, j * initial_spacing, k * initial_spacing),
                                      fluid_props.density, 0.0, fluid_props.viscosity);

    ChVector3d c_min(-0.06, -0.05, -0.05);
    ChVector3d c_max(+0.06, +0.05, +0.05);
    sysSPH.SetComputationalDomain(ChAABB(c_min, c_max));

    Check("test setup places fluid beyond the guard margin", half_length > c_max.x() + 40 * initial_spacing);

    ExpectThrow("fluid particles outside the domain are still reported", [&] {
        sysSPH.ChFsiFluidSystem::Initialize();  // the base no-arg overload is hidden
        sysSPH.DoStepDynamics(dt);
    });
}

// -----------------------------------------------------------------------------
// Run a FULLY PERIODIC fluid box and return the FLUID state only. GetParticlePositions returns
// every marker with the SPH particles first, so the body's own BCE markers, whose positions
// differ between runs by construction, must be excluded.
//
// The body, when present, is placed a whole number of periods away from one fixed image
// position, which sits just inside the +x face and therefore within kernel support of the fluid
// there. So every placement describes the same physical configuration and must give the same
// fluid, and the image must genuinely act on the fluid.
void RunPeriodicWithImagedBody(bool with_body,
                               double num_periods,
                               std::vector<ChVector3d>& pos_out,
                               int num_steps) {
    double bxDim = 0.1, byDim = 0.1, bzDim = 0.1;

    ChSystemSMC sysMBS;
    ChFsiProblemCartesian fsi(initial_spacing, &sysMBS);
    SetCommonParameters(fsi, ChVector3d(0, 0, -9.81));

    ChVector3d fsize(bxDim, byDim, bzDim - 2 * initial_spacing);
    fsi.Construct(fsize, ChVector3d(0, 0, initial_spacing), BoxSide::Z_NEG | BoxSide::Z_POS);

    ChVector3d c_min(-bxDim / 2 - initial_spacing / 2, -byDim / 2 - initial_spacing / 2, -initial_spacing / 2);
    ChVector3d c_max(+bxDim / 2 + initial_spacing / 2, +byDim / 2 + initial_spacing / 2, bzDim + initial_spacing / 2);
    double period = c_max.x() - c_min.x();

    if (with_body) {
        double image_x = c_max.x() - initial_spacing;
        AddFixedBody(sysMBS, fsi, ChVector3d(image_x + num_periods * period, 0, bzDim / 2));
    }

    // BC_ALL_PERIODIC is periodic on all three axes, so every axis exercises the minimum-image
    // shift and none exercises the non-periodic edge clamp. Case 4 is the mirror of this.
    fsi.SetComputationalDomain(ChAABB(c_min, c_max), BC_ALL_PERIODIC);

    fsi.Initialize();
    for (int step = 0; step < num_steps; step++)
        fsi.DoStepDynamics(dt);

    size_t num_fluid = fsi.GetNumSPHParticles();
    auto all_pos = fsi.GetFluidSystemSPH()->GetParticlePositions();
    pos_out.assign(all_pos.begin(), all_pos.begin() + num_fluid);
}

void TestPeriodicImagedBodyActs() {
    std::cout << "Case 3: periodic domain, an imaged body must act on the fluid" << std::endl;

    const int steps = 20;
    std::vector<ChVector3d> pos_none, pos_near, pos_far;
    ExpectNoThrow("periodic reference run with no body completes",
                  [&] { RunPeriodicWithImagedBody(false, 0.0, pos_none, steps); });
    ExpectNoThrow("periodic run with the body one period out completes",
                  [&] { RunPeriodicWithImagedBody(true, 1.0, pos_near, steps); });
    ExpectNoThrow("periodic run with the body ten periods out completes",
                  [&] { RunPeriodicWithImagedBody(true, 10.0, pos_far, steps); });

    Check("the three periodic runs produced comparable fluid sets",
          !pos_none.empty() && pos_none.size() == pos_near.size() && pos_near.size() == pos_far.size());
    if (pos_none.empty() || pos_none.size() != pos_near.size() || pos_near.size() != pos_far.size())
        return;

    double dev_near_far = 0, dev_none_near = 0;
    for (size_t i = 0; i < pos_none.size(); i++) {
        dev_near_far = std::max(dev_near_far, (pos_near[i] - pos_far[i]).Length());
        dev_none_near = std::max(dev_none_near, (pos_near[i] - pos_none[i]).Length());
    }

    std::cout << "    compared " << pos_none.size() << " fluid particles" << std::endl;
    std::cout << "      body present vs absent:            max deviation " << dev_none_near << " m" << std::endl;
    std::cout << "      one period out vs ten periods out: max deviation " << dev_near_far << " m"
              << std::endl;

    // A body one period outside a periodic domain is imaged into it, so it must move the fluid.
    Check("the body's periodic image does act on the fluid", dev_none_near > 1e-6);

    // Ten periods out is the SAME image, so the fluid must be the same to round-off. The threshold
    // sits between two measured quantities and is not arbitrary: on fixed code this deviation is
    // 7.5e-09 m in single precision and 1.4e-17 m in double, while on single-shift code it is
    // 4.3e-05 m, the same value as the body-versus-no-body deviation because the far body then does
    // nothing at all. So 1e-06 m is about 130x above the round-off it must tolerate and about 43x
    // below the failure it must catch.
    Check("a body ten periods out acts on the fluid the same as one period out", dev_near_far < 1e-6);
}

// -----------------------------------------------------------------------------
// Run a NON-PERIODIC fluid box, optionally with a fixed body placed a whole number of box
// lengths beyond the upper face along one axis, and return the FLUID state only.
//
// The box is long in x (about 0.51 m) and short in z (about 0.11 m) relative to the 0.4 m
// guard margin, which makes the two axes probe different things: a body one box length out
// along x is beyond the margin, so it reaches the hashing code only once the guard's
// rigid-body exemption works on x, whereas along z it is inside the margin and reaches the
// hashing code regardless.
void RunNonPeriodicWithOutsideBody(bool with_body,
                                   char axis,
                                   double num_box_lengths,
                                   std::vector<ChVector3d>& pos_out,
                                   int num_steps) {
    double bxDim = 0.5, byDim = 0.1, bzDim = 0.1;

    ChSystemSMC sysMBS;
    ChFsiProblemCartesian fsi(initial_spacing, &sysMBS);
    SetCommonParameters(fsi, ChVector3d(0, 0, -9.81));

    ChVector3d fsize(bxDim, byDim, bzDim - 2 * initial_spacing);
    fsi.Construct(fsize, ChVector3d(0, 0, initial_spacing), BoxSide::Z_NEG | BoxSide::Z_POS);

    ChVector3d c_min(-bxDim / 2 - initial_spacing / 2, -byDim / 2 - initial_spacing / 2, -initial_spacing / 2);
    ChVector3d c_max(+bxDim / 2 + initial_spacing / 2, +byDim / 2 + initial_spacing / 2, bzDim + initial_spacing / 2);

    if (with_body) {
        ChVector3d box = c_max - c_min;
        ChVector3d pos = (axis == 'z') ? ChVector3d(0, 0, c_max.z() + num_box_lengths * box.z())
                                       : ChVector3d(c_max.x() + num_box_lengths * box.x(), 0, bzDim / 2);
        AddFixedBody(sysMBS, fsi, pos);
    }

    fsi.SetComputationalDomain(ChAABB(c_min, c_max));  // no periodic axis

    fsi.Initialize();
    for (int step = 0; step < num_steps; step++)
        fsi.DoStepDynamics(dt);

    size_t num_fluid = fsi.GetNumSPHParticles();
    auto all_pos = fsi.GetFluidSystemSPH()->GetParticlePositions();
    pos_out.assign(all_pos.begin(), all_pos.begin() + num_fluid);
}

void TestNonPeriodicOutsideBodyExertsNoForce() {
    std::cout << "Case 4: non-periodic domain, a body beyond kernel support must exert no force" << std::endl;

    const int steps = 20;

    std::vector<ChVector3d> ref;
    ExpectNoThrow("reference run with no body at all completes",
                  [&] { RunNonPeriodicWithOutsideBody(false, 'x', 0.0, ref, steps); });
    Check("the reference run produced fluid", !ref.empty());
    if (ref.empty())
        return;

    struct Placement {
        char axis;
        double num_box_lengths;
        const char* what;
    };
    const Placement placements[] = {
        {'z', 1.0, "one box length above the domain (inside the guard margin)"},
        {'x', 1.0, "one box length beyond the domain in x (outside the guard margin)"},
        {'x', 4.0, "four box lengths beyond the domain in x"},
    };

    for (const auto& pl : placements) {
        std::string label = std::string("body ") + pl.what;
        std::vector<ChVector3d> pos;
        ExpectNoThrow(label + ": run completes",
                      [&] { RunNonPeriodicWithOutsideBody(true, pl.axis, pl.num_box_lengths, pos, steps); });
        if (pos.size() != ref.size()) {
            Check(label + ": fluid set comparable to the reference", false);
            continue;
        }
        double max_dev = 0;
        for (size_t i = 0; i < ref.size(); i++)
            max_dev = std::max(max_dev, (pos[i] - ref[i]).Length());
        std::cout << "    " << label << std::endl;
        std::cout << "      max deviation from the no-body reference: " << max_dev << " m" << std::endl;
        // Every placement here is further from the fluid than the kernel support radius, so
        // the body contributes exactly zero kernel weight and the fluid is not merely close to
        // the reference, it is identical. The tolerance is here only so that a future change to
        // neighbor ordering cannot fail the test at round-off.
        Check(label + ": fluid is unchanged by the body", max_dev < 1e-10);
    }
}

// -----------------------------------------------------------------------------
// Run a box periodic in x ONLY and return the FLUID state, so one distance evaluation mixes a
// periodic axis with two non-periodic ones. Geometry, fluid and body are those of case 3; the
// differences are the boundary-condition flag and that the body may also be displaced along the
// non-periodic z axis.
void RunMixedPeriodicX(bool with_body,
                       double num_periods_x,
                       double num_box_lengths_z,
                       std::vector<ChVector3d>& pos_out,
                       int num_steps) {
    double bxDim = 0.1, byDim = 0.1, bzDim = 0.1;

    ChSystemSMC sysMBS;
    ChFsiProblemCartesian fsi(initial_spacing, &sysMBS);
    SetCommonParameters(fsi, ChVector3d(0, 0, -9.81));

    ChVector3d fsize(bxDim, byDim, bzDim - 2 * initial_spacing);
    fsi.Construct(fsize, ChVector3d(0, 0, initial_spacing), BoxSide::Z_NEG | BoxSide::Z_POS);

    ChVector3d c_min(-bxDim / 2 - initial_spacing / 2, -byDim / 2 - initial_spacing / 2, -initial_spacing / 2);
    ChVector3d c_max(+bxDim / 2 + initial_spacing / 2, +byDim / 2 + initial_spacing / 2, bzDim + initial_spacing / 2);
    double period_x = c_max.x() - c_min.x();
    double box_z = c_max.z() - c_min.z();

    if (with_body) {
        // In x, the same image position as case 3: just inside the +x face and at mid-height, which
        // puts the image within kernel support of the fluid there. In z, whole box lengths beyond the
        // +z FACE, measured the way case 4 measures them rather than from mid-height, so that every
        // marker of the body is genuinely outside the axis. That distinction decides whether the
        // third assertion below can fail at all: measured from mid-height, one box length leaves part
        // of the body inside the domain and two box lengths puts it beyond the reach of a single
        // fold, so a run that folds the axis unconditionally passes anyway.
        double image_x = c_max.x() - initial_spacing;
        double body_z = (num_box_lengths_z > 0) ? c_max.z() + num_box_lengths_z * box_z : bzDim / 2;
        AddFixedBody(sysMBS, fsi, ChVector3d(image_x + num_periods_x * period_x, 0, body_z));
    }

    // x periodic, y and z not: this is the mixed case. Inside one distance evaluation the x
    // component takes the shift path while the y and z components must take the early exit.
    fsi.SetComputationalDomain(ChAABB(c_min, c_max), BC_X_PERIODIC);

    fsi.Initialize();
    for (int step = 0; step < num_steps; step++)
        fsi.DoStepDynamics(dt);

    size_t num_fluid = fsi.GetNumSPHParticles();
    auto all_pos = fsi.GetFluidSystemSPH()->GetParticlePositions();
    pos_out.assign(all_pos.begin(), all_pos.begin() + num_fluid);
}

void TestMixedPeriodicity() {
    std::cout << "Case 5: mixed periodicity, x periodic while y and z are not" << std::endl;

    const int steps = 20;
    std::vector<ChVector3d> ref, near_in, far_in, near_out, far_out;
    ExpectNoThrow("mixed-periodicity reference run with no body completes",
                  [&] { RunMixedPeriodicX(false, 0.0, 0.0, ref, steps); });
    ExpectNoThrow("body one period out along the periodic axis completes",
                  [&] { RunMixedPeriodicX(true, 1.0, 0.0, near_in, steps); });
    ExpectNoThrow("body ten periods out along the periodic axis completes",
                  [&] { RunMixedPeriodicX(true, 10.0, 0.0, far_in, steps); });
    ExpectNoThrow("body one period out in x and one box length beyond the non-periodic +z face completes",
                  [&] { RunMixedPeriodicX(true, 1.0, 1.0, near_out, steps); });
    ExpectNoThrow("body ten periods out in x and one box length beyond the non-periodic +z face completes",
                  [&] { RunMixedPeriodicX(true, 10.0, 1.0, far_out, steps); });

    bool comparable = !ref.empty() && ref.size() == near_in.size() && ref.size() == far_in.size() &&
                      ref.size() == near_out.size() && ref.size() == far_out.size();
    Check("the five mixed-periodicity runs produced comparable fluid sets", comparable);
    if (!comparable)
        return;

    double dev_acts = 0, dev_near_far = 0, dev_near_out = 0, dev_far_out = 0;
    for (size_t i = 0; i < ref.size(); i++) {
        dev_acts = std::max(dev_acts, (near_in[i] - ref[i]).Length());
        dev_near_far = std::max(dev_near_far, (near_in[i] - far_in[i]).Length());
        dev_near_out = std::max(dev_near_out, (near_out[i] - ref[i]).Length());
        dev_far_out = std::max(dev_far_out, (far_out[i] - ref[i]).Length());
    }

    std::cout << "    compared " << ref.size() << " fluid particles" << std::endl;
    std::cout << "      body present vs absent:            max deviation " << dev_acts << " m" << std::endl;
    std::cout << "      one period out vs ten periods out: max deviation " << dev_near_far << " m" << std::endl;
    std::cout << "      one period out, also up in z:      max deviation " << dev_near_out << " m" << std::endl;
    std::cout << "      ten periods out, also up in z:     max deviation " << dev_far_out << " m" << std::endl;

    // x is periodic, so its image acts, and the two non-periodic axes must not prevent that. Same
    // threshold, and the same measured reason for it, as case 3.
    Check("the image across the periodic axis acts on the fluid even though y and z are not periodic",
          dev_acts > 1e-6);
    Check("ten periods out along the periodic axis matches one period out", dev_near_far < 1e-6);

    // Now the body is also a whole box length beyond a non-periodic face, which puts it far beyond
    // kernel support in z, and no whole number of z box lengths may be folded away because z has no
    // period. So the fluid must be identical to the no-body run rather than merely close, for the
    // reason and with the tolerance of case 4.
    //
    // The two legs are NOT redundant, and which one can fail is worth stating, because it is
    // measured. Against a library that folds every axis unconditionally and by at most one period,
    // the one-period leg FAILS, as it should: the periodic axis is imaged correctly, so the body is
    // a neighbor candidate, and the unconditional z fold then fabricates a force. The ten-period leg
    // PASSES on that same library, not because the axis is handled correctly but because the two
    // defects mask each other there: a single fold leaves the body about nine periods away in x, so
    // no pair is a candidate and the z fold never gets the chance to matter. The ten-period leg is
    // therefore a guard on the combination rather than a detector of a known defect, and the
    // one-period leg is what makes this case able to catch unconditional folding at all.
    Check("one period out and displaced along the non-periodic axis leaves the fluid untouched", dev_near_out < 1e-10);
    Check("ten periods out and displaced along the non-periodic axis leaves the fluid untouched", dev_far_out < 1e-10);
}

int main(int argc, char* argv[]) {
    TestRigidBodyOutsideDomain();
    TestFluidOutsideDomain();
    TestPeriodicImagedBodyActs();
    TestNonPeriodicOutsideBodyExertsNoForce();
    TestMixedPeriodicity();

    if (num_failures == 0) {
        std::cout << "\nAll out-of-domain guard checks passed" << std::endl;
        return 0;
    }
    std::cout << "\n" << num_failures << " check(s) failed" << std::endl;
    return 1;
}
