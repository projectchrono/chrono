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
// Authors: Simulation-Based Engineering Laboratory, University of Wisconsin-Madison
// =============================================================================
//
// Regression test for the neighbour-search cadence with a rigid body driven through the soil.
//
// The Adami boundary condition (CrmAdamiBC / CfdAdamiBC) reads a solid BCE marker's wall velocity
// from its velocity slot and writes the extrapolated ghost velocity back into the same slot. That
// slot is restored from the solid state only when the proximity search runs (reorderDataD), so with
// num_proximity_search_steps > 1 every skipped step used to apply v <- 2v - <v_fluid> to an already
// modified value. A plate pushed slowly into a CRM bed then read about half the bearing pressure at
// cadence 2 that it read at cadence 1. ChFsiFluidSystemSPH::OnDoStepDynamics now refreshes the solid
// marker state on the steps that skip the search.
//
// The coupled system (ChFsiSystem) refreshes the solid marker state once per coupling step, so the
// corruption shows when the fluid takes several steps per coupling step: only the first substep after
// the refresh reads a clean slot. The test therefore uses five fluid steps per coupling step (2e-5 s
// against 1e-4 s), pushes a plate down into a small CRM bed at 10 mm/s, once at cadence 1 and once at
// cadence 4, and compares the vertical reaction force on the plate averaged over the last part of the
// push. With the refresh the two agree closely; without it they do not.
//
// =============================================================================

#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChLinkMotorLinearPosition.h"
#include "chrono/functions/ChFunctionRamp.h"
#include "chrono/utils/ChBodyGeometry.h"
#include "chrono_fsi/sph/ChFsiProblemSPH.h"

using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::fsi::sph;

using std::cout;
using std::endl;

double initial_spacing = 0.02;
double dt_cfd = 2e-5;      // fluid step
double dt = 1e-4;          // coupling step: five fluid steps per coupling step
double push_speed = 0.01;  // m/s, downward
int num_steps = 2000;      // 0.2 s of pushing, 2 mm of sinkage
int avg_steps = 500;       // the force is averaged over the last 0.05 s

int num_failures = 0;

void check(bool condition, const std::string& what) {
    if (condition) {
        cout << "  PASS  " << what << endl;
    } else {
        cout << "  FAIL  " << what << endl;
        num_failures++;
    }
}

struct Result {
    double mean_fz;  // mean vertical FSI force on the plate over the last avg_steps steps
    double vmax;     // fastest particle at the end
    std::vector<double> loads;
    std::vector<double> state;
    bool repeated_loads = true;
};

bool Identical(const std::vector<double>& a, const std::vector<double>& b) {
    if (a.empty() || a.size() != b.size())
        return false;
    for (size_t i = 0; i < a.size(); i++) {
        if (!std::isfinite(a[i]) || !std::isfinite(b[i]) || std::memcmp(&a[i], &b[i], sizeof(double)) != 0) {
            cout << "  first differing or non-finite component " << i << ": " << std::hexfloat << a[i] << " vs " << b[i] << std::defaultfloat << endl;
            return false;
        }
    }
    return true;
}

// Push a plate into a small CRM bed at the given neighbour-search cadence, with a fixed time step.
Result PushPlateAtCadence(int cadence, bool repeatability = false) {
    double spacing = repeatability ? 0.01 : initial_spacing;
    int steps = repeatability ? 100 : num_steps;
    ChSystemSMC sysMBS;
    sysMBS.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));

    ChFsiProblemCartesian fsi(spacing, &sysMBS);
    fsi.SetVerbose(false);
    fsi.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));

    ChFsiFluidSystemSPH::SoilProperties mat_props;
    mat_props.density = 1700;
    mat_props.Young_modulus = 1e6;
    mat_props.Poisson_ratio = 0.3;
    mat_props.mu_fric_s = 0.7;
    mat_props.mu_fric_2 = 0.7;
    mat_props.average_diam = 0.005;
    mat_props.cohesion_coeff = 0;
    fsi.SetCrmSPH(mat_props);

    ChFsiFluidSystemSPH::SPHParameters sph_params;
    sph_params.integration_scheme = IntegrationScheme::RK2;
    sph_params.initial_spacing = spacing;
    sph_params.d0_multiplier = 1.2;
    sph_params.num_bce_layers = 3;
    sph_params.shifting_method = ShiftingMethod::PPST_XSPH;
    sph_params.artificial_viscosity = 0.5;
    sph_params.use_variable_time_step = false;
    sph_params.num_proximity_search_steps = cadence;
    fsi.SetSPHParameters(sph_params);

    fsi.SetStepSizeCFD(dt_cfd);
    fsi.SetStepsizeMBD(dt);

    // A soil box with a bottom and four walls, open at the top. Soil occupies 0 <= z <= bzDim.
    double bxDim = 0.3, byDim = 0.2, bzDim = 0.12;
    fsi.Construct(ChVector3d(bxDim, byDim, bzDim), ChVector3d(0, 0, 0), BoxSide::ALL & ~BoxSide::Z_POS);

    // Ground for the motor.
    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetFixed(true);
    sysMBS.AddBody(ground);
    if (repeatability)
        fsi.AddRigidBody(ground, std::vector<ChVector3d>{}, ChFrame<>(), false);

    // The plate: a rigid FSI body, half a spacing above the soil, driven straight down by a motor.
    double plate_hx = 0.05, plate_hy = 0.05, plate_hz = 0.01;
    if (repeatability) {
        plate_hx = 0.10;
        plate_hy = 0.08;
        plate_hz = 0.05;
    }
    ChContactMaterialData cmat;
    auto geometry = chrono_types::make_shared<utils::ChBodyGeometry>();
    geometry->materials.push_back(cmat);
    geometry->coll_boxes.push_back(
        utils::ChBodyGeometry::BoxShape(ChVector3d(0, 0, 0), QUNIT, ChVector3d(2 * plate_hx, 2 * plate_hy, 2 * plate_hz), 0));
    auto plate = chrono_types::make_shared<ChBody>();
    plate->SetPos(ChVector3d(0, 0, bzDim + 0.5 * spacing + plate_hz));
    plate->SetRot(QUNIT);
    plate->SetMass(1.0);
    plate->SetInertiaXX(ChVector3d(0.001, 0.001, 0.001));
    sysMBS.AddBody(plate);
    fsi.AddRigidBody(plate, geometry, false);
    if (repeatability) {
        auto passive = chrono_types::make_shared<ChBody>();
        passive->SetFixed(true);
        passive->SetPos(ChVector3d(0, 0, bzDim + 2 * plate_hz + 4 * spacing));
        sysMBS.AddBody(passive);
        std::vector<ChVector3d> bce = {ChVector3d(-spacing, 0, 0), VNULL, ChVector3d(spacing, 0, 0)};
        fsi.AddRigidBody(passive, bce, ChFrame<>(), false);
    }

    // ChLinkMotorLinearPosition prescribes translation along the link Z axis; an identity frame makes
    // that world-vertical. A ramp with negative slope drives the plate down at push_speed.
    auto motor = chrono_types::make_shared<ChLinkMotorLinearPosition>();
    motor->SetMotorFunction(chrono_types::make_shared<ChFunctionRamp>(0.0, -push_speed));
    motor->Initialize(plate, ground, ChFrame<>(ChVector3d(0, 0, 0), QUNIT));
    sysMBS.AddLink(motor);

    fsi.Initialize();
    if (repeatability)
        check(fsi.GetNumBCE(plate) > 3 * 512 && fsi.GetNumBCE(plate) % 512 != 0, "repeatability plate spans several blocks with a partial last block");

    Result r{0.0, 0.0};
    int n_avg = 0;
    auto sample = [](std::vector<double>& values, const ChVector3d& v) { values.insert(values.end(), {v.x(), v.y(), v.z()}); };
    for (int step = 0; step < steps; step++) {
        fsi.DoStepDynamics(dt);
        if (repeatability) {
            sample(r.loads, fsi.GetFsiBodyForce(plate));
            sample(r.loads, fsi.GetFsiBodyTorque(plate));
        }
        if (step >= steps - avg_steps) {
            r.mean_fz += fsi.GetFsiBodyForce(plate).z();
            n_avg++;
        }
    }
    r.mean_fz /= n_avg;

    if (repeatability) {
        // Recompute loads without advancing the state to isolate the rigid-marker reduction.
        auto sysSPH = fsi.GetFluidSystemSPH();
        std::vector<FsiBodyForce> forces(3);
        std::vector<double> reference;
        for (int repeat = 0; repeat < 64; repeat++) {
            sysSPH->OnExchangeSolidForces();
            sysSPH->StoreSolidForces(forces);
            std::vector<double> current;
            for (const auto& force : forces) {
                sample(current, force.force);
                sample(current, force.torque);
            }
            if (repeat == 0)
                reference = current;
            else if (!Identical(reference, current)) {
                r.repeated_loads = false;
                break;
            }
        }
        check(forces[0].force == VNULL && forces[0].torque == VNULL, "body without BCE markers has zero loads");
        check(forces[2].force == VNULL && forces[2].torque == VNULL, "passive body after the partial plate block has zero loads");
    }

    for (const auto& v : fsi.GetFluidSystemSPH()->GetParticleVelocities()) {
        double s = v.Length();
        if (!std::isfinite(s))
            return Result{std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
        r.vmax = std::max(r.vmax, s);
        if (repeatability)
            sample(r.state, v);
    }
    if (repeatability)
        for (const auto& p : fsi.GetFluidSystemSPH()->GetParticlePositions())
            sample(r.state, p);
    return r;
}

int main(int argc, char* argv[]) {
    cout << "Neighbour-search cadence with a rigid body driven through the soil" << endl;

    Result r1 = PushPlateAtCadence(1);
    cout << "  cadence 1: mean vertical force on the plate over the last " << avg_steps << " steps = " << r1.mean_fz
         << " N, peak particle speed = " << r1.vmax << " m/s" << endl;
    Result r4 = PushPlateAtCadence(4);
    cout << "  cadence 4: mean vertical force on the plate over the last " << avg_steps << " steps = " << r4.mean_fz
         << " N, peak particle speed = " << r4.vmax << " m/s" << endl;

    check(std::isfinite(r1.mean_fz) && std::isfinite(r4.mean_fz) && std::isfinite(r1.vmax) && std::isfinite(r4.vmax),
          "both runs stay finite");
    check(r1.mean_fz > 0, "the plate feels an upward reaction at cadence 1");
    // With the refresh on skipped steps the two forces agree to about 1e-7 on both backends; without it, the
    // wall velocity the soil sees is corrupted on every skipped step and the force is far off.
    double rel = std::abs(r4.mean_fz - r1.mean_fz) / std::max(std::abs(r1.mean_fz), 1e-9);
    cout << "  relative difference in force between cadence 4 and cadence 1 = " << rel << endl;
    check(rel < 0.05, "cadence 4 reaction force within 5 percent of cadence 1");
    // Grains thrown clear of the surface can move fast in either run; the bound only guards against
    // the runaway (metres per second across the whole bed) that the corruption produced.
    check(r4.vmax < 10.0 * std::max(r1.vmax, 0.1), "cadence 4 peak particle speed is not a runaway relative to cadence 1");

    // The larger plate spans several force-reduction blocks; compare complete loads and final marker states.
    Result reference = PushPlateAtCadence(1, true);
    check(std::isfinite(reference.mean_fz) && reference.mean_fz > 0, "repeatability plate has a finite upward reaction");
    check(reference.repeated_loads, "rigid-marker reduction is bit-identical on a frozen state");
    for (int run = 0; run < 2; run++) {
        Result repeated = PushPlateAtCadence(1, true);
        check(repeated.repeated_loads, "rigid-marker reduction is bit-identical on a frozen state");
        check(Identical(reference.loads, repeated.loads), "repeated CRM load histories are bit-identical");
        check(Identical(reference.state, repeated.state), "repeated CRM particle states are bit-identical");
    }

    cout << (num_failures == 0 ? "Test succeeded" : "Test FAILED") << endl;
    return num_failures == 0 ? 0 : 1;
}
