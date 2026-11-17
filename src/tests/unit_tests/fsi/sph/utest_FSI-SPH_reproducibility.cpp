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
// Check bit-identical rigid loads and coupled CRM state on repeated runs.
// =============================================================================

#include <cmath>
#include <cstring>
#include <iostream>
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

constexpr double spacing = 0.01;
constexpr double dt_cfd = 2e-5;
constexpr double dt = 1e-4;
constexpr double push_speed = 0.01;
constexpr int steps = 100;

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
    double mean_fz = 0;
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

Result RunPlate() {
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
    sph_params.num_proximity_search_steps = 1;
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

    fsi.AddRigidBody(ground, std::vector<ChVector3d>{}, ChFrame<>(), false);

    // The plate: a rigid FSI body, half a spacing above the soil, driven straight down by a motor.
    double plate_hx = 0.10, plate_hy = 0.08, plate_hz = 0.05;
    ChContactMaterialData cmat;
    auto geometry = chrono_types::make_shared<utils::ChBodyGeometry>();
    geometry->materials.push_back(cmat);
    geometry->coll_boxes.push_back(utils::ChBodyGeometry::BoxShape(ChVector3d(0, 0, 0), QUNIT, ChVector3d(2 * plate_hx, 2 * plate_hy, 2 * plate_hz), 0));
    auto plate = chrono_types::make_shared<ChBody>();
    plate->SetPos(ChVector3d(0, 0, bzDim + 0.5 * spacing + plate_hz));
    plate->SetRot(QUNIT);
    plate->SetMass(1.0);
    plate->SetInertiaXX(ChVector3d(0.001, 0.001, 0.001));
    sysMBS.AddBody(plate);
    fsi.AddRigidBody(plate, geometry, false);
    auto passive = chrono_types::make_shared<ChBody>();
    passive->SetFixed(true);
    passive->SetPos(ChVector3d(0, 0, bzDim + 2 * plate_hz + 4 * spacing));
    sysMBS.AddBody(passive);
    std::vector<ChVector3d> bce = {ChVector3d(-spacing, 0, 0), VNULL, ChVector3d(spacing, 0, 0)};
    fsi.AddRigidBody(passive, bce, ChFrame<>(), false);

    // The identity motor frame prescribes downward motion along world Z.
    auto motor = chrono_types::make_shared<ChLinkMotorLinearPosition>();
    motor->SetMotorFunction(chrono_types::make_shared<ChFunctionRamp>(0.0, -push_speed));
    motor->Initialize(plate, ground, ChFrame<>(ChVector3d(0, 0, 0), QUNIT));
    sysMBS.AddLink(motor);

    fsi.Initialize();

    check(fsi.GetNumBCE(plate) > 3 * 512 && fsi.GetNumBCE(plate) % 512 != 0, "repeatability plate spans several blocks with a partial last block");

    Result r;
    auto sample = [](std::vector<double>& values, const ChVector3d& v) { values.insert(values.end(), {v.x(), v.y(), v.z()}); };
    for (int step = 0; step < steps; step++) {
        fsi.DoStepDynamics(dt);
        sample(r.loads, fsi.GetFsiBodyForce(plate));
        sample(r.loads, fsi.GetFsiBodyTorque(plate));
        r.mean_fz += fsi.GetFsiBodyForce(plate).z();
    }
    r.mean_fz /= steps;

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

    for (const auto& v : sysSPH->GetParticleVelocities())
        sample(r.state, v);
    for (const auto& p : sysSPH->GetParticlePositions())
        sample(r.state, p);
    return r;
}

int main(int argc, char* argv[]) {
    Result reference = RunPlate();
    check(std::isfinite(reference.mean_fz) && reference.mean_fz > 0, "plate has a finite upward reaction");
    check(reference.repeated_loads, "rigid-marker reduction is bit-identical on a frozen state");
    for (int run = 0; run < 2; run++) {
        Result repeated = RunPlate();
        check(repeated.repeated_loads, "rigid-marker reduction is bit-identical on a frozen state");
        check(Identical(reference.loads, repeated.loads), "repeated CRM load histories are bit-identical");
        check(Identical(reference.state, repeated.state), "repeated CRM particle states are bit-identical");
    }
    cout << (num_failures == 0 ? "Test succeeded" : "Test FAILED") << endl;
    return num_failures == 0 ? 0 : 1;
}
