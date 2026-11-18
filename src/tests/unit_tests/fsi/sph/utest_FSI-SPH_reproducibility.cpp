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
// Check bit-identical solid loads and coupled CRM state on repeated runs.
//
// The FSI loads on solids are summed in a fixed order, so repeated runs with the same GPU and build must give
// bit-identical results. A rigid plate pushed into a CRM bed is simulated three times, and the plate loads and the
// full SPH particle state are compared bitwise. When Chrono is built with FEA, the bed also holds a cable (1D) and a
// shell (2D), both fixed at the base and partly buried, and their nodal loads and node positions are compared too.
// Comparing the particle state makes the whole SPH step a determinism guard: any new order-dependent reduction
// (e.g., a floating-point atomic) anywhere in the step fails this test.
// =============================================================================

#include <algorithm>
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

#ifdef CHRONO_FEA
    #include "chrono/fea/ChBuilderBeam.h"
    #include "chrono/fea/ChElementShellANCF_3423.h"
    #include "chrono/fea/ChMesh.h"
    #include "chrono/fea/ChNodeFEAxyzD.h"
    #include "chrono/solver/ChIterativeSolverLS.h"
#endif

using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::fsi::sph;
#ifdef CHRONO_FEA
using namespace chrono::fea;
#endif

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
    bool flex_loaded = true;
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

#ifdef CHRONO_FEA
// A vertical ANCF cable with its bottom node fixed, from z0 up to z1 at (x, 0).
std::shared_ptr<ChMesh> CreateCable(ChSystem& sysMBS, double x, double z0, double z1) {
    auto section = chrono_types::make_shared<ChBeamSectionCableANCF>();
    section->SetDiameter(0.02);
    section->SetYoungModulus(2e8);
    section->SetDensity(8000);
    section->SetRayleighDamping(0.02);

    auto mesh = chrono_types::make_shared<ChMesh>();
    ChBuilderCableANCF builder;
    builder.BuildBeam(mesh, section, 4, ChVector3d(x, 0, z1), ChVector3d(x, 0, z0));
    builder.GetLastBeamNodes().back()->SetFixed(true);
    sysMBS.Add(mesh);
    return mesh;
}

// A vertical ANCF shell in the y-z plane at x, 2 x 4 elements, with its bottom row of nodes fixed.
std::shared_ptr<ChMesh> CreateShell(ChSystem& sysMBS, double x, double width, double z0, double z1) {
    auto material = chrono_types::make_shared<ChMaterialShellANCF>(8000, 2e7, 0.3);
    const int ny = 2, nz = 4;
    double dy = width / ny, dz = (z1 - z0) / nz;

    auto mesh = chrono_types::make_shared<ChMesh>();
    std::vector<std::shared_ptr<ChNodeFEAxyzD>> nodes;
    for (int k = 0; k <= nz; k++) {
        for (int j = 0; j <= ny; j++) {
            auto node = chrono_types::make_shared<ChNodeFEAxyzD>(ChVector3d(x, j * dy - width / 2, z0 + k * dz), ChVector3d(1, 0, 0));
            node->SetFixed(k == 0);
            mesh->AddNode(node);
            nodes.push_back(node);
        }
    }
    for (int k = 0; k < nz; k++) {
        for (int j = 0; j < ny; j++) {
            auto element = chrono_types::make_shared<ChElementShellANCF_3423>();
            element->SetNodes(nodes[j + (ny + 1) * k], nodes[j + 1 + (ny + 1) * k], nodes[j + 1 + (ny + 1) * (k + 1)], nodes[j + (ny + 1) * (k + 1)]);
            element->SetDimensions(dy, dz);
            element->AddLayer(0.01, 0, material);
            element->SetAlphaDamp(0.0);
            mesh->AddElement(element);
        }
    }
    sysMBS.Add(mesh);
    return mesh;
}
#endif

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

#ifdef CHRONO_FEA
    // Flexible solids at both ends of the bed, clear of the plate, each buried over its lower quarter.
    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    solver->SetMaxIterations(2000);
    solver->SetTolerance(1e-12);
    solver->EnableDiagonalPreconditioner(true);
    sysMBS.SetSolver(solver);

    auto cable = CreateCable(sysMBS, -0.125, 0.08, 0.24);
    auto shell = CreateShell(sysMBS, 0.125, 0.08, 0.08, 0.24);
    fsi.AddFeaMesh(cable, false);
    fsi.AddFeaMesh(shell, false);
#endif

    fsi.Initialize();

    // 512 is the block size of the rigid force reduction (SphBceManager); with several blocks and a partial last one,
    // the plate exercises both the per-block sums and the final per-body sum. Update if that block size changes.
    check(fsi.GetNumBCE(plate) > 3 * 512 && fsi.GetNumBCE(plate) % 512 != 0, "repeatability plate spans several blocks with a partial last block");

    Result r;
    auto sysSPH = fsi.GetFluidSystemSPH();
    auto sample = [](std::vector<double>& values, const ChVector3d& v) { values.insert(values.end(), {v.x(), v.y(), v.z()}); };

    // Loads on all FSI solids, as computed in the last call to OnExchangeSolidForces.
    std::vector<FsiBodyForce> forces(3);
#ifdef CHRONO_FEA
    std::vector<FsiMeshForce> forces1D(1), forces2D(1);
    forces1D[0].force.resize(cable->GetNumNodes());
    forces2D[0].force.resize(shell->GetNumNodes());
#endif
    auto sample_loads = [&](std::vector<double>& values) {
#ifdef CHRONO_FEA
        sysSPH->StoreSolidForces(forces, forces1D, forces2D);
        for (const auto& f : forces1D[0].force)
            sample(values, f);
        for (const auto& f : forces2D[0].force)
            sample(values, f);
#else
        sysSPH->StoreSolidForces(forces);
#endif
        for (const auto& force : forces) {
            sample(values, force.force);
            sample(values, force.torque);
        }
    };

    for (int step = 0; step < steps; step++) {
        fsi.DoStepDynamics(dt);
        sample_loads(r.loads);
        r.mean_fz += fsi.GetFsiBodyForce(plate).z();
    }
    r.mean_fz /= steps;

#ifdef CHRONO_FEA
    // The buried parts of both flexible solids must carry soil loads.
    auto max_load = [](const FsiMeshForce& mesh_forces) {
        double max = 0;
        for (const auto& f : mesh_forces.force)
            max = std::max(max, f.Length());
        return max;
    };
    r.flex_loaded = max_load(forces1D[0]) > 0 && max_load(forces2D[0]) > 0;
#endif

    // Recompute loads without advancing the state to isolate the solid-force reductions.
    std::vector<double> reference;
    for (int repeat = 0; repeat < 64; repeat++) {
        sysSPH->OnExchangeSolidForces();
        std::vector<double> current;
        sample_loads(current);
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
#ifdef CHRONO_FEA
    for (const auto& mesh : {cable, shell})
        for (const auto& node : mesh->GetNodes())
            sample(r.state, std::static_pointer_cast<ChNodeFEAxyzD>(node)->GetPos());
#endif
    return r;
}

int main(int argc, char* argv[]) {
    Result reference = RunPlate();
    check(std::isfinite(reference.mean_fz) && reference.mean_fz > 0, "plate has a finite upward reaction");
#ifdef CHRONO_FEA
    check(reference.flex_loaded, "cable and shell carry soil loads");
#endif
    check(reference.repeated_loads, "solid-force reductions are bit-identical on a frozen state");
    for (int run = 0; run < 2; run++) {
        Result repeated = RunPlate();
        check(repeated.repeated_loads, "solid-force reductions are bit-identical on a frozen state");
        check(Identical(reference.loads, repeated.loads), "repeated CRM load histories are bit-identical");
        check(Identical(reference.state, repeated.state), "repeated CRM particle states are bit-identical");
    }
    cout << (num_failures == 0 ? "Test succeeded" : "Test FAILED") << endl;
    return num_failures == 0 ? 0 : 1;
}
