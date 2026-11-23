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
// Check the ChFsiFluidSystemSPH marker getters and their MarkerGroup selection.
//
// A rigid plate is pushed into a CRM bed; when Chrono is built with FEA, the bed also holds a cable (1D) and a shell
// (2D), both fixed at the base and partly buried. After a few steps, the test checks that:
// - every getter returns one entry per marker for ALL, and that FLUID and SOLID are exact slices of ALL;
// - the force on an SPH particle is its mass times its acceleration;
// - BCE markers on fixed boundaries report zero acceleration and force;
// - BCE markers on fixed boundaries carry the stress extrapolated from the bed, compressive under its weight;
// - the forces on the BCE markers of each solid sum to the FSI force on that solid (the plate force, or the sum of
//   the nodal forces of a mesh), so they are the per-marker fluid loads;
// - the accelerations of solid BCE markers are those of the solid (the plate acceleration, or the node accelerations
//   interpolated along the cable), not the fluid force per unit mass;
// - FindParticlesInBox returns indices in the ALL order, and the indexed getters return the ALL entries at them.
//
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
constexpr double dt = 1e-4;
constexpr int steps = 50;

int num_failures = 0;

void check(bool condition, const std::string& what) {
    if (condition) {
        cout << "  PASS  " << what << endl;
    } else {
        cout << "  FAIL  " << what << endl;
        num_failures++;
    }
}

// Check that 'part' equals the entries of 'all' starting at 'first', bit for bit.
template <typename T>
bool IsSlice(const std::vector<T>& part, const std::vector<T>& all, size_t first) {
    if (first + part.size() > all.size())
        return false;
    return part.empty() || std::memcmp(part.data(), all.data() + first, part.size() * sizeof(T)) == 0;
}

// Sum of a range of vectors.
ChVector3d Sum(const std::vector<ChVector3d>& v, size_t first, size_t last) {
    ChVector3d sum(0, 0, 0);
    for (size_t i = first; i < last; i++)
        sum += v[i];
    return sum;
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

// Sum of the FSI forces on the nodes of a mesh.
ChVector3d SumNodeForces(const FsiMeshForce& mesh_forces) {
    ChVector3d sum(0, 0, 0);
    for (const auto& f : mesh_forces.force)
        sum += f;
    return sum;
}
#endif

int main(int argc, char* argv[]) {
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
    sph_params.num_proximity_search_steps = 1;
    fsi.SetSPHParameters(sph_params);

    fsi.SetStepSizeCFD(2e-5);
    fsi.SetStepsizeMBD(dt);

    // A soil box with a bottom and four walls, open at the top. Soil occupies 0 <= z <= bzDim.
    double bzDim = 0.12;
    fsi.Construct(ChVector3d(0.3, 0.2, bzDim), ChVector3d(0, 0, 0), BoxSide::ALL & ~BoxSide::Z_POS);

    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetFixed(true);
    sysMBS.AddBody(ground);

    // The plate: a rigid FSI body, half a spacing above the soil, driven straight down by a motor.
    double plate_hz = 0.05;
    auto geometry = chrono_types::make_shared<utils::ChBodyGeometry>();
    geometry->materials.push_back(ChContactMaterialData());
    geometry->coll_boxes.push_back(utils::ChBodyGeometry::BoxShape(ChVector3d(0, 0, 0), QUNIT, ChVector3d(0.2, 0.16, 2 * plate_hz), 0));
    auto plate = chrono_types::make_shared<ChBody>();
    plate->SetPos(ChVector3d(0, 0, bzDim + 0.5 * spacing + plate_hz));
    plate->SetMass(1.0);
    plate->SetInertiaXX(ChVector3d(0.001, 0.001, 0.001));
    sysMBS.AddBody(plate);
    fsi.AddRigidBody(plate, geometry, false);

    auto motor = chrono_types::make_shared<ChLinkMotorLinearPosition>();
    motor->SetMotorFunction(chrono_types::make_shared<ChFunctionRamp>(0.0, -0.01));
    motor->Initialize(plate, ground, ChFrame<>(ChVector3d(0, 0, 0), QUNIT));
    sysMBS.AddLink(motor);

#ifdef CHRONO_FEA
    // Flexible solids at both ends of the bed, clear of the plate, each buried over its lower quarter.
    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    solver->SetMaxIterations(2000);
    solver->SetTolerance(1e-12);
    solver->EnableDiagonalPreconditioner(true);
    sysMBS.SetSolver(solver);

    double cable_x = -0.125;
    auto cable = CreateCable(sysMBS, cable_x, 0.08, 0.24);
    auto shell = CreateShell(sysMBS, 0.125, 0.08, 0.08, 0.24);
    fsi.AddFeaMesh(cable, false);
    fsi.AddFeaMesh(shell, false);
#endif

    fsi.Initialize();
    for (int step = 0; step < steps; step++)
        fsi.DoStepDynamics(dt);

    auto sysSPH = fsi.GetFluidSystemSPH();
    size_t num_fluid = sysSPH->GetNumFluidMarkers();
    size_t first_solid = num_fluid + sysSPH->GetNumBoundaryMarkers();
    size_t num_plate = fsi.GetNumBCE(plate);
    size_t last_plate = first_solid + num_plate;

    // Results for all markers
    auto pos = sysSPH->GetParticlePositions(MarkerGroup::ALL);
    auto vel = sysSPH->GetParticleVelocities(MarkerGroup::ALL);
    auto acc = sysSPH->GetParticleAccelerations(MarkerGroup::ALL);
    auto frc = sysSPH->GetParticleForces(MarkerGroup::ALL);
    auto prop = sysSPH->GetParticleFluidProperties(MarkerGroup::ALL);
    auto flags = sysSPH->GetFreeSurfaceFlags(MarkerGroup::ALL);
    auto tau_d = sysSPH->GetParticleStressDiagonal(MarkerGroup::ALL);
    auto tau_o = sysSPH->GetParticleStressOffDiagonal(MarkerGroup::ALL);
    size_t num_all = pos.size();

    check(num_all > first_solid && num_plate > 0, "markers include SPH particles, boundary BCE, and solid BCE");
    check(vel.size() == num_all && acc.size() == num_all && frc.size() == num_all && prop.size() == num_all && flags.size() == num_all && tau_d.size() == num_all &&
              tau_o.size() == num_all,
          "all getters return one entry per marker for ALL");
    check(sysSPH->GetParticleAccelerations().size() == num_all && sysSPH->GetParticleForces().size() == num_all && sysSPH->GetParticleStressDiagonal().size() == num_all &&
              sysSPH->GetParticleStressOffDiagonal().size() == num_all,
          "the default group is ALL");

    // FLUID and SOLID are exact slices of ALL
    bool slices = true;
    for (auto group : {MarkerGroup::FLUID, MarkerGroup::SOLID}) {
        size_t first = (group == MarkerGroup::FLUID) ? 0 : first_solid;
        size_t num = (group == MarkerGroup::FLUID) ? num_fluid : num_all - first_solid;
        auto p = sysSPH->GetParticlePositions(group);
        auto v = sysSPH->GetParticleVelocities(group);
        auto a = sysSPH->GetParticleAccelerations(group);
        auto f = sysSPH->GetParticleForces(group);
        auto r = sysSPH->GetParticleFluidProperties(group);
        auto fl = sysSPH->GetFreeSurfaceFlags(group);
        auto sd = sysSPH->GetParticleStressDiagonal(group);
        auto so = sysSPH->GetParticleStressOffDiagonal(group);
        slices = slices && p.size() == num && v.size() == num && a.size() == num && f.size() == num && r.size() == num && fl.size() == num && sd.size() == num && so.size() == num;
        slices =
            slices && IsSlice(p, pos, first) && IsSlice(v, vel, first) && IsSlice(a, acc, first) && IsSlice(f, frc, first) && IsSlice(r, prop, first) && IsSlice(fl, flags, first);
        slices = slices && IsSlice(sd, tau_d, first) && IsSlice(so, tau_o, first);
    }
    check(slices, "FLUID and SOLID results are exact slices of ALL");

    // SPH particles: force equals mass times acceleration
    double mass = sysSPH->GetParticleMass();
    double max_dev = 0;
    for (size_t i = 0; i < num_fluid; i++) {
        double norm = mass * acc[i].Length();
        if (norm > 0)
            max_dev = std::max(max_dev, (frc[i] - mass * acc[i]).Length() / norm);
    }
    check(max_dev < 1e-6, "SPH particle forces equal mass times acceleration");

    // BCE markers on fixed boundaries: zero acceleration and force
    bool walls = true;
    for (size_t i = num_fluid; i < first_solid; i++)
        walls = walls && acc[i] == VNULL && frc[i] == VNULL;
    check(walls, "BCE markers on fixed boundaries report zero acceleration and force");

    // BCE markers on fixed boundaries: stress extrapolated from the bed, compressive on average under its weight
    double wall_pressure = 0;
    for (size_t i = num_fluid; i < first_solid; i++)
        wall_pressure -= (tau_d[i].x() + tau_d[i].y() + tau_d[i].z()) / 3;
    check(first_solid > num_fluid && wall_pressure > 0, "BCE markers on fixed boundaries carry the compressive stress of the bed");

    // Plate markers: forces sum to the plate force, and accelerations are those of the plate (which does not rotate)
    auto plate_force = fsi.GetFsiBodyForce(plate);
    check(plate_force.Length() > 0 && (Sum(frc, first_solid, last_plate) - plate_force).Length() <= 1e-4 * plate_force.Length(), "plate marker forces sum to the plate force");
    double max_acc_dev = 0;
    for (size_t i = first_solid; i < last_plate; i++)
        max_acc_dev = std::max(max_acc_dev, (acc[i] - plate->GetPosDt2()).Length());
    check(max_acc_dev <= 1e-6, "plate marker accelerations equal the plate acceleration");

#ifdef CHRONO_FEA
    // Flexible markers follow the plate markers; cable markers are at negative x, shell markers at positive x
    std::vector<FsiBodyForce> body_forces(2);
    std::vector<FsiMeshForce> forces1D(1), forces2D(1);
    forces1D[0].force.resize(cable->GetNumNodes());
    forces2D[0].force.resize(shell->GetNumNodes());
    sysSPH->StoreSolidForces(body_forces, forces1D, forces2D);

    ChVector3d cable_sum(0, 0, 0), shell_sum(0, 0, 0);
    size_t num_cable = 0, num_shell = 0;
    for (size_t i = last_plate; i < num_all; i++) {
        if (pos[i].x() < 0) {
            cable_sum += frc[i];
            num_cable++;
        } else {
            shell_sum += frc[i];
            num_shell++;
        }
    }
    auto cable_force = SumNodeForces(forces1D[0]);
    auto shell_force = SumNodeForces(forces2D[0]);
    check(num_cable > 0 && num_shell > 0 && num_plate + num_cable + num_shell == num_all - first_solid, "solid BCE markers are on the plate, cable, and shell");
    check(cable_force.Length() > 0 && (cable_sum - cable_force).Length() <= 1e-4 * cable_force.Length(), "cable marker forces sum to the cable nodal forces");
    check(shell_force.Length() > 0 && (shell_sum - shell_force).Length() <= 1e-4 * shell_force.Length(), "shell marker forces sum to the shell nodal forces");

    // Cable markers: accelerations interpolate the node accelerations along the cable (nodes ordered top to bottom)
    std::vector<std::shared_ptr<ChNodeFEAxyzD>> nodes;
    for (const auto& node : cable->GetNodes())
        nodes.push_back(std::static_pointer_cast<ChNodeFEAxyzD>(node));
    double max_node_acc = 0;
    for (const auto& node : nodes)
        max_node_acc = std::max(max_node_acc, node->GetPosDt2().Length());
    double max_cable_dev = 0;
    for (size_t i = last_plate; i < num_all; i++) {
        if (pos[i].x() >= 0)
            continue;
        double z = pos[i].z();
        size_t k = 0;
        while (k + 2 < nodes.size() && z < nodes[k + 1]->GetPos().z())
            k++;
        double t = (z - nodes[k]->GetPos().z()) / (nodes[k + 1]->GetPos().z() - nodes[k]->GetPos().z());
        auto expected = (1 - t) * nodes[k]->GetPosDt2() + t * nodes[k + 1]->GetPosDt2();
        max_cable_dev = std::max(max_cable_dev, (acc[i] - expected).Length());
    }
    check(max_node_acc > 0 && max_cable_dev <= 1e-3 * max_node_acc, "cable marker accelerations interpolate the cable node accelerations");
#endif

    // Box search: a box enclosing the domain finds every marker of each group, with indices in the ALL order
    ChFrame<> frame(ChVector3d(0, 0, 0.1));
    ChVector3d size(10, 10, 10);
    auto ids_fluid = sysSPH->FindParticlesInBox(frame, size, MarkerGroup::FLUID);
    auto ids_solid = sysSPH->FindParticlesInBox(frame, size, MarkerGroup::SOLID);
    auto ids_all = sysSPH->FindParticlesInBox(frame, size);
    check(ids_fluid.size() == num_fluid && ids_solid.size() == num_all - first_solid && ids_all.size() == num_all, "box enclosing the domain finds every marker of each group");
    check(ids_fluid.back() == (int)num_fluid - 1 && ids_solid.front() == (int)first_solid && ids_solid.back() == (int)num_all - 1, "box indices refer to the ALL order");

    // A box around the plate, inset from its faces, finds only plate markers among the solid markers
    auto ids_plate = sysSPH->FindParticlesInBox(ChFrame<>(plate->GetPos()), ChVector3d(0.18, 0.14, 0.08), MarkerGroup::SOLID);
    bool only_plate = !ids_plate.empty();
    for (auto i : ids_plate)
        only_plate = only_plate && (size_t)i >= first_solid && (size_t)i < last_plate;
    check(only_plate, "box inside the plate finds only plate markers");

    // Indexed getters return the ALL entries at the given indices
    auto pos_ids = sysSPH->GetPositions(ids_all);
    auto vel_ids = sysSPH->GetVelocities(ids_all);
    auto acc_ids = sysSPH->GetAccelerations(ids_all);
    auto frc_ids = sysSPH->GetForces(ids_all);
    auto pos3 = sysSPH->GetPositions();
    auto vel3 = sysSPH->GetVelocities();
    auto acc3 = sysSPH->GetAccelerations();
    auto frc3 = sysSPH->GetForces();
    bool indexed = pos_ids.size() == num_all && vel_ids.size() == num_all && acc_ids.size() == num_all && frc_ids.size() == num_all;
    for (size_t k = 0; k < ids_all.size() && indexed; k++) {
        size_t i = ids_all[k];
        indexed = std::memcmp(&pos_ids[k], &pos3[i], sizeof(Real3)) == 0 && std::memcmp(&vel_ids[k], &vel3[i], sizeof(Real3)) == 0 &&
                  std::memcmp(&acc_ids[k], &acc3[i], sizeof(Real3)) == 0 && std::memcmp(&frc_ids[k], &frc3[i], sizeof(Real3)) == 0;
    }
    check(indexed, "indexed getters return the ALL entries at the given indices");

    cout << (num_failures == 0 ? "Test succeeded" : "Test FAILED") << endl;
    return num_failures == 0 ? 0 : 1;
}
