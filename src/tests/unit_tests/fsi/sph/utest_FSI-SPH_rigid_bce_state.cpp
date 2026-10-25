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
// Authors: Radu Serban
// =============================================================================
//
// Unit test for the velocities of rigid body BCE markers set from solid states
// loaded through ChFsiFluidSystemSPH::LoadSolidStates (the generic path, also used
// during initialization). The body angular velocity in FsiBodyState is expressed in
// the global frame; for a rotated and spinning body, the velocity of each BCE marker
// must be v + w x (p - x), with all quantities in the global frame.
//
// =============================================================================

#include <cmath>
#include <iostream>
#include <vector>

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChBody.h"
#include "chrono/utils/ChBodyGeometry.h"

#include "chrono_fsi/sph/ChFsiProblemSPH.h"

using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::fsi::sph;

double initial_spacing = 0.01;
double dt = 1e-4;

int num_failures = 0;

// Check the velocities of all BCE markers of the body (all markers within 'radius' of the body position).
void CheckBceVelocities(const std::string& name,
                        ChFsiFluidSystemSPH& sysSPH,
                        const ChVector3d& body_pos,
                        const ChVector3d& body_vel,
                        const ChVector3d& body_angvel_abs,
                        double radius) {
    auto pos = sysSPH.GetParticlePositions();
    auto vel = sysSPH.GetParticleVelocities();

    size_t num_checked = 0;
    double max_err = 0;
    double max_vel = 0;
    for (size_t i = 0; i < pos.size(); i++) {
        auto r = pos[i] - body_pos;
        if (r.Length() > radius)
            continue;
        auto expected = body_vel + Vcross(body_angvel_abs, r);
        max_err = std::max(max_err, (vel[i] - expected).Length());
        max_vel = std::max(max_vel, expected.Length());
        num_checked++;
    }

    // Marker states are stored in single precision
    bool ok = num_checked > 0 && max_err < 1e-4 * max_vel;
    std::cout << (ok ? "  ok:   " : "FAIL:   ") << name << "  (" << num_checked << " BCE markers, max error " << max_err << ", max velocity " << max_vel << ")" << std::endl;
    if (!ok)
        num_failures++;
}

int main(int argc, char* argv[]) {
    ChSystemSMC sysMBS;
    sysMBS.SetGravitationalAcceleration(VNULL);

    ChFsiProblemCartesian fsi(initial_spacing, &sysMBS);
    fsi.SetVerbose(false);
    auto sysSPH = fsi.GetFluidSystemSPH();
    fsi.SetGravitationalAcceleration(VNULL);

    ChFsiFluidSystemSPH::FluidProperties fluid_props;
    fluid_props.density = 1000;
    fluid_props.viscosity = 1;
    fsi.SetCfdSPH(fluid_props);

    ChFsiFluidSystemSPH::SPHParameters sph_params;
    sph_params.initial_spacing = initial_spacing;
    sph_params.num_bce_layers = 3;
    sph_params.max_velocity = 1;
    fsi.SetSPHParameters(sph_params);

    fsi.SetStepSizeCFD(dt);
    fsi.SetStepsizeMBD(dt);

    // Rigid body with a box FSI geometry, rotated and spinning
    ChVector3d body_pos(0, 0, 0);
    ChQuaterniond body_rot = QuatFromAngleAxis(50 * CH_DEG_TO_RAD, ChVector3d(1, 1, 0).GetNormalized());
    ChVector3d body_vel(0.5, -0.2, 0.1);
    ChVector3d body_angvel_abs(0.3, -2.0, 1.5);

    auto body = chrono_types::make_shared<ChBody>();
    body->SetMass(1);
    body->SetPos(body_pos);
    body->SetRot(body_rot);
    body->SetPosDt(body_vel);
    body->SetAngVelParent(body_angvel_abs);
    sysMBS.AddBody(body);

    double box_size = 0.1;
    auto geometry = chrono_types::make_shared<utils::ChBodyGeometry>();
    geometry->coll_boxes.push_back(utils::ChBodyGeometry::BoxShape(VNULL, QUNIT, ChVector3d(box_size)));
    fsi.AddRigidBody(body, geometry, false);

    // Small fluid block, far from the body (so that all markers near the body are BCE markers of the body)
    fsi.Construct(ChVector3d(0.1, 0.1, 0.1), ChVector3d(1.0, 0, 0), BoxSide::NONE);
    fsi.SetComputationalDomain(ChAABB(ChVector3d(-2, -2, -2), ChVector3d(2, 2, 2)));

    // Radius enclosing the body BCE markers (box half-diagonal plus BCE layers)
    double radius = std::sqrt(3.0) * box_size / 2 + 4 * initial_spacing;

    // 1. Solid states loaded during initialization
    fsi.Initialize();
    CheckBceVelocities("BCE velocities after initialization", *sysSPH, body_pos, body_vel, body_angvel_abs, radius);

    // 2. Solid states loaded through the generic FSI path.
    //    Take one step first, so that the sorted marker arrays (built by the proximity search) are available.
    fsi.DoStepDynamics(dt);
    FsiBodyState state;
    state.pos = ChVector3d(0.05, -0.02, 0.03);
    state.rot = QuatFromAngleAxis(-120 * CH_DEG_TO_RAD, ChVector3d(0.2, -1, 0.5).GetNormalized());
    state.lin_vel = ChVector3d(-0.3, 0.4, 0.2);
    state.ang_vel = ChVector3d(-1.0, 0.5, 2.5);
    sysSPH->LoadSolidStates({state});
    sysSPH->OnExchangeSolidStates();
    CheckBceVelocities("BCE velocities after LoadSolidStates", *sysSPH, state.pos, state.lin_vel, state.ang_vel, radius);

    std::cout << (num_failures == 0 ? "PASSED" : "FAILED") << std::endl;
    return num_failures == 0 ? 0 : 1;
}
