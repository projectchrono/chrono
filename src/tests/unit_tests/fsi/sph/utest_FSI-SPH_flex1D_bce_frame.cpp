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
// Unit test for the continuity of the BCE marker positions on 1-D FEA meshes.
// The BCE markers of a 1-D segment are placed in a cross-section frame attached
// to the segment. A single segment, centered at the origin, is rotated in small
// increments through directions for which a frame calculated from the current
// segment direction alone switches its reference axis (|x.y| = 0.9). Between two
// consecutive configurations, no BCE marker may move more than the rotation of
// the segment implies.
//
// =============================================================================

#include <cmath>
#include <iostream>
#include <vector>

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChNodeFEAxyzD.h"
#include "chrono/fea/ChElementCableANCF.h"

#include "chrono_fsi/sph/ChFsiProblemSPH.h"

using namespace chrono;
using namespace chrono::fea;
using namespace chrono::fsi;
using namespace chrono::fsi::sph;

double initial_spacing = 0.01;
double dt = 1e-4;
double seg_length = 0.1;

// Segment direction for the given angle: rotation from u = (1,0,1)/sqrt(2) towards the y axis.
// The y component of the direction crosses 0.9 at angle asin(0.9) = 64.16 degrees.
ChVector3d SegmentDirection(double angle) {
    ChVector3d u = ChVector3d(1, 0, 1).GetNormalized();
    return std::cos(angle) * u + std::sin(angle) * ChVector3d(0, 1, 0);
}

int main(int argc, char* argv[]) {
    ChSystemSMC sysMBS;
    sysMBS.SetGravitationalAcceleration(VNULL);
    sysMBS.SetSolverType(ChSolver::Type::MINRES);

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

    // Single-segment FEA mesh (one ANCF cable element), centered at the origin
    double angle_start = 50 * CH_DEG_TO_RAD;
    double angle_end = 80 * CH_DEG_TO_RAD;
    double angle_incr = 0.25 * CH_DEG_TO_RAD;

    auto dir = SegmentDirection(angle_start);
    auto node0 = chrono_types::make_shared<ChNodeFEAxyzD>(-0.5 * seg_length * dir, dir);
    auto node1 = chrono_types::make_shared<ChNodeFEAxyzD>(+0.5 * seg_length * dir, dir);
    auto section = chrono_types::make_shared<ChBeamSectionCableANCF>();
    auto element = chrono_types::make_shared<ChElementCableANCF>();
    element->SetNodes(node0, node1);
    element->SetSection(section);

    auto mesh = chrono_types::make_shared<ChMesh>();
    mesh->AddNode(node0);
    mesh->AddNode(node1);
    mesh->AddElement(element);
    sysMBS.Add(mesh);

    fsi.SetBcePattern1D(BcePatternMesh1D::FULL, false);
    fsi.AddFeaMesh(mesh, false);

    // Small fluid block, far from the segment (so that all markers near the origin are BCE markers of the segment)
    fsi.Construct(ChVector3d(0.1, 0.1, 0.1), ChVector3d(1.0, 0, 0), BoxSide::NONE);
    fsi.SetComputationalDomain(ChAABB(ChVector3d(-2, -2, -2), ChVector3d(2, 2, 2)));

    fsi.Initialize();

    // Take one step, so that the sorted marker arrays (built by the proximity search) are available
    fsi.DoStepDynamics(dt);

    // Collect the BCE markers of the segment and their largest distance from the segment center
    double radius = 0.5 * seg_length + 4 * initial_spacing;
    auto pos = sysSPH->GetParticlePositions();
    std::vector<size_t> bce;
    double max_dist = 0;
    for (size_t i = 0; i < pos.size(); i++) {
        double dist = pos[i].Length();
        if (dist > radius)
            continue;
        bce.push_back(i);
        max_dist = std::max(max_dist, dist);
    }
    std::vector<ChVector3d> prev_pos(bce.size());
    for (size_t k = 0; k < bce.size(); k++)
        prev_pos[k] = pos[bce[k]];

    std::cout << "Segment BCE markers: " << bce.size() << "  (max. distance from center: " << max_dist << ")" << std::endl;

    // Rotate the segment and check the displacements of its BCE markers between consecutive configurations.
    // A rigid rotation by angle_incr moves a point at distance r from the rotation center by at most r * angle_incr.
    // The first configuration is the initial one (no displacement expected).
    FsiMeshState state;
    state.pos.resize(2);
    state.vel.assign(2, VNULL);
    state.acc.assign(2, VNULL);
    state.dir.resize(2);
    state.has_node_directions = false;

    double tol = 1e-5;  // marker states are stored in single precision
    int num_failures = 0;
    double max_ratio = 0;

    for (double angle = angle_start; angle <= angle_end + 1e-12; angle += angle_incr) {
        dir = SegmentDirection(angle);
        state.pos[0] = -0.5 * seg_length * dir;
        state.pos[1] = +0.5 * seg_length * dir;
        state.dir[0] = dir;
        state.dir[1] = dir;
        sysSPH->LoadSolidStates({}, {state}, {});
        sysSPH->OnExchangeSolidStates();

        double max_allowed = (angle == angle_start ? 0 : max_dist * angle_incr) + tol;
        pos = sysSPH->GetParticlePositions();
        double max_disp = 0;
        for (size_t k = 0; k < bce.size(); k++) {
            max_disp = std::max(max_disp, (pos[bce[k]] - prev_pos[k]).Length());
            prev_pos[k] = pos[bce[k]];
        }
        max_ratio = std::max(max_ratio, max_disp / max_allowed);

        if (max_disp > max_allowed) {
            std::cout << "FAIL:   angle " << angle * CH_RAD_TO_DEG << " deg (y = " << dir.y() << ")  max. displacement " << max_disp << " > " << max_allowed << std::endl;
            num_failures++;
        }
    }

    std::cout << "Max. ratio of BCE displacement to allowed displacement: " << max_ratio << std::endl;
    std::cout << (bce.size() > 0 && num_failures == 0 ? "PASSED" : "FAILED") << std::endl;
    return bce.size() > 0 && num_failures == 0 ? 0 : 1;
}
