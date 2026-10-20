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
// Unit test for the node directions of 1-D FEA meshes calculated on the device
// from the adjacent segment directions (NodeDirectionsMode::AVERAGE).
// With node directions, the center-line of each segment is a cubic Hermite curve
// (with tangents equal to the unit node directions scaled by the segment length)
// and the BCE markers on the center-line are placed on that curve.
// The cable elements are added to the mesh out of order, so that the node indices
// of consecutive segments are not consecutive, and the segments have different
// lengths. The positions of the center-line BCE markers are compared against a
// host calculation of the Hermite curves.
//
// =============================================================================

#include <cmath>
#include <iostream>
#include <map>
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

int num_failures = 0;

// Node positions on a circular arc in the x-z plane (center at origin), with unequal spacing
std::vector<ChVector3d> ArcPoints(double radius, double angle_span) {
    std::vector<double> fractions = {0.0, 0.15, 0.45, 0.6, 1.0};
    std::vector<ChVector3d> points;
    for (auto f : fractions) {
        double angle = -angle_span / 2 + f * angle_span;
        points.push_back(ChVector3d(radius * std::sin(angle), 0, radius * std::cos(angle)));
    }
    return points;
}

// Check that each point on the center-lines of the segments (at the BCE segment coordinates) has a BCE marker.
// Segment i connects chain nodes seg[i].x() and seg[i].y(); 'pos' holds the chain node positions.
void CheckCenterline(const std::string& name,
                     ChFsiFluidSystemSPH& sysSPH,
                     const std::vector<ChVector2i>& segs,
                     const std::vector<int>& num_div,
                     const std::vector<ChVector3d>& pos) {
    // Node directions: normalized sums of the adjacent unit segment directions
    std::vector<ChVector3d> dir(pos.size(), VNULL);
    for (const auto& s : segs) {
        auto d = (pos[s.y()] - pos[s.x()]).GetNormalized();
        dir[s.x()] += d;
        dir[s.y()] += d;
    }
    for (auto& d : dir)
        d.Normalize();

    auto markers = sysSPH.GetParticlePositions();

    double max_err = 0;
    size_t num_checked = 0;
    for (size_t is = 0; is < segs.size(); is++) {
        const auto& P0 = pos[segs[is].x()];
        const auto& P1 = pos[segs[is].y()];
        double len = (P1 - P0).Length();
        auto D0 = dir[segs[is].x()] * len;  // Hermite tangents
        auto D1 = dir[segs[is].y()] * len;
        int n = num_div[is];
        for (int i = 0; i <= n; i++) {
            double t = double(i) / n;
            double t2 = t * t;
            double t3 = t2 * t;
            auto P = P0 * (2 * t3 - 3 * t2 + 1) + P1 * (-2 * t3 + 3 * t2) + D0 * (t3 - 2 * t2 + t) + D1 * (t3 - t2);

            double min_dist = 1e10;
            for (const auto& m : markers)
                min_dist = std::min(min_dist, (m - P).Length());
            max_err = std::max(max_err, min_dist);
            num_checked++;
        }
    }

    // Marker states are stored in single precision
    bool ok = num_checked > 0 && max_err < 1e-5;
    std::cout << (ok ? "  ok:   " : "FAIL:   ") << name << "  (" << num_checked << " center-line points, max. distance to nearest BCE marker " << max_err << ")" << std::endl;
    if (!ok)
        num_failures++;
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

    // Cable along a circular arc (chain of 5 nodes and 4 segments of different lengths)
    auto pos0 = ArcPoints(0.2, CH_PI_2);
    int num_nodes = (int)pos0.size();

    auto mesh = chrono_types::make_shared<ChMesh>();
    std::vector<std::shared_ptr<ChNodeFEAxyzD>> nodes;
    for (const auto& p : pos0) {
        auto node = chrono_types::make_shared<ChNodeFEAxyzD>(p, ChVector3d(1, 0, 0));
        mesh->AddNode(node);
        nodes.push_back(node);
    }

    // Add the cable elements out of order (each element oriented along the chain).
    // The FSI node indices are assigned in the order in which nodes are first encountered when traversing the segments.
    std::vector<ChVector2i> segs = {{2, 3}, {0, 1}, {3, 4}, {1, 2}};
    auto section = chrono_types::make_shared<ChBeamSectionCableANCF>();
    for (const auto& s : segs) {
        auto element = chrono_types::make_shared<ChElementCableANCF>();
        element->SetNodes(nodes[s.x()], nodes[s.y()]);
        element->SetSection(section);
        mesh->AddElement(element);
    }
    sysMBS.Add(mesh);

    std::map<int, int> chain2fsi;
    for (const auto& s : segs) {
        chain2fsi.insert({s.x(), (int)chain2fsi.size()});
        chain2fsi.insert({s.y(), (int)chain2fsi.size()});
    }

    // Number of BCE divisions on each segment (based on the initial segment lengths)
    std::vector<int> num_div;
    for (const auto& s : segs)
        num_div.push_back((int)std::ceil((pos0[s.y()] - pos0[s.x()]).Length() / initial_spacing));

    fsi.UseNodeDirections(NodeDirectionsMode::AVERAGE);
    fsi.SetBcePattern1D(BcePatternMesh1D::FULL, false);
    fsi.AddFeaMesh(mesh, false);

    // Small fluid block, far from the cable
    fsi.Construct(ChVector3d(0.1, 0.1, 0.1), ChVector3d(1.0, 0, 0), BoxSide::NONE);
    fsi.SetComputationalDomain(ChAABB(ChVector3d(-2, -2, -2), ChVector3d(2, 2, 2)));

    fsi.Initialize();

    // Take one step, so that the sorted marker arrays (built by the proximity search) are available
    fsi.DoStepDynamics(dt);

    // Load mesh states (positions in FSI node order) and check the center-line BCE markers
    auto check = [&](const std::string& name, const std::vector<ChVector3d>& pos) {
        FsiMeshState state;
        state.pos.resize(num_nodes);
        state.vel.assign(num_nodes, VNULL);
        state.acc.assign(num_nodes, VNULL);
        state.dir.assign(num_nodes, ChVector3d(1, 0, 0));
        state.has_node_directions = false;
        for (int i = 0; i < num_nodes; i++)
            state.pos[chain2fsi.at(i)] = pos[i];
        sysSPH->LoadSolidStates({}, {state}, {});
        sysSPH->OnExchangeSolidStates();
        CheckCenterline(name, *sysSPH, segs, num_div, pos);
    };

    check("initial configuration", pos0);
    check("deformed configuration", ArcPoints(0.12, 150 * CH_DEG_TO_RAD));

    std::cout << (num_failures == 0 ? "PASSED" : "FAILED") << std::endl;
    return num_failures == 0 ? 0 : 1;
}
