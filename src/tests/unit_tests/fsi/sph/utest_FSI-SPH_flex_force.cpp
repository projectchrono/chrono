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
// Author: Huzaifa
// =============================================================================

#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <tuple>
#include <vector>

#include "gtest/gtest.h"
#include "chrono_fsi/sph/physics/SphBceManager.cuh"

using namespace chrono;
using namespace chrono::fsi::sph;

using Components = std::array<long double, 3>;
Components Values(Real3 v) {
    return {v.x, v.y, v.z};
}

class FlexForce : public ::testing::TestWithParam<std::tuple<IntegrationScheme, uint>> {};

TEST_P(FlexForce, TransferAndRepeatability) {
    auto [scheme, count] = GetParam();
    auto params = chrono_types::make_shared<ChFsiParamsSPH>();
    params->integration_scheme = scheme;
    params->markerMass = Real(2.5);
    FsiDataManager data(params);
    auto& c = *data.countersH;
    c = {};
    c.numFsiNodes1D = 6;
    c.numFsiNodes2D = 8;
    c.numFsiElements1D = c.numFsiElements2D = 3;
    c.numFsiMeshes1D = c.numFsiMeshes2D = 2;
    c.numMesh1DMarkers = c.numMesh2DMarkers = count;
    c.startMesh1DMarkers = 11;
    c.startMesh2DMarkers = c.startMesh1DMarkers + count;
    c.numAllMarkers = c.startMesh2DMarkers + count;
    data.sphMarkers_D->resize(c.numAllMarkers);
    data.bceAcc.resize(c.numAllMarkers);
    data.flex1D_FSIforces_D.assign(c.numFsiNodes1D, mR3(17));
    data.flex2D_FSIforces_D.assign(c.numFsiNodes2D, mR3(17));
    data.fsiMesh1DState_D->Resize(c.numFsiNodes1D, false);
    data.fsiMesh2DState_D->Resize(c.numFsiNodes2D, false);
    thrust::host_vector<Real3> pos1 = {mR3(0, 0, 0), mR3(2, 0, 0), mR3(9, 0, 0), mR3(1, 0, 0), mR3(3, 0, 0), mR3(4, 0, 0)};
    thrust::host_vector<Real3> pos2 = {mR3(0, 0, 0), mR3(0, 1, 0), mR3(9, 0, 0), mR3(1, 0, 0), mR3(1, 1, 0), mR3(2, 0, 0), mR3(3, 0, 0), mR3(2, 1, 0)};
    data.fsiMesh1DState_D->pos = pos1;
    data.fsiMesh2DState_D->pos = pos2;
    data.fsiMesh1DState_D->acc = pos1;
    data.fsiMesh2DState_D->acc = pos2;
    data.flex1D_Nodes_H = {chrono::fsi::sph::mI2(3, 1), chrono::fsi::sph::mI2(0, 3), chrono::fsi::sph::mI2(4, 5)};
    data.flex2D_Nodes_H = {chrono::fsi::sph::mI3(4, 0, 3), chrono::fsi::sph::mI3(1, 3, 0), chrono::fsi::sph::mI3(5, 6, 7)};
    data.flex1D_Normals_H.assign(3, mR3(0, 1, 0));
    for (uint i = 0; i < count; i++) {
        uint element = i % 3;
        auto solid = chrono::fsi::sph::mU3(element == 2 ? 1 : 0, element == 2 ? 0 : element, element);
        data.flex1D_BCEsolids_H.push_back(solid);
        data.flex2D_BCEsolids_H.push_back(solid);
        data.flex1D_BCEcoords_H.push_back(mR3(Real(i % 9) / 8, 0, 0));
        Real u = Real(i % 5) / 4;
        data.flex2D_BCEcoords_H.push_back(mR3(u, (1 - u) * Real(i % 3) / 2, 0));
    }
    SphBceManager bce(data, NodeDirections::NONE, false, true);
    bce.Initialize({});

    std::vector<Components> expected1(c.numFsiNodes1D), expected2(c.numFsiNodes2D);
    std::vector<Components> magnitude1(c.numFsiNodes1D), magnitude2(c.numFsiNodes2D);
    thrust::host_vector<Real4> original(c.numAllMarkers, mR4(99));
    for (uint i = 0; i < count; i++) {
        Real3 force = mR3(Real((int(i % 17) - 8) * 0.1234567), Real(0.31 + (i % 7) * 0.017), Real((int(i % 13) - 6) * 0.04321));
        original[c.startMesh1DMarkers + i] = mR4(force, 0);
        original[c.startMesh2DMarkers + i] = mR4(force, 0);
        auto f = Values(force);
        auto seg = data.flex1D_Nodes_H[i % 3];
        auto tri = data.flex2D_Nodes_H[i % 3];
        Real t = data.flex1D_BCEcoords_H[i].x;
        auto uv = data.flex2D_BCEcoords_H[i];
        std::array<int, 2> nodes1 = {seg.x, seg.y};
        std::array<int, 3> nodes2 = {tri.x, tri.y, tri.z};
        std::array<Real, 2> weights1 = {1 - t, t};
        std::array<Real, 3> weights2 = {uv.x, uv.y, 1 - uv.x - uv.y};
        for (int axis = 0; axis < 3; axis++) {
            long double scale = scheme == IntegrationScheme::IMPLICIT_SPH ? 1 : params->markerMass;
            for (int j = 0; j < 2; j++) {
                auto value = f[axis] * scale * weights1[j];
                expected1[nodes1[j]][axis] += value;
                magnitude1[nodes1[j]][axis] += std::abs(value);
            }
            for (int j = 0; j < 3; j++) {
                auto value = f[axis] * scale * weights2[j];
                expected2[nodes2[j]][axis] += value;
                magnitude2[nodes2[j]][axis] += std::abs(value);
            }
        }
    }

    auto check = [](const thrust::host_vector<Real3>& actual, const std::vector<Components>& expected, const std::vector<Components>& magnitude) {
        ASSERT_EQ(actual.size(), expected.size());
        for (size_t node = 0; node < actual.size(); node++) {
            auto v = Values(actual[node]);
            for (int axis = 0; axis < 3; axis++) {
                ASSERT_TRUE(std::isfinite(v[axis]));
                // Bound rounding over the short per-lane sums and fixed reduction tree, using absolute load sums.
                auto tolerance = 64 * std::numeric_limits<Real>::epsilon() * magnitude[node][axis];
                EXPECT_LE(std::abs(v[axis] - expected[node][axis]), tolerance) << "node " << node << " axis " << axis;
            }
        }
    };
    auto identical = [](const thrust::host_vector<Real3>& a, const thrust::host_vector<Real3>& b) {
        ASSERT_EQ(a.size(), b.size());
        for (size_t i = 0; i < a.size(); i++) {
            EXPECT_EQ(std::memcmp(&a[i].x, &b[i].x, sizeof(Real)), 0);
            EXPECT_EQ(std::memcmp(&a[i].y, &b[i].y, sizeof(Real)), 0);
            EXPECT_EQ(std::memcmp(&a[i].z, &b[i].z, sizeof(Real)), 0);
        }
    };
    thrust::host_vector<Real3> first1, first2;
    for (int order = 0; order < 2; order++) {
        thrust::host_vector<uint> mapping(c.numAllMarkers);
        thrust::host_vector<Real4> sorted(c.numAllMarkers);
        for (uint i = 0; i < c.numAllMarkers; i++) {
            mapping[i] = order == 0 ? c.numAllMarkers - 1 - i : i;
            sorted[mapping[i]] = original[i];
        }
        data.markersProximity_D->mapOriginalToSorted = mapping;
        data.derivVelRhoD = sorted;
        for (int repeat = 0; repeat < 64; repeat++) {
            bce.CalcFeaMesh1DForces();
            bce.CalcFeaMesh2DForces();
            thrust::host_vector<Real3> actual1 = data.flex1D_FSIforces_D;
            thrust::host_vector<Real3> actual2 = data.flex2D_FSIforces_D;
            if (order == 0 && repeat == 0) {
                check(actual1, expected1, magnitude1);
                check(actual2, expected2, magnitude2);
                first1 = actual1;
                first2 = actual2;
            } else {
                identical(actual1, first1);
                identical(actual2, first2);
            }
        }
        bce.updateBCEAcc();
        thrust::host_vector<Real3> acc = data.bceAcc;
        for (uint i = 0; i < count; i++) {
            auto seg = data.flex1D_Nodes_H[i % 3];
            Real t = data.flex1D_BCEcoords_H[i].x;
            auto expected = Values(pos1[seg.x] * (1 - t) + pos1[seg.y] * t);
            auto actual = Values(acc[mapping[c.startMesh1DMarkers + i]]);
            // Exact comparison is safe: node accelerations are small integers and t = k/8, so every product and sum
            // is exactly representable and the result does not depend on how the interpolation is evaluated.
            EXPECT_EQ(actual, expected);
        }
    }
    data.derivVelRhoD.assign(c.numAllMarkers, mR4(0));
    bce.CalcFeaMesh1DForces();
    bce.CalcFeaMesh2DForces();
    check(data.flex1D_FSIforces_D, std::vector<Components>(c.numFsiNodes1D), std::vector<Components>(c.numFsiNodes1D));
    check(data.flex2D_FSIforces_D, std::vector<Components>(c.numFsiNodes2D), std::vector<Components>(c.numFsiNodes2D));
}

INSTANTIATE_TEST_SUITE_P(Sph, FlexForce, ::testing::Combine(::testing::Values(IntegrationScheme::RK2, IntegrationScheme::IMPLICIT_SPH), ::testing::Values(0u, 1u, 257u, 1031u)));
