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
// Author: Abhinov Koutharapu
// =============================================================================

#include <cstdlib>
#include <vector>

#include <benchmark/benchmark.h>

#include "chrono/core/ChVector3.h"
#include "chrono/geometry/ChDelaunay2D.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"

using namespace chrono;

class DelaunayFixture : public ::benchmark::Fixture {
  public:
    void SetUp(const ::benchmark::State& st) override {
        const int num_points = static_cast<int>(st.range(0));
        points.clear();
        points.reserve(num_points);

        std::srand(42);
        for (int i = 0; i < num_points; i++) {
            const double x = static_cast<double>(std::rand()) / RAND_MAX;
            const double y = static_cast<double>(std::rand()) / RAND_MAX;
            points.emplace_back(x, y, 0.0);
        }
    }

    void TearDown(const ::benchmark::State&) override { points.clear(); }

    std::vector<ChVector3d> points;
};

BENCHMARK_DEFINE_F(DelaunayFixture, Triangulate)(benchmark::State& st) {
    ChTriangleMeshConnected mesh;
    for (auto _ : st) {
        ChDelaunay2D::Triangulate(points, mesh);
        benchmark::DoNotOptimize(mesh);
    }
    st.SetItemsProcessed(st.iterations() * static_cast<int64_t>(points.size()));
}
BENCHMARK_REGISTER_F(DelaunayFixture, Triangulate)->Unit(benchmark::kMillisecond)->Arg(50)->Arg(100)->Arg(500);

////BENCHMARK_MAIN();