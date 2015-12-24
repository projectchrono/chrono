// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Hammad Mazhar
// =============================================================================
//
// ChronoParallel unit test for MPR collision detection
// =============================================================================

#include <stdio.h>
#include <vector>
#include <cmath>

#include "unit_testing.h"

#include "chrono_parallel/lcp/MPMUtils.h"

using namespace chrono;

int main(int argc, char* argv[]) {
    real3 min_bounding_point(-1, -1, -1);
    real3 max_bounding_point(1, 1, 1);

    real radius = 0.25;

    max_bounding_point = real3(Max(Ceil(max_bounding_point.x), Ceil(max_bounding_point.x + radius * 6)),
                               Max(Ceil(max_bounding_point.y), Ceil(max_bounding_point.y + radius * 6)),
                               Max(Ceil(max_bounding_point.z), Ceil(max_bounding_point.z + radius * 6)));

    min_bounding_point = real3(Min(Floor(min_bounding_point.x), Floor(min_bounding_point.x - radius * 6)),
                               Min(Floor(min_bounding_point.y), Floor(min_bounding_point.y - radius * 6)),
                               Min(Floor(min_bounding_point.z), Floor(min_bounding_point.z - radius * 6)));

    real3 diag = max_bounding_point - min_bounding_point;

    int3 bins_per_axis = int3(diag / (radius * 2));

    real3 point_a = real3(0.44236, 0.65093, 0.24482);
    real3 point_b = real3(0.63257, 0.83347, 0.74071);
    real3 point_c = real3(0.82623, 0.92491, 0.92109);

    const real bin_edge = radius * 2;
    const real inv_bin_edge = real(1) / bin_edge;

    printf("bins_per_axis [%d %d %d]\n", bins_per_axis.x, bins_per_axis.y, bins_per_axis.z);
    printf("max_bounding_point [%f %f %f]\n", max_bounding_point.x, max_bounding_point.y, max_bounding_point.z);
    printf("min_bounding_point [%f %f %f]\n", min_bounding_point.x, min_bounding_point.y, min_bounding_point.z);

    {
        printf("Grid Coord\n");
        int x = GridCoord(point_a.x, inv_bin_edge, min_bounding_point.x);
        int y = GridCoord(point_a.y, inv_bin_edge, min_bounding_point.y);
        int z = GridCoord(point_a.z, inv_bin_edge, min_bounding_point.z);

        WeakEqual(x, 7);
        WeakEqual(y, 7);
        WeakEqual(z, 6);
    }
    {
        printf("Current Node\n");
        WeakEqual(NodeLocation(5, 5, 4, bin_edge, min_bounding_point), real3(-.5, -.5, -1));
        WeakEqual(NodeLocation(9, 9, 8, bin_edge, min_bounding_point), real3(1.5, 1.5, 1));
    }
    {
        printf("N\n");
        WeakEqual(N(point_a - NodeLocation(5, 5, 4, bin_edge, min_bounding_point), inv_bin_edge), 0);
        WeakEqual(N(point_a - NodeLocation(7, 7, 7, bin_edge, min_bounding_point), inv_bin_edge), 0.1822061256);
    }
    {
        int ind = 0;
        real3 xi = point_a;
        printf("Grid Hash Count\n");
        LOOPOVERNODES(real3 p = point_a - current_node_location;
                      printf("[%d %d %d][%d %d] N:%f\n", i, j, k, current_node, ind, N(p, inv_bin_edge)); ind++;)
        WeakEqual(ind, 125);
    }
}
