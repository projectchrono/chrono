// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Hammad Mazhar
// =============================================================================
//
// Chrono::Multicore unit test for MPR collision detection
// =============================================================================

#include <cstdio>
#include <vector>
#include <cmath>

#include "unit_testing.h"

#include "chrono_multicore/cuda/ChMPMUtils.h"

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

    vec3 bins_per_axis = vec3(diag / (radius * 2));

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
        WeakEqual(N(point_a - NodeLocation(7, 7, 7, bin_edge, min_bounding_point), inv_bin_edge), 0.1822061256,
                  C_EPSILON);
    }
    {  // Each node should have 125 surrounding nodes
        int ind = 0;
        real3 xi = point_a;
        printf("Grid Hash Count\n");
        LOOPOVERNODES(real3 p = point_a - current_node_location;
                      // printf("[%d %d %d][%d %d] N:%f\n", i, j, k, current_node, ind, N(p, inv_bin_edge));  //
                      ind++;  //
        )
        WeakEqual(ind, 125);
    }

    real youngs_modulus = (real)1.4e5;
    real poissons_ratio = (real).2;
    real theta_c = (real)2.5e-2;
    real theta_s = (real)7.5e-3;
    real lambda =
        youngs_modulus * poissons_ratio / (((real)1. + poissons_ratio) * ((real)1. - (real)2. * poissons_ratio));
    real mu = youngs_modulus / ((real)2. * ((real)1. + poissons_ratio));
    real initial_density = (real)4e2;
    real alpha = (real).95;
    real hardening_coefficient = (real)10.;

    {
        printf("Potential_Energy_Derivative\n");
        const Mat33 FE = Mat33(0.8147, 0.9058, 0.1270, 0.9134, 0.6324, .0975, 0.2785, 0.5469, 0.9575) * 100;
        const Mat33 FP = Mat33(1, 0, 5, 2, 1, 6, 3, 4, 0) * (.001);
        Mat33 PED = Potential_Energy_Derivative(FE, FP, mu, lambda, hardening_coefficient);
        // Print(PED, "PED");
    }

    {
        printf("Rotational_Derivative\n");
        const Mat33 FE = Mat33(0.8147, 0.9058, 0.1270, 0.9134, 0.6324, .0975, 0.2785, 0.5469, 0.9575);
        const Mat33 FP = Mat33(1, 0, 5, 2, 1, 6, 3, 4, 0);
        Mat33 RD = Rotational_Derivative(FE, FP);

        // Print(RD, "RD");
        WeakEqual(RD,
                  Mat33(-0.4388320607360907121829996, -2.5644905725011524211254255, 2.6523953516380869288582289,
                        3.3403002881859262807040523, -0.3555579545919572703738254, -0.1151537247848418710205465,
                        -1.6067063031830095543028847, 0.5767590099191256536315109, -0.3543936405970238290308316),
                  1e-6);
    }
    {
        printf("Rotational_Derivative_Simple\n");
        const Mat33 FE = Mat33(0.8147, 0.9058, 0.1270, 0.9134, 0.6324, .0975, 0.2785, 0.5469, 0.9575);
        const Mat33 FP = Mat33(1, 0, 5, 2, 1, 6, 3, 4, 0);
        Mat33 U, V, R, S, W;
        real3 E;
        SVD(FE, U, E, V);
        // Perform polar decomposition F = R*S
        R = MultTranspose(U, V);
        S = V * MultTranspose(Mat33(E), V);
        // See tech report end of page 2
        W = TransposeMult(R, FP);

        Mat33 RD1 = Rotational_Derivative_Simple(R, S, FP);

        Mat33 RD2 = Rotational_Derivative(FE, FP);

        //        Print(RD1,"RD1");
        //        Print(RD2,"RD2");

        WeakEqual(RD1, RD2, 1e-6);
    }
}
