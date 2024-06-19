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
// Chrono::Multicore unit test for Blaze
// =============================================================================

#include <cstdio>
#include <vector>
#include <cmath>
#include <random>
#include <algorithm>
#include <tuple>

#include "chrono_multicore/ChDataManager.h"
#include "unit_testing.h"

using namespace chrono;
std::vector<std::tuple<int, int, chrono::real> > data;

template <typename T>
static void inline SetCheck(T& M, const int row, const int col, const real& a) {
    //	if(M.find(row, col) != M.end(row)){
    //		printf("Fail Assert %d %d %f\n", row, col, a);
    //		exit(1);
    //	}
    if (a != 0.0) {
        M.set(row, col, a);
    }
}

typedef std::tuple<int, int, chrono::real> mytuple;

bool mycompare(const mytuple& lhs, const mytuple& rhs) {
    if (std::get<0>(lhs) < std::get<0>(rhs)) {
        return 1;
    } else if (std::get<0>(lhs) == std::get<0>(rhs)) {
        return std::get<1>(lhs) < std::get<1>(rhs);
    }
    return 0;
}

int main(int argc, char* argv[]) {
    int rows = 1000000;  // 1068197;
    int cols = 1000000;  // 3150835;
    int cap = 1452233;

    CompressedMatrix<chrono::real> D;
    data.resize(cap);
    // Seed with a real random value, if available
    std::random_device rd;
    std::default_random_engine e1(rd());
    std::uniform_int_distribution<int> row_dist(0, rows - 1);
    std::uniform_int_distribution<int> col_dist(0, cols - 1);
    std::uniform_real_distribution<chrono::real> data_dist(-100, 100);

    //

    // for (int test = 0; test < 10000; test++) {
    D.clear();
    D.resize(rows, cols);
    D.reserve(cap);
    for (int i = 0; i < cap; i++) {
        data[i] = std::make_tuple(row_dist(e1), col_dist(e1), data_dist(e1));
    }
    sort(data.begin(), data.end(), mycompare);
    // Append
    for (int i = 0; i < cap; i++) {
        std::tuple<int, int, chrono::real> T = data[i];
        if (i > 0 && std::get<0>(data[i]) != std::get<0>(data[i - 1])) {
            D.finalize(std::get<0>(data[i - 1]));
        }
        D.append(std::get<0>(T), std::get<1>(T), 0);
    }
    // Set
    for (int i = 0; i < cap; i++) {
        std::tuple<int, int, chrono::real> T = data[i];

        if (D.find(std::get<0>(T), std::get<1>(T)) != D.end(std::get<0>(T))) {
            printf("Fail Assert %d %d %f %d\n", std::get<0>(T), std::get<1>(T), std::get<2>(T), i);

            //			for (int j = 0; j < cap; j++) {
            //				std::tuple<int, int, chrono::real> T = data[j];
            //				printf("[%d %d] %f %d\n", std::get < 0 > (T), std::get < 1 > (T), std::get < 2 > (T), j);
            //			}
            //			exit(1);
        }
        if (std::get<2>(T) != 0.0) {
            D.set(std::get<0>(T), std::get<1>(T), std::get<2>(T));
        }
        // SetCheck(D,std::get < 0 > (T), std::get < 1 > (T), std::get < 2 > (T));
    }
    //}
}
