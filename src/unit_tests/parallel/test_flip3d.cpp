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

//#include "unit_testing.h"

#include "chrono_parallel/math/other_types.h"  // for uint, int2, vec3
#include "chrono_parallel/math/real.h"         // for real
#include "chrono_parallel/math/real3.h"        // for real3
#include "chrono_parallel/math/matrix.h"        // for quaternion, real4
#include "chrono_parallel/solver/ChSolverParallel.h"
#include "chrono_parallel/physics/MPMUtils.h"
#include "chrono_parallel/ChDataManager.h"

using namespace chrono;

using blaze::DynamicMatrix;

real bin_edge = 0.020000;

real3 min_bounding_point = real3(-0.020000, -0.020000, -0.020000);
real3 max_bounding_point = real3(0.040000, 0.040000, 0.040000);

real3 diag = max_bounding_point - min_bounding_point;
vec3 bins_per_axis = vec3(diag / bin_edge);
real inv_bin_edge = real(1.) / bin_edge;
uint grid_size = bins_per_axis.x * bins_per_axis.y * bins_per_axis.z;

real3 gravity = real3(0, 0, -9.80665);
real dt = 0.1;
real rho = .1;
DynamicMatrix<real> A;
DynamicVector<real> density;
ChProjectNone ProjectNone;

ChParallelDataManager* data_manager;

void Print(std::ostream& os, const DynamicMatrix<real>& M) {
    for (size_t i = 0UL; i < (~M).rows(); ++i) {
        os << "";
        for (size_t j = 0UL; j < (~M).columns(); ++j) {
            os << std::setw(1) << (~M)(i, j) << ", ";
        }
        os << "\n";
    }
}

class CH_PARALLEL_API ChShurProductFLIP : public ChShurProduct {
  public:
    ChShurProductFLIP() {}
    virtual ~ChShurProductFLIP() {}

    // Perform the Multiplication
    void operator()(const DynamicVector<real>& v_array, DynamicVector<real>& result_array) {
        // printf("Mult: [%d %d] [%d] [%d]\n", A.rows(), A.columns(), v_array.size(), result_array.size());
        result_array = A * v_array;
    }
};

class CH_PARALLEL_API ChProjectFLIP : public ChProjectConstraints {
  public:
    ChProjectFLIP() {}
    virtual ~ChProjectFLIP() {}

    // Project the Lagrange multipliers
    virtual void operator()(real* data) {
        for (int i = 0; i < grid_size; i++) {
            vec3 g = GridDecode(i, bins_per_axis);
            if ((g.x + 1) >= bins_per_axis.x) {
                data[i] = 0;
            }
            if ((g.y + 1) >= bins_per_axis.y) {
                data[i] = 0;
            }
            if ((g.z + 1) >= bins_per_axis.z) {
                data[i] = 0;
            }
            //            if (density[i] <= 0) {
            //                data[i] = 0;
            //            }
            //            if (data[i] < 0) {
            //                data[i] = 0;
            //            }
        }
    }
};

int main(int argc, char* argv[]) {
    printf("max_bounding_point [%f %f]\n", max_bounding_point.x, max_bounding_point.y);
    printf("min_bounding_point [%f %f]\n", min_bounding_point.x, min_bounding_point.y);
    printf("diag [%f %f] edge:%f \n", diag.x, diag.y, bin_edge);
    printf("Compute DOF [%d] [%d %d %d] [%f]\n", grid_size, bins_per_axis.x, bins_per_axis.y, bins_per_axis.z,
           bin_edge);

    ChSolverParallel* solver = new ChSolverBB();
    data_manager = new ChParallelDataManager();
    data_manager->settings.solver.cache_step_length = false;
    data_manager->settings.solver.solver_mode = SLIDING;
    data_manager->settings.solver.use_power_iteration = false;
    data_manager->system_timer.AddTimer("ChSolverParallel_Solve");
    data_manager->settings.solver.tol_speed = 0;
    solver->Setup(data_manager);

    // 4x4 grid

    DynamicVector<real> v(grid_size * 3);
    DynamicVector<real> hf(grid_size * 3);
    DynamicVector<real> node_mass(grid_size);
    DynamicVector<real> rhs(grid_size);
    DynamicVector<real> pressure(grid_size, 0);

    DynamicMatrix<real> M_inv(grid_size * 3, grid_size * 3, 0);
    DynamicMatrix<real> D_T(grid_size, grid_size * 3, 0);
    density.resize(grid_size);
    node_mass = 1;
    v = 0;
    hf = 0;

    density[GridHash(1, 1, 1, bins_per_axis)] = 15.525000;
    //    density[GridHash(3, 2, bins_per_axis)] = 1;
    //    density[GridHash(2, 3, bins_per_axis)] = 1;
    //    density[GridHash(3, 3, bins_per_axis)] = 1;

    for (int i = 0; i < grid_size; i++) {
        vec3 g = GridDecode(i, bins_per_axis);
        if (density[i] <= 0) {
            v[i * 3 + 0] = 0;
            v[i * 3 + 1] = 0;
            v[i * 3 + 2] = 0;

            hf[i * 3 + 0] = 0;
            hf[i * 3 + 1] = 0;
            hf[i * 3 + 2] = 0;

        } else {
            v[i * 3 + 0] = 0;
            v[i * 3 + 1] = 0;
            v[i * 3 + 2] = 0;

            hf[i * 3 + 0] = 0;
            hf[i * 3 + 1] = 0;
            hf[i * 3 + 2] = 0;
        }
        printf("HERE : [%d %d] [%f %f]\n", g.x, g.y, hf[i * 2 + 0], hf[i * 2 + 1]);
    }

    v[GridHash(1, 1, 1, bins_per_axis) * 3 + 0] = 0.013717;
    //        v[GridHash(3, 2, bins_per_axis) * 2 + 0] = 0.013717;
    //        v[GridHash(4, 2, bins_per_axis) * 2 + 0] = 0.013717;
    //        v[GridHash(2, 3, bins_per_axis) * 2 + 0] = 0.013717;
    //        v[GridHash(3, 3, bins_per_axis) * 2 + 0] = 0.013717;
    //        v[GridHash(4, 3, bins_per_axis) * 2 + 0] = 0.013717;

    // v[GridHash(3, 1, bins_per_axis) * 2 + 0] = -1;
    // v[GridHash(4, 1, bins_per_axis) * 2 + 0] = 1;

    for (int i = 0; i < grid_size; i++) {
        vec3 g = GridDecode(i, bins_per_axis);
        if (density[i] > 0) {
            M_inv(i * 3 + 0, i * 3 + 0) = dt / density[i];
            M_inv(i * 3 + 1, i * 3 + 1) = dt / density[i];
            M_inv(i * 3 + 2, i * 3 + 2) = dt / density[i];
        }
    }

    std::cout << "M_inv:\n";
    Print(std::cout, M_inv);

    for (int i = 0; i < grid_size; i++) {
        vec3 g = GridDecode(i, bins_per_axis);

        int g_left = GridHash(g.x - 1, g.y, g.z, bins_per_axis);
        int g_right = GridHash(g.x + 1, g.y, g.z, bins_per_axis);
        int g_down = GridHash(g.x, g.y - 1, g.z, bins_per_axis);
        int g_up = GridHash(g.x, g.y + 1, g.z, bins_per_axis);
        int g_front = GridHash(g.x, g.y, g.z - 1, bins_per_axis);
        int g_back = GridHash(g.x, g.y, g.z + 1, bins_per_axis);

        bool cell_active = (density[i] != 0);
        if ((g.x + 1) >= bins_per_axis.x) {
        } else if ((g.y + 1) >= bins_per_axis.y) {
        } else if ((g.z + 1) >= bins_per_axis.z) {
        } else {
            if (cell_active) {
                D_T(i, i * 3 + 0) = -1;
                D_T(i, i * 3 + 1) = -1;
                D_T(i, i * 3 + 2) = -1;
                D_T(i, g_right * 3 + 0) = 1;
                D_T(i, g_up * 3 + 1) = 1;
                D_T(i, g_back * 3 + 1) = 1;
            }
        }
    }
    D_T = D_T * inv_bin_edge;

    std::cout << "D_T:\n";
    Print(std::cout, D_T);

    DynamicMatrix<real> D = trans(D_T);
    A = D_T * M_inv * D;

    rhs = D_T * v;
    rhs = -rhs;

    for (int i = 0; i < grid_size; i++) {
        vec3 g = GridDecode(i, bins_per_axis);
        printf("rhs: [%d %d %d ] [%f] [%f %f %f]\n", g.x, g.y, g.z, rhs[i], hf[i * 3 + 0], hf[i * 3 + 1],
               hf[i * 3 + 2]);
    }

    std::cout << "A: \n";
    Print(std::cout, A);

    ChShurProductFLIP Multiply;
    ChProjectFLIP Proj;
    solver->Solve(Multiply, Proj, 100, grid_size, rhs, pressure);

    for (int i = 0; i < data_manager->measures.solver.maxd_hist.size(); i++) {
        printf("Iter hist: %d [%f %f]\n", i, data_manager->measures.solver.maxd_hist[i],
               data_manager->measures.solver.maxdeltalambda_hist[i]);
    }

    DynamicVector<real> force = -D * pressure;

    v = v + M_inv * hf + M_inv * D * pressure;

    for (int i = 0; i < grid_size; i++) {
        vec3 g = GridDecode(i, bins_per_axis);

        // printf("v: [%d %d] [%f %f] [%f] [%f]\n", g.x, g.y, v[i * 2 + 0], v[i * 2 + 1], node_mass[i],
        //  pressure[i]);
        printf("[%d %d %d] v: [%f %f %f] p:[%f ] f:[%f %f %f]\n", g.x, g.y, g.z, v[i * 3 + 0], v[i * 3 + 1],
               v[i * 3 + 2], pressure[i], force[i * 3 + 0], force[i * 3 + 1], force[i * 3 + 2]);
    }
}
