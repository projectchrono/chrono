#pragma once
#include "chrono_parallel/math/real3.h"
#include "chrono_parallel/math/svd.h"
#define one_third 1. / 3
#define two_thirds 2. / 3
#define four_thirds 4. / 3
#define five_thirds 5. / 3
#define one_sixth 1. / 6
#define one_ninth 1. / 9
#define one_twelfth 1. / 12
#define one_twenty_fourth 1. / 24
#define one_twenty_seventh 1. / 27
#define one_sixtieth 1. / 60
#define thirteen_over_twelve 13. / 12
#define root_two Sqrt(2.)
#define root_three Sqrt(3.)
#define root_six Sqrt(6.)
#define root_two_thirds Sqrt(2. / 3)
#define one_over_root_two 1. / Sqrt(2.)
#define one_over_root_three 1. / Sqrt(3.)

namespace chrono {
// Interpolation Functions
real N(const real x) {
    if (Abs(x) < real(1.0)) {
        return real(0.5) * Cube(Abs(x)) - Sqr(x) + two_thirds;
    } else if (Abs(x) < real(2.0)) {
        return -one_sixth * Cube(Abs(x)) + Sqr(x) - real(2.0) * Abs(x) + four_thirds;
    }
    return real(0.0);
}

real N(const real3& X, real inv_grid_dx) {
    return N(X.x * inv_grid_dx) * N(X.y * inv_grid_dx) * N(X.z * inv_grid_dx);
}

real dN(const real x) {
    if (Abs(x) < real(1.0)) {
        return real(1.5) * Sign(x) * Sqr(x) - real(2.0) * x;
    } else if (abs(x) < real(2.0)) {
        return -real(0.5) * Sign(x) * Sqr(x) + real(2.0) * x - real(2.0) * Sign(x);
    }
    return real(0.0);
}

real3 dN(const real3& X, real inv_grid_dx) {
    real3 val = real3();
    real3 T = X * inv_grid_dx;
    val.x = dN(T.x) * inv_grid_dx * N(T.y) * N(T.z);
    val.y = N(T.x) * dN(T.y) * inv_grid_dx * N(T.z);
    val.z = N(T.x) * N(T.y) * dN(T.z) * inv_grid_dx;
    return val;
}

inline int GridCoord(real x, real inv_bin_edge, real minimum) {
    real l = x - minimum;
    int c = Round(l * inv_bin_edge);
    return c;
}

inline int GridHash(int x, int y, int z, const int3& bins_per_axis) {
    return ((z * bins_per_axis.y) * bins_per_axis.x) + (y * bins_per_axis.x) + x;
}
inline real3 NodeLocation(int i, int j, int k, real bin_edge, real3 min_bounding_point) {
    real3 node_location;
    node_location.x = i * bin_edge + min_bounding_point.x;
    node_location.y = j * bin_edge + min_bounding_point.y;
    node_location.z = k * bin_edge + min_bounding_point.z;
    return node_location;
}
#define LOOPOVERNODES(X)                                                                           \
    const int cx = GridCoord(xi.x, inv_bin_edge, min_bounding_point.x);                            \
    const int cy = GridCoord(xi.y, inv_bin_edge, min_bounding_point.y);                            \
    const int cz = GridCoord(xi.z, inv_bin_edge, min_bounding_point.z);                            \
                                                                                                   \
    for (int k = cz - 2; k <= cz + 2; ++k) {                                                       \
        for (int j = cy - 2; j <= cy + 2; ++j) {                                                   \
            for (int i = cx - 2; i <= cx + 2; ++i) {                                               \
                const int current_node = GridHash(i, j, k, bins_per_axis);                         \
                real3 current_node_location = NodeLocation(i, j, k, bin_edge, min_bounding_point); \
                X                                                                                  \
            }                                                                                      \
        }                                                                                          \
    }

Mat33 Potential_Energy_Derivative(const Mat33& FE, const Mat33& FP, real mu, real lambda, real hardening_coefficient) {
    real plastic_determinant = Determinant(FP);
    real elastic_determinant = Determinant(FE);
    real current_mu = mu * exp(hardening_coefficient * (real(1.) - plastic_determinant));
    real current_lambda = lambda * exp(hardening_coefficient * (real(1.) - plastic_determinant));
    Mat33 UE, VE;
    real3 EE;
    SVD(FE, UE, EE, VE);
    Mat33 RE = MultTranspose(UE, VE);
    return real(2.) * current_mu * (FE - RE) +
           current_lambda * elastic_determinant * (elastic_determinant - real(1.)) * InverseTranspose(FE);
}
}
