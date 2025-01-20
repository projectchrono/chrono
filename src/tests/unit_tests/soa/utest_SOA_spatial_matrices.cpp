// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 projectchrono.org
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
// Tests for miscellaneous linear algebra operations with spatial vectors and
// matrices.
//
// =============================================================================

#include <iomanip>

#include "chrono/soa/ChSpatial.h"

#include "gtest/gtest.h"

using std::cout;
using std::endl;
using namespace chrono;
using namespace chrono::soa;

//#define DBG_PRINT

constexpr double ABS_ERR = 1e-10;

// -----------------------------------------------------------------------------

// Types encapsulating a constexpr (which can be used as template parameter)
struct ONE {
    static constexpr int DOF = 1;
};

struct TWO {
    static constexpr int DOF = 2;
};

struct THREE {
    static constexpr int DOF = 3;
};

struct FOUR {
    static constexpr int DOF = 4;
};

// -----------------------------------------------------------------------------

// Define the suite of typed tests (for different number of DOFs)
using DOF_list = testing::Types<ONE, TWO, THREE, FOUR>;
template <class>
struct SOA_linalg : testing::Test {};
TYPED_TEST_SUITE(SOA_linalg, DOF_list);

// -----------------------------------------------------------------------------

TYPED_TEST(SOA_linalg, matrix_construction) {
    constexpr int DOF = TypeParam::DOF;

#ifdef DBG_PRINT
    cout.setf(std::ios::fixed, std::ios::floatfield | std::ios::showpos);
    cout.width(12);
    cout.precision(6);
#endif

    // Spatial matrix from blocks and equivalent 6x6 Eigen matrix
    ChSpatialMat S(ChMatrix33d::Random(), ChMatrix33d::Random(), ChMatrix33d::Random(), ChMatrix33d::Random());
    ChMatrixNM<double, 6, 6> S_e = S.eigen();
#ifdef DBG_PRINT
    cout << "\nSpatial matrix\n";
    cout << S << endl;
    cout << S_e << endl;
#endif

    // Shift matrix from 3x1 vector and equivalent 6x6 Eigen matrix
    ChShiftMat P(ChVector3d(1, 2, 3));
    ChMatrixNM<double, 6, 6> P_e = P.eigen();
#ifdef DBG_PRINT
    cout << "\nShift matrix\n";
    cout << P << endl;
    cout << P_e << endl;
#endif

    // Velocity matrix from blocks
    ChVelMat<DOF> H(ChMatrix3N<DOF>::Random(), ChMatrix3N<DOF>::Random());
    ChMatrix6N<DOF> H_e = H.eigen();
#ifdef DBG_PRINT
    cout << "\nVelocity matrix\n";
    cout << H << endl;
    cout << H_e << endl;
#endif

    // Construct a symmetric matrix
    ChMatrixNN<DOF> D;
    D.setRandom();
    D = D.selfadjointView<Eigen::Lower>();
#ifdef DBG_PRINT
    cout << "\nSymmetric matrix\n";
    cout << D << endl;
#endif

    // Test construction of shifted spatial matrix
    auto S1 = ChSpatialMat::constructFrom(S, P);
    auto S1_e = P_e.transpose() * S_e * P_e;
#ifdef DBG_PRINT
    cout << "\nShifted spatial matrix\n";
    cout << S1 << endl;
    cout << S1_e << endl;
#endif
    ASSERT_TRUE(S1.A00().equals(S1_e.block(0, 0, 3, 3), ABS_ERR));
    ASSERT_TRUE(S1.A01().equals(S1_e.block(0, 3, 3, 3), ABS_ERR));
    ASSERT_TRUE(S1.A10().equals(S1_e.block(3, 0, 3, 3), ABS_ERR));
    ASSERT_TRUE(S1.A11().equals(S1_e.block(3, 3, 3, 3), ABS_ERR));

    // Test construction of spatial matrix from symmetric velocity transform
    auto S2 = ChSpatialMat::constructFrom(H, D);
    auto S2_e = H_e * D * H_e.transpose();
#ifdef DBG_PRINT
    cout << "\nSpatial matrix from symmetric vel transform\n";
    cout << S2 << endl;
    cout << S2_e << endl;
#endif
    ASSERT_TRUE(S2.A00().equals(S2_e.block(0, 0, 3, 3), ABS_ERR));
    ASSERT_TRUE(S2.A01().equals(S2_e.block(0, 3, 3, 3), ABS_ERR));
    ASSERT_TRUE(S2.A10().equals(S2_e.block(3, 0, 3, 3), ABS_ERR));
    ASSERT_TRUE(S2.A11().equals(S2_e.block(3, 3, 3, 3), ABS_ERR));
}

// -----------------------------------------------------------------------------

TYPED_TEST(SOA_linalg, SOA_multiplication) {
    constexpr int DOF = TypeParam::DOF;

    ChSpatialVec V(ChVector6::Random());
    ChSpatialMat S(ChMatrix33d::Random(), ChMatrix33d::Random(), ChMatrix33d::Random(), ChMatrix33d::Random());
    ChShiftMat P(ChVector3d(1, 2, 3));

    ChVelMat<DOF> H1(ChMatrix3N<DOF>::Random(), ChMatrix3N<DOF>::Random());
    ChVelMat<DOF> H2(ChMatrix3N<DOF>::Random(), ChMatrix3N<DOF>::Random());
    ChVectorN<double, DOF> v = ChVectorN<double, DOF>::Random();
    ChMatrixNN<DOF> M = ChMatrixNN<DOF>::Random();
    ChMatrix33d R = ChMatrix33d::Random();

    {
        auto sv = (S * V).eigen();
        auto sv_e = S.eigen() * V.eigen();
        ASSERT_TRUE(sv.equals(sv_e, ABS_ERR));
    }
    {
        auto pv = (P * V).eigen();
        auto pv_e = P.eigen() * V.eigen();
        ASSERT_TRUE(pv.equals(pv_e, ABS_ERR));
    }
    {
        auto pv = (~P * V).eigen();
        auto pv_e = P.eigen().transpose() * V.eigen();
        ASSERT_TRUE(pv.equals(pv_e, ABS_ERR));
    }
    {
        auto sp = (S * P).eigen();
        auto sp_e = S.eigen() * P.eigen();
        ASSERT_TRUE(sp.equals(sp_e, ABS_ERR));
    }
    {
        auto ps = (~P * S).eigen();
        auto ps_e = P.eigen().transpose() * S.eigen();
        ASSERT_TRUE(ps.equals(ps_e, ABS_ERR));
    }
    {
        auto hv = (H1 * v).eigen();
        auto hv_e = H1.eigen() * v;
        ASSERT_TRUE(hv.equals(hv_e, ABS_ERR));
    }
    {
        auto hv = ~H1 * V;
        auto hv_e = H1.eigen().transpose() * V.eigen();
        ASSERT_TRUE(hv.equals(hv_e, ABS_ERR));
    }
    {
        ChMatrix6N<DOF> hm = (H1 * M).eigen();
        ChMatrix6N<DOF> hm_e = H1.eigen() * M;
//#ifdef DBG_PRINT
        cout << "-----------  eigen(H * M)" << endl;
        cout << hm << endl;
        cout << "-----------  eigen(H) * M" << endl;
        cout << hm_e << endl;
        cout << "-----------  D = eigen(H * M) - eigen(H) * M" << endl;
        ChMatrix6N<DOF> foo = hm - hm_e;
        cout << foo << endl;
        cout << "-----------  ||D|| = " << foo.norm() << endl;
//#endif
        ASSERT_NEAR((hm - hm_e).norm(), 0.0, ABS_ERR);
        ASSERT_TRUE(hm.equals(hm_e, ABS_ERR));
    }
    {
        auto sh = (S * H1).eigen();
        auto sh_e = S.eigen() * H1.eigen();
        ASSERT_TRUE(sh.equals(sh_e, ABS_ERR));
    }
    ////{ auto rh = (R * H1).eigen(); }
    {
        auto hh = (H1 * ~H2).eigen();
        auto hh_e = H1.eigen() * H2.eigen().transpose();
        ASSERT_TRUE(hh.equals(hh_e, ABS_ERR));
    }
    {
        auto hh = ~H1 * H2;
        auto hh_e = H1.eigen().transpose() * H2.eigen();
        ASSERT_TRUE(hh.equals(hh_e, ABS_ERR));
    }
}
