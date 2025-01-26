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

#include "chrono/core/ChRotation.h"
#include "chrono/soa/ChSpatial.h"

#include "gtest/gtest.h"

using std::cout;
using std::endl;
using namespace chrono;
using namespace chrono::soa;

// #define DBG_PRINT

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
    ChSpatialMat S(ChMatrix33d::Random(3, 3), ChMatrix33d::Random(3, 3), ChMatrix33d::Random(3, 3),
                   ChMatrix33d::Random(3, 3));
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
    ChVelMat<DOF> H(ChMatrix3N<DOF>::Random(3, DOF), ChMatrix3N<DOF>::Random(3, DOF));
    ChMatrix6N<DOF> H_e = H.eigen();
#ifdef DBG_PRINT
    cout << "\nVelocity matrix\n";
    cout << H << endl;
    cout << H_e << endl;
#endif

    // Construct a symmetric matrix
    ChMatrixNN<DOF> D;
    D.setRandom();
    D = D.template selfadjointView<Eigen::Lower>();
#ifdef DBG_PRINT
    cout << "\nSymmetric matrix\n";
    cout << D << endl;
#endif

    // Test construction of shifted spatial matrix
    ChSpatialMat S1 = ChSpatialMat::constructFrom(S, P);
    ChMatrix66d S1_e = P_e.transpose() * S_e * P_e;
#ifdef DBG_PRINT
    cout << "\nShifted spatial matrix\n";
    cout << S1 << endl;
    cout << S1_e << endl;
#endif
    ASSERT_TRUE(S1.A00().isApprox(S1_e.block(0, 0, 3, 3), ABS_ERR));
    ASSERT_TRUE(S1.A01().isApprox(S1_e.block(0, 3, 3, 3), ABS_ERR));
    ASSERT_TRUE(S1.A10().isApprox(S1_e.block(3, 0, 3, 3), ABS_ERR));
    ASSERT_TRUE(S1.A11().isApprox(S1_e.block(3, 3, 3, 3), ABS_ERR));

    // Test construction of spatial matrix from symmetric velocity transform
    ChSpatialMat S2 = ChSpatialMat::constructFrom(H, D);
    ChMatrix66d S2_e = H_e * D * H_e.transpose();
#ifdef DBG_PRINT
    cout << "\nSpatial matrix from symmetric vel transform\n";
    cout << S2 << endl;
    cout << S2_e << endl;
#endif
    ASSERT_TRUE(S2.A00().isApprox(S2_e.block(0, 0, 3, 3), ABS_ERR));
    ASSERT_TRUE(S2.A01().isApprox(S2_e.block(0, 3, 3, 3), ABS_ERR));
    ASSERT_TRUE(S2.A10().isApprox(S2_e.block(3, 0, 3, 3), ABS_ERR));
    ASSERT_TRUE(S2.A11().isApprox(S2_e.block(3, 3, 3, 3), ABS_ERR));
}

// -----------------------------------------------------------------------------

TYPED_TEST(SOA_linalg, SOA_multiplication) {
    constexpr int DOF = TypeParam::DOF;

    ChSpatialVec V(ChVector3d(1, -2, 3), ChVector3d(0.3, 0.2, -0.3));

    ChMatrix33d A00(ChVector3d(+0.1, +0.2, +0.3), ChVector3d(-0.4, +0.5, -0.6), ChVector3d(+1.0, +2.0, +3.0));
    ChMatrix33d A01(ChVector3d(+0.1, -0.2, +0.3), ChVector3d(+1.0, -2.0, +3.0), ChVector3d(-1.0, +2.0, -3.0));
    ChMatrix33d A10(ChVector3d(-0.4, +0.5, -0.6), ChVector3d(-0.4, -0.5, -0.6), ChVector3d(+0.1, +0.2, -0.3));
    ChMatrix33d A11(ChVector3d(-0.1, -0.2, -0.6), ChVector3d(-0.5, +0.5, +0.1), ChVector3d(-0.4, +0.2, +0.3));
    ChSpatialMat S(A00, A01, A10, A11);

    ChShiftMat P(ChVector3d(1, 2, 3));

    ChVelMat<DOF> H1;
    ChVelMat<DOF> H2;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < DOF; j++) {
            H1.ang()(i, j) = 0.1 + 2 * i - 3 * j;
            H1.lin()(i, j) = 0.1 - 2 * i + 3 * j;
            H2.ang()(i, j) = -0.1 + 3 * i - 2 * j;
            H2.lin()(i, j) = -0.1 - 3 * i + 2 * j;
        }
    }

    ChVectorN<double, DOF> v;
    for (int j = 0; j < DOF; j++) {
        v(j) = -0.3 + 2 * j;
    }

    ChMatrixNN<DOF> M;
    for (int i = 0; i < DOF; i++) {
        for (int j = 0; j < DOF; j++) {
            M(i, j) = 1.0 + (i + 0.1) * (j + 0.2);
        }
    }

    ChMatrix33d R(QuatFromAngleX(CH_PI / 6));

#ifdef DBG_PRINT
    cout << "========================================" << endl;
    cout << "   DOF = " << DOF << endl;
    cout << "========================================" << endl;
    cout << "V (ChSpatialVec)" << endl;
    cout << V << endl;
    cout << "S (ChSpatialMat)" << endl;
    cout << S << endl;
    cout << "S.eigen()" << endl;
    cout << S.eigen() << endl;
    cout << "P (ChShiftMat)" << endl;
    cout << P << endl;
    cout << "P.eigen()" << endl;
    cout << P.eigen() << endl;
    cout << "H1 (ChVelMat)" << endl;
    cout << H1 << endl;
    cout << "H1.eigen()" << endl;
    cout << H1.eigen() << endl;
    cout << "H2 (ChVelMat)" << endl;
    cout << H2 << endl;
    cout << "H2.eigen()" << endl;
    cout << H2.eigen() << endl;
    cout << "v (ChVectorN<double, DOF>)" << endl;
    cout << v << endl;
    cout << "M (ChMatrixNN<DOF>)" << endl;
    cout << M << endl;
    cout << "========================================" << endl;
#endif

    {
        ChSpatialVec sv = S * V;
        ChVector6 sv_e = S.eigen() * V;
#ifdef DBG_PRINT
        cout << "\nS * V" << endl;
        cout << sv << endl;
        cout << "---" << endl;
        cout << sv_e << endl;
        cout << "||diff|| = " << (sv - sv_e).norm() << endl;
#endif
        ASSERT_NEAR((sv - sv_e).norm(), 0.0, ABS_ERR);
        ////ASSERT_TRUE(sv.isApprox(sv_e, ABS_ERR));
    }
    {
        ChSpatialVec pv = P * V;
        ChVector6 pv_e = P.eigen() * V;
#ifdef DBG_PRINT
        cout << "\nP * V" << endl;
        cout << pv << endl;
        cout << "---" << endl;
        cout << pv_e << endl;
        cout << "||diff|| = " << (pv - pv_e).norm() << endl;
#endif
        ASSERT_NEAR((pv - pv_e).norm(), 0.0, ABS_ERR);
        ////ASSERT_TRUE(pv.isApprox(pv_e, ABS_ERR));
    }
    {
        ChSpatialVec pv = ~P * V;
        ChVector6 pv_e = P.eigen().transpose() * V;
#ifdef DBG_PRINT
        cout << "\nP^T * V" << endl;
        cout << pv << endl;
        cout << "---" << endl;
        cout << pv_e << endl;
        cout << "||diff|| = " << (pv - pv_e).norm() << endl;
#endif
        ASSERT_NEAR((pv - pv_e).norm(), 0.0, ABS_ERR);
        ////ASSERT_TRUE(pv.isApprox(pv_e, ABS_ERR));
    }
    {
        ChMatrix66d sp = (S * P).eigen();
        ChMatrix66d sp_e = S.eigen() * P.eigen();
#ifdef DBG_PRINT
        cout << "\nS * P" << endl;
        cout << sp << endl;
        cout << "---" << endl;
        cout << sp_e << endl;
        cout << "||diff|| = " << (sp - sp_e).norm() << endl;
#endif
        ASSERT_NEAR((sp - sp_e).norm(), 0.0, ABS_ERR);
        ////ASSERT_TRUE(sp.isApprox(sp_e, ABS_ERR));
    }
    {
        ChMatrix66d ps = (~P * S).eigen();
        ChMatrix66d ps_e = P.eigen().transpose() * S.eigen();
#ifdef DBG_PRINT
        cout << "\nP^T * S" << endl;
        cout << ps << endl;
        cout << "---" << endl;
        cout << ps_e << endl;
        cout << "||diff|| = " << (ps - ps_e).norm() << endl;
#endif
        ASSERT_NEAR((ps - ps_e).norm(), 0.0, ABS_ERR);
        ////ASSERT_TRUE(ps.isApprox(ps_e, ABS_ERR));
    }
    {
        ChSpatialVec hv = H1 * v;
        ChVector6 hv_e = H1.eigen() * v;
#ifdef DBG_PRINT
        cout << "\nH * v" << endl;
        cout << hv << endl;
        cout << "---" << endl;
        cout << hv_e << endl;
        cout << "||diff|| = " << (hv - hv_e).norm() << endl;
#endif
        ASSERT_NEAR((hv - hv_e).norm(), 0.0, ABS_ERR);
        ////ASSERT_TRUE(hv.isApprox(hv_e, ABS_ERR));
    }
    {
        ChVectorN<double, DOF> hv = ~H1 * V;
        ChVectorN<double, DOF> hv_e = H1.eigen().transpose() * V;
#ifdef DBG_PRINT
        cout << "\nH^T * V" << endl;
        cout << hv << endl;
        cout << "---" << endl;
        cout << hv_e << endl;
        cout << "||diff|| = " << (hv - hv_e).norm() << endl;
#endif
        ASSERT_NEAR((hv - hv_e).norm(), 0.0, ABS_ERR);
        ////ASSERT_TRUE(hv.isApprox(hv_e, ABS_ERR));
    }
    {
        ChMatrix6N<DOF> hm = (H1 * M).eigen();
        ChMatrix6N<DOF> hm_e = H1.eigen() * M;
#ifdef DBG_PRINT
        cout << "H * M" << endl;
        cout << hm << endl;
        cout << "---" << endl;
        cout << hm_e << endl;
        cout << "||diff|| = " << (hm - hm_e).norm() << endl;
#endif
        ASSERT_NEAR((hm - hm_e).norm(), 0.0, ABS_ERR);
        ////ASSERT_TRUE(hm.isApprox(hm_e, ABS_ERR));
    }
    {
        ChMatrix6N<DOF> sh = (S * H1).eigen();
        ChMatrix6N<DOF> sh_e = S.eigen() * H1.eigen();
#ifdef DBG_PRINT
        cout << "S * H" << endl;
        cout << sh << endl;
        cout << "---" << endl;
        cout << sh_e << endl;
        cout << "||diff|| = " << (sh - sh_e).norm() << endl;
#endif
        ASSERT_NEAR((sh - sh_e).norm(), 0.0, ABS_ERR);
        ////ASSERT_TRUE(sh.isApprox(sh_e, ABS_ERR));
    }
    ////{ auto rh = (R * H1).eigen(); }
    {
        ChMatrix66d hh = (H1 * ~H2).eigen();
        ChMatrix66d hh_e = H1.eigen() * H2.eigen().transpose();
#ifdef DBG_PRINT
        cout << "H * H" << endl;
        cout << hh << endl;
        cout << "---" << endl;
        cout << hh_e << endl;
        cout << "||diff|| = " << (hh - hh_e).norm() << endl;
#endif
        ASSERT_NEAR((hh - hh_e).norm(), 0.0, ABS_ERR);
        ////ASSERT_TRUE(hh.isApprox(hh_e, ABS_ERR));
    }
    {
        ChMatrixNN<DOF> hh = ~H1 * H2;
        ChMatrixNN<DOF> hh_e = H1.eigen().transpose() * H2.eigen();
#ifdef DBG_PRINT
        cout << "H^T * H" << endl;
        cout << hh << endl;
        cout << "---" << endl;
        cout << hh_e << endl;
        cout << "||diff|| = " << (hh - hh_e).norm() << endl;
#endif
        ASSERT_NEAR((hh - hh_e).norm(), 0.0, ABS_ERR);
        ////ASSERT_TRUE(hh.isApprox(hh_e, ABS_ERR));
    }
}
