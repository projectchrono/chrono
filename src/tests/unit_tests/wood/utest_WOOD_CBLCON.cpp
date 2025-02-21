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

#include "gtest/gtest.h"

#include "chrono_wood/ChElementCBLCON.h"


using namespace chrono;
using namespace wood;

TEST(CBLConnectorTest, Amatrix) {
    ChElementCBLCON connector;
    ChMatrixNM<double,3,6> A;
    connector.ComputeAmatrix(A, ChVector3d(0.0, 0.0, 0.0), ChVector3d(1.0, 2.0, 3.0));
    ASSERT_DOUBLE_EQ(A(0,0), 1.0);
    ASSERT_DOUBLE_EQ(A(1,1), 1.0);
    ASSERT_DOUBLE_EQ(A(2,2), 1.0);
    ASSERT_DOUBLE_EQ(A(0,4), -3.0);
    ASSERT_DOUBLE_EQ(A(0,5), 2.0);
    ASSERT_DOUBLE_EQ(A(1,3), 3.0);
    ASSERT_DOUBLE_EQ(A(1,5), -1.0);
    ASSERT_DOUBLE_EQ(A(2,3), -2.0);
    ASSERT_DOUBLE_EQ(A(2,4), 1.0);
}
