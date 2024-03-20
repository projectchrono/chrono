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
// Authors: Radu Serban
// =============================================================================
//
// Demo for working with Eigen references.
// (Mostly for Chrono developers)
//
// =============================================================================

#include <iostream>
#include <vector>

#include "chrono/core/ChMatrix.h"
#include "chrono/core/ChSparsityPatternLearner.h"

#include "gtest/gtest.h"

const double precision = 1e-8;

using std::cout;
using std::endl;
using namespace chrono;

// ------------------------------------------------------------------

TEST(SparseMatrix, pattern_learner) {
    ChSparsityPatternLearner spl(3, 3);

    spl.SetElement(0, 0, 1.1);
    spl.SetElement(1, 1, 2.2);
    spl.SetElement(1, 0, 2.1);
    spl.SetElement(2, 2, 3.3);

    ChSparseMatrix spmat_learned;
    spl.Apply(spmat_learned);

    ASSERT_TRUE(spmat_learned.outerIndexPtr()[0] == 0);
    ASSERT_TRUE(spmat_learned.outerIndexPtr()[1] == 1);
    ASSERT_TRUE(spmat_learned.outerIndexPtr()[2] == 3);
    ASSERT_TRUE(spmat_learned.outerIndexPtr()[3] == 4);
    ASSERT_TRUE(spmat_learned.innerIndexPtr()[0] == 0);
    ASSERT_TRUE(spmat_learned.innerIndexPtr()[1] == 0);
    ASSERT_TRUE(spmat_learned.innerIndexPtr()[2] == 1);
    ASSERT_TRUE(spmat_learned.innerIndexPtr()[3] == 2);

    ChSparseMatrix spmat_mirror(3, 3);

    spmat_mirror.SetElement(0, 0, 1.1);
    spmat_mirror.SetElement(1, 1, 2.2);
    spmat_mirror.SetElement(1, 0, 2.1);
    spmat_mirror.SetElement(2, 2, 3.3);
    spmat_mirror.makeCompressed();

    ASSERT_TRUE(spmat_learned.outerIndexPtr()[0] == spmat_mirror.outerIndexPtr()[0]);
    ASSERT_TRUE(spmat_learned.outerIndexPtr()[1] == spmat_mirror.outerIndexPtr()[1]);
    ASSERT_TRUE(spmat_learned.outerIndexPtr()[2] == spmat_mirror.outerIndexPtr()[2]);
    ASSERT_TRUE(spmat_learned.outerIndexPtr()[3] == spmat_mirror.outerIndexPtr()[3]);
    ASSERT_TRUE(spmat_learned.innerIndexPtr()[0] == spmat_mirror.innerIndexPtr()[0]);
    ASSERT_TRUE(spmat_learned.innerIndexPtr()[1] == spmat_mirror.innerIndexPtr()[1]);
    ASSERT_TRUE(spmat_learned.innerIndexPtr()[2] == spmat_mirror.innerIndexPtr()[2]);
    ASSERT_TRUE(spmat_learned.innerIndexPtr()[3] == spmat_mirror.innerIndexPtr()[3]);

    ASSERT_NEAR(spmat_mirror.valuePtr()[0], 1.1, precision);
    ASSERT_NEAR(spmat_mirror.valuePtr()[1], 2.1, precision);
    ASSERT_NEAR(spmat_mirror.valuePtr()[2], 2.2, precision);
    ASSERT_NEAR(spmat_mirror.valuePtr()[3], 3.3, precision);
}
