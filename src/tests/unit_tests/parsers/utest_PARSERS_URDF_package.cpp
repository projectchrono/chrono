// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Aaron Young
// =============================================================================
//
// Test to check that when loading a URDF file, the relative filenames are
// resolved correctly.
//
// =============================================================================

#include "gtest/gtest.h"

#include "chrono_parsers/ChParserURDF.h"

using namespace chrono;
using namespace chrono::parsers;

TEST(ChParserURDF, URDF_package) {
    // Create the URDF parser with a file that contains a filename with package://
    const std::string filename = "robot/r2d2/r2d2-package.urdf";
    ChParserURDF parser(GetChronoDataFile(filename));

#ifdef HAVE_ROS
    EXPECT_TRUE(parser.GetModelTree() != nullptr);
#else
    EXPECT_TRUE(parser.GetModelTree() == nullptr);
#endif
}
