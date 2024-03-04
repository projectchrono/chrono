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

TEST(ChParserURDF, URDF_simple) {
    // Create the URDF parser with a basic file
    const std::string filename = "urdf/box.urdf";
    ChParserURDF parser(GetChronoDataFile(filename));

    // If parsing succeed (which it should), the model will be set
    EXPECT_TRUE(parser.GetModelTree() != nullptr);
}
