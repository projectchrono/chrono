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

#include "chrono/physics/ChSystemNSC.h"
#include "chrono_parsers/ChParserURDF.h"

using namespace chrono;
using namespace chrono::parsers;

void CheckBox(ChSystemNSC& system, ChParserURDF& parser, const std::string& name) {
    auto body = parser.GetChBody(name);
    EXPECT_TRUE(body != nullptr);
    EXPECT_TRUE(system.SearchBody(name) == body);
    EXPECT_TRUE(body->GetCollisionModel()->GetShapeInstances().size() == 1);
}

TEST(ChParserURDF, URDF_simple) {
    ChSystemNSC system;

    // Create the URDF parser with a basic file
    const std::string filename = "urdf/box.urdf";
    ChParserURDF parser(GetChronoDataFile(filename));
    parser.PopulateSystem(system);

    // If parsing succeed (which it should), the model will be set
    EXPECT_TRUE(parser.GetModelTree() != nullptr);
    // And the box's bodies will be in the system
    CheckBox(system, parser, "box0");
    CheckBox(system, parser, "box1");
    CheckBox(system, parser, "box2");
}
