// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
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
// Demo for the URDF -> Chrono parser
//
// =============================================================================

#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/core/ChRealtimeStep.h"

#include "chrono_parsers/ChParserURDF.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

using namespace chrono;
using namespace chrono::parsers;
using namespace chrono::irrlicht;

int main(int argc, char* argv[]) {
    std::string filename = (argc > 1) ? std::string(argv[1]) : "robot/robosimian/rs.urdf";

    // Make a system
    ChSystemSMC sys;

    // Create parser instance and set options.
    ChParserURDF parser;
    parser.Parse(sys, GetChronoDataFile(filename));

    // Get a report on parsed elements
    parser.PrintModelTree();

    return 0;
}
